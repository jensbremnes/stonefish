/*
    This file is a part of Stonefish.

    Stonefish is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Stonefish is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  AquacultureTestApp.cpp
//  Stonefish
//

#include "AquacultureTestApp.h"
#include "AquacultureTestManager.h"

#include <core/Console.h>
#include <core/Robot.h>
#include <core/SimulationManager.h>
#include <graphics/IMGUI.h>
#include <sensors/Sample.h>
#include <sensors/scalar/Multibeam.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
    //================================= NET HOLD =================================
    //
    //Two loops, and neither of them fights the autopilot the base class already has.
    //
    //  standoff   the fitted perpendicular distance to the net    ->  surge
    //  bearing    the fitted bearing of the net normal            ->  the HEADING SETPOINT
    //
    //SWAY AND DEPTH ARE LEFT ALONE. Those are the axes that fly along the net, and flying
    //along the net IS the inspection - it is where the pilot decides what to look at and how
    //long to look at it. What the mode takes over is the two axes that are pure bookkeeping:
    //keeping the right distance off a curved surface in a 0.3 m/s current, and staying square
    //to it. That is the same division of labour as the inherited altitude hold, which holds a
    //height off the seabed and leaves the pilot flying over it.
    //
    //The bearing loop writes headingSp_ rather than yaw_ on purpose. The heading hold is
    //already tuned, already takes its D term off the IMU rate, and already degrades
    //gracefully; re-deriving all of that for a second yaw controller would be a way of having
    //two of them disagree. The cost is one frame of lag, because AugmentDemands runs after
    //RunHolds, which at 10 fps is a tenth of a second of stale heading command.

    //Kp is the ALTITUDE loop's gain, not the depth loop's: both hold a range off a ranging
    //sensor against a surface that is not flat, which is a different problem from holding a
    //depth off a pressure sensor.
    const sf::Scalar STANDOFF_KP = 0.50;
    //Ki has to trim a standing 0.30 m/s current, not just a bias, so it is larger than the
    //vertical loop's 0.03.
    const sf::Scalar STANDOFF_KI = 0.06;
    const sf::Scalar STANDOFF_KD = 0.45;
    const sf::Scalar STANDOFF_IBAND = 0.80;   //!< m
    const sf::Scalar STANDOFF_ILIMIT = 0.30;
    //Below the manual sway cap, so the pilot always out-authorities the range loop and can
    //fly along the net faster than the loop can pull the vehicle off it.
    const sf::Scalar STANDOFF_LIMIT = 0.35;

    //THE D TERM IS TAKEN ON THE MEASUREMENT, which the under-ice altitude loop specifically
    //must not do. The difference is real and not a relaxation of that lesson: that loop
    //differentiated a 10 Hz altitude built from FOUR slant beams crossing a rough ice
    //underside, which stepped by tens of centimetres as the beams moved. This is a
    //least-squares fit of 49 beams against a smooth shell, sampled at 10 Hz, and its noise is
    //the fit residual divided by sqrt(49). There is also no alternative here: the DVL reports
    //velocity only on a bottom ping, and the bottom is 45 m below the pen.
    const sf::Scalar DIST_RATE_TAU = 0.25;

    const sf::Scalar STANDOFF_MIN = 0.80;     //!< m
    const sf::Scalar STANDOFF_MAX = 6.00;     //!< m
    const sf::Scalar STANDOFF_DEFAULT = 1.50; //!< m
    //The same rate the inherited depth setpoint moves at, so W/S here feels like Q/E there.
    const sf::Scalar SP_RATE_STANDOFF = 0.30; //!< m/s
    const sf::Scalar STANDOFF_STEP = 0.25;    //!< m, for the [ and ] trim keys

    //How far off the net normal the bow can be held. Beyond about 45 degrees the net leaves
    //the bow camera's 40 degree half angle, and an inspection that cannot see the net is not
    //one; beyond about 60 the fan stops straddling and the fit stops being steerable.
    const sf::Scalar LOOK_MAX = 45.0 * M_PI / 180.0;
    const sf::Scalar SP_RATE_LOOK = 20.0 * M_PI / 180.0;  //!< rad/s

    //Largest heading correction commanded in one go. Bounds what a bad fit can ask for.
    const sf::Scalar MAX_TURN = 60.0 * M_PI / 180.0;

    //A beam within this of its maximum has no return at all.
    const sf::Scalar NO_RETURN_MARGIN = 0.30; //!< m
    //A beam this far outside the fitted plane is not on the net.
    const sf::Scalar FIT_RESIDUAL = 0.45;     //!< m
    const unsigned int MIN_GOOD_BEAMS = 12;
    //Beams on the net must reach at least this far either side of the boresight before the
    //fitted bearing is trusted to steer with. A group of returns all on one side fits a line
    //perfectly well and points it in almost any direction, which is how a vehicle can be
    //walked off the net by its own controller.
    const sf::Scalar STRADDLE_MIN = 0.25;     //!< m
    //Closer than this and nothing else matters.
    const sf::Scalar TOO_CLOSE = 0.60;        //!< m
    //Below the cylindrical wall a plane fit is meaningless: the cone is 14.7 degrees from
    //horizontal, so the fan looks along it rather than at it.
    const sf::Scalar CONE_DEPTH_MARGIN = 1.0; //!< m above the wall's bottom
    const sf::Scalar LOST_TIMEOUT = 3.0;      //!< s before the mode releases itself

    //HUD geometry, in the columns the other two demos established.
    const GLfloat COL_X = 10.f;
    const GLfloat COL_W = 250.f;
    const GLfloat NET_PANEL_Y = 510.f;
    const GLfloat NET_PANEL_H = 112.f;
    const GLfloat PROFILE_PLOT_Y = 632.f;
    const GLfloat TRACE_PLOT_Y = 756.f;
    const GLfloat PLOT_H = 112.f;
    const sf::Scalar PLOT_RANGE_MAX = 6.0;    //!< m, the vertical scale of both plots
    const size_t TRACE_LEN = 400;

    //! 1 while the key is held, 0 otherwise. Held keys have to be polled: SDL delivers one
    //! key-down event and then a repeat stream, neither of which is a "still held".
    inline sf::Scalar Held(const Uint8* keys, SDL_Scancode code)
    {
        return keys[code] ? sf::Scalar(1) : sf::Scalar(0);
    }
}

AquacultureTestApp::AquacultureTestApp(std::string dataDirPath, sf::RenderSettings s,
                                       sf::HelperSettings h, sf::SimulationManager* sim)
    : BlueROV2TestApp("Salmon farm net inspection", dataDirPath, s, h, sim),
      payloadResolved_(false), autoEngaged_(false), profiler_(nullptr),
      netHold_(NetHold::ARMED), netState_(NetState::NO_DATA),
      standoffSp_(STANDOFF_DEFAULT), lookSp_(0), standoffSpRate_(0), lostFor_(0),
      distRate_(DIST_RATE_TAU),
      standoffPid_(STANDOFF_KP, STANDOFF_KI, STANDOFF_KD,
                   STANDOFF_IBAND, STANDOFF_ILIMIT, STANDOFF_LIMIT)
{
    fix_.valid = false;
    fix_.distance = 0;
    fix_.bearing = 0;
    fix_.chord = 0;
    fix_.good = 0;
    fix_.missing = 0;
    fix_.straddles = false;
    fix_.hole = false;
    fix_.holeBearing = 0;

    profilePlot_.resize(1);
    standoffPlot_.resize(2);
}

void AquacultureTestApp::ResolvePayload()
{
    payloadResolved_ = true;

    sf::Robot* rov = getSimulationManager()->getRobot(farm::ROBOT);
    if(rov == nullptr)
    {
        cError("Robot '%s' not found - net hold is unavailable.", farm::ROBOT);
        return;
    }

    //Sensors attached from C++ are named WITHOUT the robot prefix; the parser prefixes
    //everything it builds. Only the payload is looked up here, so the bare name is enough.
    profiler_ = dynamic_cast<sf::Multibeam*>(rov->getSensor(farm::NET_PROFILER));
    if(profiler_ == nullptr)
        cError("Net profiler '%s' not found - net hold is unavailable.", farm::NET_PROFILER);

    //The pilot view is the scenario bow camera, which the base class resolves for itself -
    //there is no second camera in this demo. DO NOT clear cameraResolved_ in order to point
    //camera_ somewhere else: ApplyCameraView() re-looks-up the scenario camera by name
    //whenever that flag is false, so anything assigned to camera_ here is overwritten on the
    //next frame and the inset goes black.
}

//! Reads the net profiler and fits a straight net to it.
//!
//! The fan lies in the vehicle's horizontal plane, beam i at (i/steps - 0.5) * fov, positive
//! to starboard. Each beam with a return gives a point; a total-least-squares line through
//! those points gives the net's normal, and the normal gives both the perpendicular standoff
//! and the bearing the vehicle has to turn through to be square to it.
//!
//! Three things that are not obvious:
//!
//!   - TWO PASSES. The first fits every beam that returned; the second refits only the beams
//!     that landed on the fitted plane. One pass is not enough because a beam that goes
//!     through the tear and hits something on the far side, or clips a lifting strap, is a
//!     leverage point on a total-least-squares fit and drags the whole plane with it.
//!
//!   - THE TARGET IS A CYLINDER, NOT A PLANE. The fitted line is a chord of a 19.1 m circle,
//!     so it sits a sagitta of chord^2/(8R) FARTHER away than the nearest point of the net.
//!     At a 1.5 m standoff the fan spans about 3 m of net and that is 6 cm, which is 4 % of
//!     the standoff - small, but it is a bias and not noise, so it is removed rather than
//!     tuned out. The bearing is unbiased by symmetry and is left alone.
//!
//!   - THE PROFILER IS NOT AT THE VEHICLE ORIGIN. It sits 0.20 m forward, so the vehicle's
//!     own standoff is the sensor's plus 0.20*cos(bearing).
//!
//! A run of beams with no return, flanked by beams that do have one, is a hole in the net -
//! which is what the tear reads as, because the tear's collision shell carries the same hole
//! as its twine.
AquacultureTestApp::NetFix AquacultureTestApp::ReadNet() const
{
    NetFix f;
    f.valid = false;
    f.distance = 0;
    f.bearing = 0;
    f.chord = 0;
    f.good = 0;
    f.missing = 0;
    f.straddles = false;
    f.hole = false;
    f.holeBearing = 0;

    if(profiler_ == nullptr)
        return f;

    const unsigned short n = profiler_->getNumOfChannels();
    if(n < 3)
        return f;

    const sf::Sample s = profiler_->getLastSample();
    const double half = 0.5 * farm::PROFILER_FOV * M_PI / 180.0;

    std::vector<double> px(n, 0.0), py(n, 0.0), ang(n, 0.0);
    std::vector<bool> hit(n, false);

    for(unsigned short i=0; i<n; ++i)
    {
        const double a = (double)i/(double)(n-1) * 2.0 * half - half;
        const double r = (double)s.getValue(i);
        ang[i] = a;
        if(r <= 0.0 || r >= farm::PROFILER_RANGE_MAX - NO_RETURN_MARGIN)
        {
            ++f.missing;
            continue;
        }
        hit[i] = true;
        px[i] = r * std::cos(a);
        py[i] = r * std::sin(a);
        ++f.good;
    }

    if(f.good < MIN_GOOD_BEAMS)
        return f;

    std::vector<bool> off(n, false);
    double nx = 1.0, ny = 0.0, dSensor = 0.0;
    double tmin = 0.0, tmax = 0.0;
    unsigned int onNet = 0;

    for(int pass=0; pass<2; ++pass)
    {
        double cx = 0.0, cy = 0.0;
        unsigned int used = 0;
        for(unsigned short i=0; i<n; ++i)
        {
            if(!hit[i] || off[i]) continue;
            cx += px[i]; cy += py[i]; ++used;
        }
        if(used < MIN_GOOD_BEAMS)
            return f;
        cx /= used; cy /= used;

        double sxx = 0.0, sxy = 0.0, syy = 0.0;
        for(unsigned short i=0; i<n; ++i)
        {
            if(!hit[i] || off[i]) continue;
            const double dx = px[i] - cx, dy = py[i] - cy;
            sxx += dx*dx;
            sxy += dx*dy;
            syy += dy*dy;
        }

        //Principal direction of the point cloud: the net's tangent. Its perpendicular is the
        //normal, flipped if necessary so that it points from the vehicle towards the net.
        const double theta = 0.5 * std::atan2(2.0*sxy, sxx - syy);
        nx = -std::sin(theta);
        ny = std::cos(theta);
        if(nx*cx + ny*cy < 0.0) { nx = -nx; ny = -ny; }
        dSensor = nx*cx + ny*cy;

        tmin = 1e9; tmax = -1e9;
        onNet = 0;
        for(unsigned short i=0; i<n; ++i)
        {
            if(!hit[i]) continue;
            if(std::fabs(nx*px[i] + ny*py[i] - dSensor) > FIT_RESIDUAL)
            {
                off[i] = true;
                continue;
            }
            off[i] = false;
            const double t = -ny*px[i] + nx*py[i];
            tmin = std::min(tmin, t);
            tmax = std::max(tmax, t);
            ++onNet;
        }
        if(onNet < MIN_GOOD_BEAMS)
            return f;
    }

    f.bearing = (sf::Scalar)std::atan2(ny, nx);
    f.chord = (sf::Scalar)(tmax - tmin);
    f.straddles = (tmin < -STRADDLE_MIN && tmax > STRADDLE_MIN);

    //The cylinder correction. Positive sagitta means the fitted chord is farther than the net.
    const double sagitta = (f.chord * f.chord) / (8.0 * farm::PEN_RADIUS);
    f.distance = (sf::Scalar)(dSensor - sagitta
                              + farm::PROFILER_X * std::cos((double)f.bearing));
    f.valid = true;

    //A hole: a run of at least two beams with no return or off the plane, with a good beam on
    //both sides of it. A run that touches either end of the fan is the net simply ending -
    //which is what happens at the top of the wall and at the sector edges - not a hole.
    unsigned int run = 0;
    double runAng = 0.0;
    bool sawGood = false;
    for(unsigned short i=0; i<n; ++i)
    {
        const bool bad = !hit[i] || off[i];
        if(bad)
        {
            if(sawGood) { ++run; runAng += ang[i]; }
        }
        else
        {
            if(run >= 2 && sawGood)
            {
                f.hole = true;
                f.holeBearing = (sf::Scalar)(runAng / run);
            }
            run = 0;
            runAng = 0.0;
            sawGood = true;
        }
    }

    return f;
}

void AquacultureTestApp::SetNetHold(NetHold m)
{
    if(m == netHold_)
        return;

    netHold_ = m;
    lostFor_ = 0;
    standoffSpRate_ = 0;

    if(m == NetHold::ON)
    {
        //The heading hold is needed too: the bearing loop drives its setpoint, so that axis
        //cannot be left open loop. The vertical hold is left exactly as the pilot had it -
        //depth, altitude or off - because that axis stays theirs.
        SetHeadingHold(true);
        standoffPid_.Reset();
        distRate_.Reset();
        //Engage at the distance the vehicle is already at rather than at whatever the
        //setpoint was left on, so engaging never pulls the vehicle anywhere.
        if(fix_.valid)
            standoffSp_ = std::max(STANDOFF_MIN, std::min(STANDOFF_MAX, fix_.distance));
        lookSp_ = 0;
        cInfo("NET HOLD on at %.2f m. W/S move the standoff, A/D the look angle; "
              "sway and depth stay yours.", (double)standoffSp_);
    }
    else
    {
        //Park the heading setpoint where the vehicle is, so releasing the mode does not leave
        //the inherited heading hold flying back to a stale target.
        if(state_.valid)
            headingSp_ = state_.yaw;
        cInfo("NET HOLD off - every axis is manual again.");
    }
}

//! Moves the setpoints of the two keys the mode borrows.
//!
//! This is the same idea as the inherited holds: with a mode engaged, the motion key on that
//! axis stops commanding thrust and moves the target instead, at a rate a pilot can aim. W
//! and S are the surge keys and surge is what the standoff loop owns, so W and S move the
//! standoff; A and D are the yaw keys and yaw is what the bearing loop owns, so A and D swing
//! the bow off the net normal. Neither key does anything else while the mode is on, because
//! AugmentDemands overwrites surge_ and headingSp_ after ReadKeyboard has already run.
void AquacultureTestApp::UpdateNetSetpoints(sf::Scalar dt)
{
    const Uint8* keys = SDL_GetKeyboardState(nullptr);

    //W closes in, S backs off: the same sense the keys have when they command surge.
    const sf::Scalar in = Held(keys, SDL_SCANCODE_W) - Held(keys, SDL_SCANCODE_S);
    standoffSpRate_ = -in * SP_RATE_STANDOFF;
    standoffSp_ = std::max(STANDOFF_MIN,
                           std::min(STANDOFF_MAX, standoffSp_ + standoffSpRate_ * dt));

    const sf::Scalar look = Held(keys, SDL_SCANCODE_D) - Held(keys, SDL_SCANCODE_A);
    lookSp_ = std::max(-LOOK_MAX, std::min(LOOK_MAX, lookSp_ + look * SP_RATE_LOOK * dt));
}

void AquacultureTestApp::ProcessInputs()
{
    if(!payloadResolved_)
        ResolvePayload();

    BlueROV2TestApp::ProcessInputs();
}

//! Runs the net hold loops. Called between the inherited holds and thrust allocation.
void AquacultureTestApp::AugmentDemands(sf::Scalar dt)
{
    fix_ = ReadNet();

    //--- classify -----------------------------------------------------------------------
    if(profiler_ == nullptr || (fix_.good == 0 && fix_.missing == 0))
        netState_ = NetState::NO_DATA;
    else if(state_.valid && state_.depth > farm::PEN_WALL_DEPTH - CONE_DEPTH_MARGIN)
        netState_ = NetState::CONE;
    else if(!fix_.valid)
        netState_ = NetState::LOST;
    else if(fix_.distance < TOO_CLOSE)
        netState_ = NetState::TOO_CLOSE;
    else if(fix_.hole)
        netState_ = NetState::PARTIAL;
    else
        netState_ = NetState::GOOD;

    //--- engage, once ---------------------------------------------------------------------
    //
    //The mode engages itself on the first good fix rather than waiting for a key, for the
    //same reason UnderIceTest flies its ice standoff from the first frame: without it the
    //scenario does not work unattended. The vehicle is +2.0 N buoyant and sits in a 0.30 m/s
    //current with every axis open, so left alone it simply drifts off the pen - it reaches
    //6.5 m of standoff inside a minute, which is beyond the visibility of this water. N
    //releases it, and everything the base class does is then unchanged.
    if(!autoEngaged_ && netState_ == NetState::GOOD && state_.valid)
    {
        autoEngaged_ = true;
        SetNetHold(NetHold::ON);
    }

    if(netHold_ != NetHold::ON)
    {
        lostFor_ = 0;
        standoffSpRate_ = 0;
        return;
    }

    UpdateNetSetpoints(dt);

    //--- the degradation ladder -----------------------------------------------------------
    //
    //A PARTIAL fix is NOT a failure. The tear is a run of missing beams with good beams on
    //both sides of it, and the surrounding beams still fix the net plane perfectly well - so
    //the loop keeps flying and the HUD reports a hole. Only a fit that cannot be made at all,
    //for long enough that it is not a transient, releases the mode.
    if(netState_ == NetState::LOST || netState_ == NetState::NO_DATA
       || netState_ == NetState::CONE)
    {
        lostFor_ += dt;
        //Ramp the surge demand out rather than dropping it, and leave sway and the inherited
        //depth and heading holds carrying on exactly as they were.
        surge_ *= 0.85;
        if(lostFor_ > LOST_TIMEOUT)
        {
            cWarning("Net lost for %.1f s - NET HOLD released, every axis is manual again.",
                     (double)lostFor_);
            SetNetHold(NetHold::ARMED);
        }
        return;
    }
    lostFor_ = 0;

    //--- the standoff loop ----------------------------------------------------------------
    //
    //PIDHold takes error as SETPOINT MINUS MEASUREMENT and subtracts kd times the
    //measurement's rate, so its output is positive when the vehicle is too CLOSE. The normal
    //demand is that negated: positive aNormal moves towards the net. Written this way round
    //rather than by flipping the error, so that the D term still damps the vehicle's own
    //motion instead of reinforcing it.
    //
    //The setpoint's own slew rate is subtracted from the measured rate, for the same reason
    //the inherited loops carry depthSpRate_: while the pilot holds W the distance is SUPPOSED
    //to be changing, and damping that is what makes a hold fight its own pilot and then
    //overshoot when the key comes up.
    const sf::Scalar error = standoffSp_ - fix_.distance;
    const sf::Scalar rate = distRate_.Update(fix_.distance, dt) - standoffSpRate_;
    sf::Scalar aNormal = -standoffPid_.Update(error, rate, dt);

    //Too close overrides everything: back straight out.
    if(netState_ == NetState::TOO_CLOSE)
    {
        aNormal = -0.25;
        standoffPid_.Reset();
    }

    //--- the bearing loop -----------------------------------------------------------------
    //
    //THE SETPOINT IS AN ABSOLUTE HEADING, NOT A PER-FRAME NUDGE. Writing
    //headingSp_ = yaw + 30 deg/s * dt looks like a 30 deg/s slew and is not one: it
    //re-anchors the setpoint to wherever the vehicle has got to on every frame, so the
    //heading hold never sees more than a degree of error, answers it with about a hundredth
    //of its yaw authority, and - thrust being quadratic - turns at half a degree a second. A
    //vehicle 60 degrees off the net took minutes to come back, which read as the mode having
    //given up. The whole error is commanded instead, and the heading hold's own tuning
    //provides the rate; it is the loop that was measured at a 90 degree turn with no
    //overshoot.
    const sf::Scalar want = brov2::WrapPi(fix_.bearing - lookSp_);
    //A one-sided fit still points roughly the right way - a grazing line fit has a normal
    //perpendicular to the surface, which is the direction that brings the net back across the
    //boresight - but it is the fit least worth trusting, so it gets a third of the correction
    //rather than being ignored. Ignoring it is worse: freezing the heading leaves the vehicle
    //flying along the net with no way back.
    const sf::Scalar bite = fix_.straddles ? sf::Scalar(1.0) : sf::Scalar(0.35);
    headingSp_ = brov2::WrapPi(state_.yaw + bite * std::max(-MAX_TURN, std::min(MAX_TURN, want)));

    //--- write the demand -----------------------------------------------------------------
    //
    //Only the component along the net's normal. The pilot's sway is left in place, which is
    //what makes this a hold rather than a survey: the vehicle is driven ALONG the net by
    //hand, at whatever speed and in whichever direction, and the loop simply keeps the
    //distance and the alignment while that happens.
    //
    //The demand is resolved along the MEASURED net normal rather than along the bow, so that
    //while the heading loop is still settling - or while the pilot is holding a look angle
    //with A or D - the vehicle still moves perpendicular to the net rather than at an angle
    //to it. That is also why sway is ADDED to rather than overwritten.
    surge_ = aNormal * std::cos(fix_.bearing);
    sway_ += aNormal * std::sin(fix_.bearing);
}

void AquacultureTestApp::KeyDown(SDL_Event* event)
{
    //Consumed before the base class's swallow list, the same way Z is, so they work whether
    //the keyboard is flying the vehicle or moving the camera.
    switch(event->key.keysym.sym)
    {
        case SDLK_n:
            SetNetHold(netHold_ == NetHold::ON ? NetHold::ARMED : NetHold::ON);
            return;

        //Discrete trim on the standoff, for when a quarter of a metre is wanted exactly
        //rather than held for.
        case SDLK_LEFTBRACKET:
            standoffSp_ = std::max(STANDOFF_MIN, standoffSp_ - STANDOFF_STEP);
            cInfo("Standoff setpoint: %.2f m.", (double)standoffSp_);
            return;

        case SDLK_RIGHTBRACKET:
            standoffSp_ = std::min(STANDOFF_MAX, standoffSp_ + STANDOFF_STEP);
            cInfo("Standoff setpoint: %.2f m.", (double)standoffSp_);
            return;

        default:
            break;
    }

    BlueROV2TestApp::KeyDown(event);
}

void AquacultureTestApp::DoTelemetry()
{
    BlueROV2TestApp::DoTelemetry();

    getGUI()->DoPanel(COL_X, NET_PANEL_Y, COL_W, NET_PANEL_H);

    char buf[128];
    const GLfloat tx = COL_X + 10.f;
    GLfloat ty = NET_PANEL_Y + 12.f;

    const char* state = "--";
    switch(netState_)
    {
        case NetState::NO_DATA:   state = "no data"; break;
        case NetState::LOST:      state = "NET LOST"; break;
        case NetState::TOO_CLOSE: state = "TOO CLOSE"; break;
        case NetState::CONE:      state = "CONE - no fit"; break;
        case NetState::PARTIAL:   state = "HOLE IN NET"; break;
        case NetState::GOOD:      state = "net"; break;
    }
    snprintf(buf, sizeof(buf), "NET HOLD [N]: %s - %s",
             netHold_ == NetHold::ON ? "ON" : "off", state);
    getGUI()->DoLabel(tx, ty, std::string(buf));
    ty += 22.f;

    if(fix_.valid)
        snprintf(buf, sizeof(buf), "Standoff %5.2f / %4.2f m  [W/S]", (double)fix_.distance,
                 (double)standoffSp_);
    else
        snprintf(buf, sizeof(buf), "Standoff    -- / %4.2f m  [W/S]", (double)standoffSp_);
    getGUI()->DoLabel(tx, ty, std::string(buf));
    ty += 20.f;

    if(fix_.valid)
        snprintf(buf, sizeof(buf), "Net brg %+5.1f    Look %+3.0f deg [A/D]",
                 (double)fix_.bearing * 180.0 / M_PI, (double)lookSp_ * 180.0 / M_PI);
    else
        snprintf(buf, sizeof(buf), "Net brg    --    Look %+3.0f deg [A/D]",
                 (double)lookSp_ * 180.0 / M_PI);
    getGUI()->DoLabel(tx, ty, std::string(buf));
    ty += 20.f;

    snprintf(buf, sizeof(buf), "Beams %2u on net, %2u missing%s", fix_.good, fix_.missing,
             (fix_.valid && !fix_.straddles) ? "  1-SIDED" : "");
    getGUI()->DoLabel(tx, ty, std::string(buf));
    ty += 20.f;

    getGUI()->DoLabel(tx, ty, "Fly along the net on , and .");
}

void AquacultureTestApp::DoHUD()
{
    BlueROV2TestApp::DoHUD();

    sf::Uid id;
    id.owner = 12;

    sf::Scalar range[2] = { sf::Scalar(0), PLOT_RANGE_MAX };

    //--- the net profile across the fan ------------------------------------------------------
    //One frame of the profiler, as a profile rather than a time series: beam i at position i,
    //port on the left. A beam with no return is drawn at full scale, so a hole in the net is a
    //notch that goes off the top of the box rather than a gap in the trace.
    if(profiler_ != nullptr)
    {
        const sf::Sample s = profiler_->getLastSample();
        const unsigned short n = profiler_->getNumOfChannels();
        profilePlot_[0].resize(n);
        for(unsigned short i=0; i<n; ++i)
        {
            const sf::Scalar r = s.getValue(i);
            const bool miss = (r <= 0 || r >= farm::PROFILER_RANGE_MAX - NO_RETURN_MARGIN);
            profilePlot_[0][i] = (GLfloat)(miss ? PLOT_RANGE_MAX
                                                : std::min(PLOT_RANGE_MAX, (sf::Scalar)r));
        }
        id.item = 0;
        getGUI()->DoTimePlot(id, COL_X, PROFILE_PLOT_Y, COL_W, PLOT_H, profilePlot_,
                             "NET PROFILE ACROSS FAN 0-6 m (port-stbd)", range);
    }

    //--- the standoff trace ------------------------------------------------------------------
    //The measurement and its setpoint on one pair of axes, which is the quickest way to see
    //whether the range loop is holding or hunting, and the only way to watch the loop follow
    //the setpoint while W or S is held down.
    {
        standoffPlot_[0].push_back((GLfloat)(fix_.valid
                                             ? std::min(PLOT_RANGE_MAX, fix_.distance)
                                             : sf::Scalar(0)));
        standoffPlot_[1].push_back((GLfloat)standoffSp_);
        for(size_t k=0; k<standoffPlot_.size(); ++k)
        {
            if(standoffPlot_[k].size() > TRACE_LEN)
                standoffPlot_[k].erase(standoffPlot_[k].begin(),
                                       standoffPlot_[k].begin()
                                           + (standoffPlot_[k].size() - TRACE_LEN));
        }
        id.item = 1;
        getGUI()->DoTimePlot(id, COL_X, TRACE_PLOT_Y, COL_W, PLOT_H, standoffPlot_,
                             "STANDOFF AND SETPOINT 0-6 m", range);
    }
}
