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
//  BlueROV2TestApp.cpp
//  Stonefish
//

#include "BlueROV2TestApp.h"

#include <actuators/Thruster.h>
#include <core/Robot.h>
#include <core/SimulationManager.h>
#include <entities/SolidEntity.h>
#include <entities/forcefields/Ocean.h>
#include <graphics/IMGUI.h>
#include <sensors/scalar/DVL.h>
#include <sensors/scalar/IMU.h>
#include <sensors/scalar/Pressure.h>
#include <sensors/vision/ColorCamera.h>
#include <core/Console.h>
#include <utils/SystemUtil.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>

//! Fixed thrust allocation for the BlueROV2 Heavy, in the NED body frame
//! (X forward, Y starboard, Z down).
//!
//! The four vectored horizontal thrusters take surge, sway and yaw; the four outboard
//! vertical thrusters take heave, roll and pitch. Positive heave is DOWNWARD in NED.
//!
//! The signs come from the thruster origins in bluerov2_heavy.scn, mapped through PROPELLER
//! HANDEDNESS. This second step is easy to miss: FDThrust::Update() sets
//!     backward = (RH && input < 0) || (!RH && input > 0)
//! so for the SAME positive setpoint a right-handed propeller pushes forward and a left-handed
//! one pushes backward. Writing
//!     setpoint_i = handedness_i * geometric_coefficient_i,   handedness = +1 for right="true"
//! is therefore required. Both groups here are handed (cw, ccw, ccw, cw) = (+1,-1,-1,+1), so a
//! naive all-ones heave column commands thrusts (+,-,-,+) which cancel exactly and the vehicle
//! does not move at all - which is precisely what happened before this was corrected.
//!
//! Geometrically, a horizontal thruster contributes along (cos psi, sin psi, 0) and a vertical
//! one along (0, 0, +1). Each column below sums to zero force and moment in the axes it is not
//! meant to drive, so the demands do not cross-couple.
namespace
{
    struct Allocation
    {
        const char* name;
        sf::Scalar a, b, c;
    };

    //Horizontal group: coefficients are (surge, sway, yaw)
    const Allocation HORIZONTAL[4] = {
        {"ThrusterFrontRight",  sf::Scalar( 1), sf::Scalar(-1), sf::Scalar(-1)},
        {"ThrusterFrontLeft",   sf::Scalar(-1), sf::Scalar(-1), sf::Scalar(-1)},
        {"ThrusterBackRight",   sf::Scalar(-1), sf::Scalar(-1), sf::Scalar( 1)},
        {"ThrusterBackLeft",    sf::Scalar( 1), sf::Scalar(-1), sf::Scalar( 1)}
    };

    //Vertical group: coefficients are (heave, roll, pitch)
    const Allocation VERTICAL[4] = {
        {"ThrusterVertFrontRight", sf::Scalar( 1), sf::Scalar( 1), sf::Scalar(-1)},
        {"ThrusterVertFrontLeft",  sf::Scalar(-1), sf::Scalar( 1), sf::Scalar( 1)},
        {"ThrusterVertBackRight",  sf::Scalar(-1), sf::Scalar(-1), sf::Scalar(-1)},
        {"ThrusterVertBackLeft",   sf::Scalar( 1), sf::Scalar(-1), sf::Scalar( 1)}
    };

    //! Fraction of full thruster authority that a held key is allowed to command, per axis.
    //!
    //! Full authority is far too much to fly on. The measured steady rate at a yaw demand of
    //! 1.0 is 380 deg/s - more than a revolution per second - because the model's rotational
    //! damping is geometry-driven and cannot be matched to Wu's Nrr (see KNOWN LIMITATIONS in
    //! bluerov2_heavy.scn), while every thruster makes its full datasheet bollard thrust with
    //! no interaction losses. The caps below trade that unusable authority for rates a pilot
    //! can actually fly. The sliders still reach 1.0, so full authority is one drag away.
    const sf::Scalar GAIN_SURGE = sf::Scalar(0.80);
    const sf::Scalar GAIN_SWAY  = sf::Scalar(0.60);
    const sf::Scalar GAIN_HEAVE = sf::Scalar(0.50);
    const sf::Scalar GAIN_YAW   = sf::Scalar(0.12);
    const sf::Scalar GAIN_PITCH = sf::Scalar(0.30);
    const sf::Scalar GAIN_ROLL  = sf::Scalar(0.30);

    //! How fast a demand moves toward what the keys ask for, in demand units per second.
    //! 2.0 is half a second from rest to an axis maximum. Stepping straight to the maximum,
    //! which is what this used to do, is most of what made the controls feel twitchy.
    const sf::Scalar RAMP_RATE = sf::Scalar(2.0);

    //! Largest frame time the ramp will integrate, so a stall does not jump the demand
    //! straight to its maximum. The autopilot integrates against the same clamp.
    const sf::Scalar RAMP_MAX_DT = sf::Scalar(0.1);

    // ================================ AUTOPILOT ================================

    //! Depth loop. Both vertical modes work in the same quantity - metres the vehicle should
    //! move down - but they get separate controllers, because the two measurements are not
    //! interchangeable; see the altitude block below.
    //!
    //! The output limit is the manual heave cap, so no hold can command authority a pilot
    //! could not. The integrator is allowed most of that on purpose: it has to carry the
    //! +2.0 N of net buoyancy by itself (see VERTICAL_TRIM).
    const sf::Scalar VERT_KP = sf::Scalar(2.00);
    const sf::Scalar VERT_KI = sf::Scalar(0.03);
    const sf::Scalar VERT_KD = sf::Scalar(1.20);
    const sf::Scalar VERT_IBAND = sf::Scalar(1.0);   //m
    //The integrator has to be able to reach the trim of the WORST case, not of still water.
    //Measured: level and still needs 0.098, but 15 deg nose-up at full surge needs 0.40, and an
    //integrator capped below that leaves a standing depth error it can never remove - which is
    //exactly what a 0.20 limit did, parking the vehicle 0.63 m shallow.
    const sf::Scalar VERT_ILIMIT = sf::Scalar(0.45);
    const sf::Scalar VERT_LIMIT = GAIN_HEAVE;

    //! Altitude loop. Same structure, but a quarter of the proportional gain, because the
    //! measurement is much worse: depth comes from a 10 Hz pressure sensor with a millimetre of
    //! noise and a continuous reading, while the DVL's altitude is the nearest of four slant
    //! beams off a rough surface and steps as those beams cross features. ALT_KP is close to the
    //! gain the old ice standoff hold used, which flew the canopy well.
    //!
    //! Only P and I read the DVL. The DAMPING comes from the pressure sensor - the same depth
    //! rate the depth loop uses - which is why ALT_KD can stay as high as VERT_KD. See the note
    //! in RunHolds(); it is the change that made this mode usable at all.
    const sf::Scalar ALT_KP = sf::Scalar(0.50);
    const sf::Scalar ALT_KI = sf::Scalar(0.03);
    const sf::Scalar ALT_KD = sf::Scalar(1.20);
    const sf::Scalar ALT_IBAND = sf::Scalar(1.0);
    const sf::Scalar ALT_ILIMIT = sf::Scalar(0.45);
    const sf::Scalar ALT_LIMIT = sf::Scalar(0.30);

    //! Steady heave demand that offsets the +2.0 N of net buoyancy. The vertical integrator is
    //! seeded with it so a fresh engage settles in a second or two instead of sagging upward
    //! while the integral winds in.
    //!
    //! Thrust is quadratic in the normalised setpoint, not linear: four vertical units at
    //! 51.5 N bollard each need sqrt(2.0/(4*51.5)) = 0.098, where reading it as linear would
    //! have predicted 0.010 and left the vehicle rising.
    const sf::Scalar VERTICAL_TRIM = sf::Scalar(0.098);

    //! Heading loop.
    //!
    //! Its limit is deliberately ABOVE the manual yaw cap. The autopilot needs more authority to
    //! arrest a turn than a pilot needs to command one, because the model has almost no
    //! rotational damping to help it: held to 0.12 the loop spent 0.096 of that just sustaining
    //! a 30 deg/s setpoint slew, had 0.024 left to stop with, and rang +-4 deg for half a minute.
    const sf::Scalar HDG_KP = sf::Scalar(0.60);
    const sf::Scalar HDG_KI = sf::Scalar(0.01);
    const sf::Scalar HDG_KD = sf::Scalar(0.20);
    const sf::Scalar HDG_IBAND = sf::Scalar(15.0 * M_PI/180.0);
    const sf::Scalar HDG_ILIMIT = sf::Scalar(0.05);
    const sf::Scalar HDG_LIMIT = sf::Scalar(0.20);

    //! Attitude loop. The gains below are the PITCH gains; roll takes the same ones scaled by
    //! ROLL_GAIN_RATIO.
    //!
    //! The two axes are not interchangeable even though they run off the same four thrusters and
    //! see the same inertia (0.16 kgm2 on both) and the same restoring moment (B*BG = 2.30
    //! Nm/rad). The vertical thrusters sit at x = +-0.12 but y = +-0.218, so the same demand
    //! makes 1.8 times as much roll moment as pitch moment. Running one set of gains on both put
    //! the roll loop 1.8x higher in loop gain than the pitch loop, and it showed exactly as you
    //! would expect: under ice, pitch held to +-1 deg while roll sat in a 2 Hz limit cycle of
    //! +-4 deg. Scaling roll by the moment arm ratio equalises them.
    const sf::Scalar ATT_KP = sf::Scalar(1.20);
    const sf::Scalar ATT_KI = sf::Scalar(0.15);
    const sf::Scalar ATT_KD = sf::Scalar(0.50);
    const sf::Scalar ATT_IBAND = sf::Scalar(20.0 * M_PI/180.0);
    const sf::Scalar ATT_ILIMIT = sf::Scalar(0.30);
    const sf::Scalar ATT_LIMIT = GAIN_PITCH;
    //! 0.12/0.218 - the ratio of the vertical thrusters' pitch and roll moment arms.
    const sf::Scalar ROLL_GAIN_RATIO = sf::Scalar(0.55);

    //! Filter time constants for the measured rates that feed the D terms. The IMU rates carry
    //! 0.05 rad/s of noise at 50 Hz; the depth rate is reconstructed from a 10 Hz sensor and
    //! needs a longer one.
    const sf::Scalar RATE_TAU = sf::Scalar(0.15);
    const sf::Scalar DEPTH_RATE_TAU = sf::Scalar(0.50);

    //! How fast a held key moves a setpoint.
    const sf::Scalar SP_RATE_DEPTH = sf::Scalar(0.30);                  //m/s
    const sf::Scalar SP_RATE_HEADING = sf::Scalar(30.0 * M_PI/180.0);   //rad/s
    const sf::Scalar SP_RATE_ATTITUDE = sf::Scalar(15.0 * M_PI/180.0);  //rad/s

    //! Setpoint travel limits.
    const sf::Scalar SP_MIN_ALTITUDE = sf::Scalar(0.5);
    const sf::Scalar SP_MAX_ALTITUDE = sf::Scalar(15.0);
    const sf::Scalar SP_MAX_ATTITUDE = sf::Scalar(30.0 * M_PI/180.0);

    //! Where the pressure head sits in the body frame, from bluerov2_heavy.scn. The sensor
    //! measures the depth of the HEAD; the autopilot and the telemetry panel both talk about
    //! the depth of the vehicle origin, so this offset is rotated by the measured attitude and
    //! taken back off - it is 0.10 m of depth only while the vehicle is level.
    const sf::Vector3 PRESSURE_MOUNT(sf::Scalar(0), sf::Scalar(0), sf::Scalar(0.10));

    //! Past this much tilt, projecting the DVL's slant altitude onto the vertical stops meaning
    //! anything and the altitude is treated as missing. cos(78 deg).
    const sf::Scalar MIN_COS_TILT = sf::Scalar(0.2);

    const char* ROBOT_NAME = "BLUEROV2";
    const char* CAMERA_NAME = "camera";
    const char* PRESSURE_NAME = "pressure";
    const char* IMU_NAME = "imu";
    const char* DVL_NAME = "dvl";

    //Gap between the small camera inset and the window edge, in pixels
    const unsigned int MARGIN = 10;

    //Autopilot panel geometry, in the right hand column: clear of UnderIceTest's forward sonar
    //display above it and of the pilot camera inset below it. The left column is full.
    const GLfloat AP_PANEL_W = 250.f;
    const GLfloat AP_PANEL_H = 118.f;
    const GLfloat AP_PANEL_Y = 170.f;

    inline sf::Scalar Clamp(sf::Scalar v)
    {
        return std::max(sf::Scalar(-1), std::min(sf::Scalar(1), v));
    }

    //! Returns 1 when the key is held, 0 otherwise.
    inline sf::Scalar Held(const Uint8* keys, SDL_Scancode code)
    {
        return keys[code] ? sf::Scalar(1) : sf::Scalar(0);
    }

    //! Moves value toward target at RAMP_RATE, without overshooting it.
    inline void RampTowards(sf::Scalar& value, sf::Scalar target, sf::Scalar dt)
    {
        const sf::Scalar step = RAMP_RATE * dt;
        if(target > value)
            value = std::min(target, value + step);
        else
            value = std::max(target, value - step);
    }

    //! Scales a group of four thruster commands down as a whole if any one of them is over the
    //! limit, instead of clipping each on its own.
    //!
    //! Clipping per thruster corrupts every axis at once and does it silently. The vertical
    //! group carries heave, roll and pitch, whose manual caps alone already sum to 1.10, and
    //! with the vertical and attitude holds both engaged it saturates routinely. Uniform
    //! scaling keeps the direction of the commanded wrench and loses authority evenly, which is
    //! predictable; clipping does neither.
    inline void Desaturate(sf::Scalar c[4])
    {
        sf::Scalar peak(0);
        for(size_t i=0; i<4; ++i)
            peak = std::max(peak, std::fabs(c[i]));
        if(peak > sf::Scalar(1))
            for(size_t i=0; i<4; ++i)
                c[i] /= peak;
    }

    //! Looks a sensor up under both the prefixed and the bare name. The parser prefixes
    //! everything it builds with the robot name, but sensors attached from C++ after the parse
    //! - as UnderIceTest does - are not prefixed.
    sf::Sensor* FindSensor(sf::Robot* rov, const char* name)
    {
        sf::Sensor* s = rov->getSensor(std::string(ROBOT_NAME) + "/" + name);
        return s != nullptr ? s : rov->getSensor(name);
    }
}

BlueROV2TestApp::BlueROV2TestApp(std::string title, std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h,
                                 sf::SimulationManager* sim)
    : GraphicalSimulationApp(title, dataDirPath, s, h, sim),
      vehicleControl_(true),
      cameraView_(CameraView::SMALL),
      camera_(nullptr),
      cameraResolved_(false),
      surge_(0), sway_(0), heave_(0), roll_(0), pitch_(0), yaw_(0),
      verticalHold_(VerticalHold::DEPTH),
      headingHold_(true),
      attitudeHold_(true),
      depthSp_(0), altitudeSp_(2), headingSp_(0), pitchSp_(0), rollSp_(0),
      pendingSeed_(SEED_ALL),
      depthSpRate_(0), headingSpRate_(0), pitchSpRate_(0), rollSpRate_(0),
      lastRampTime_(0),
      sensorsResolved_(false),
      pressure_(nullptr),
      imu_(nullptr),
      dvl_(nullptr),
      depthRate_(DEPTH_RATE_TAU),
      rateP_(RATE_TAU), rateQ_(RATE_TAU), rateR_(RATE_TAU),
      depthPid_(VERT_KP, VERT_KI, VERT_KD, VERT_IBAND, VERT_ILIMIT, VERT_LIMIT),
      altitudePid_(ALT_KP, ALT_KI, ALT_KD, ALT_IBAND, ALT_ILIMIT, ALT_LIMIT),
      headingPid_(HDG_KP, HDG_KI, HDG_KD, HDG_IBAND, HDG_ILIMIT, HDG_LIMIT),
      pitchPid_(ATT_KP, ATT_KI, ATT_KD, ATT_IBAND, ATT_ILIMIT, ATT_LIMIT),
      rollPid_(ATT_KP * ROLL_GAIN_RATIO, ATT_KI * ROLL_GAIN_RATIO, ATT_KD * ROLL_GAIN_RATIO,
               ATT_IBAND, ATT_ILIMIT, ATT_LIMIT),
      altitudeLost_(false)
{
    //The robot does not exist yet - the scenario is built during Init(), after construction -
    //so the camera and the autopilot's sensors are resolved on the first frame instead, in
    //ProcessInputs(). The setpoints are seeded there too, off the first good fix.
    state_.valid = false;
    state_.altitudeValid = false;
    state_.altitudeSign = sf::Scalar(1);
    state_.depth = state_.depthRate = state_.altitude = sf::Scalar(0);
    state_.roll = state_.pitch = state_.yaw = sf::Scalar(0);
    state_.p = state_.q = state_.r = sf::Scalar(0);

    depthPid_.Reset(VERTICAL_TRIM);
    altitudePid_.Reset(VERTICAL_TRIM);
}

void BlueROV2TestApp::KeyDown(SDL_Event* event)
{
    //Tab hands the keyboard back and forth between the vehicle and the camera
    if(event->key.keysym.sym == SDLK_TAB)
    {
        vehicleControl_ = !vehicleControl_;
        cInfo(vehicleControl_ ? "Keyboard now pilots the vehicle." : "Keyboard now moves the camera.");
        return;
    }

    //V cycles the forward camera inset. Handled before the swallow list below, so it works
    //in both control modes; the base class does not bind it.
    if(event->key.keysym.sym == SDLK_v)
    {
        switch(cameraView_)
        {
            case CameraView::OFF:   cameraView_ = CameraView::SMALL; cInfo("Camera view: small."); break;
            case CameraView::SMALL: cameraView_ = CameraView::LARGE; cInfo("Camera view: large."); break;
            case CameraView::LARGE: cameraView_ = CameraView::OFF;   cInfo("Camera view: off."); break;
        }
        ApplyCameraView();
        return;
    }

    //Autopilot, also handled in both control modes. Z cycles rather than toggles because depth
    //and altitude are the same movement - the same four thrusters against the same vertical
    //error - and holding both at once is not a thing.
    //
    //Z is the base class's "move the camera down", which is why it is consumed here for good
    //rather than only while piloting.
    if(event->key.keysym.sym == SDLK_z)
    {
        switch(verticalHold_)
        {
            case VerticalHold::OFF:      SetVerticalHold(VerticalHold::DEPTH); break;
            case VerticalHold::DEPTH:    SetVerticalHold(VerticalHold::ALTITUDE); break;
            case VerticalHold::ALTITUDE: SetVerticalHold(VerticalHold::OFF); break;
        }
        return;
    }

    if(event->key.keysym.sym == SDLK_x)
    {
        SetHeadingHold(!headingHold_);
        return;
    }

    if(event->key.keysym.sym == SDLK_b)
    {
        SetAttitudeHold(!attitudeHold_);
        return;
    }

    //While piloting, swallow the keys the base class would otherwise use to move the camera
    if(vehicleControl_)
    {
        switch(event->key.keysym.sym)
        {
            case SDLK_w: case SDLK_s: case SDLK_a: case SDLK_d:
            case SDLK_q: case SDLK_e: case SDLK_SPACE:
                return;

            default:
                break;
        }
    }

    GraphicalSimulationApp::KeyDown(event);
}

void BlueROV2TestApp::SetVerticalHold(VerticalHold m)
{
    if(m == verticalHold_)
        return;

    verticalHold_ = m;
    altitudeLost_ = false;

    //Seed the setpoints from where the vehicle is and the integrator from the buoyancy trim,
    //so engaging neither kicks nor sags.
    if(state_.valid)
    {
        depthSp_ = state_.depth;
        pendingSeed_ &= ~SEED_DEPTH;
        if(state_.altitudeValid)
        {
            altitudeSp_ = btClamped(state_.altitude, SP_MIN_ALTITUDE, SP_MAX_ALTITUDE);
            pendingSeed_ &= ~SEED_ALTITUDE;
        }
    }
    depthPid_.Reset(VERTICAL_TRIM);
    altitudePid_.Reset(VERTICAL_TRIM);

    switch(verticalHold_)
    {
        case VerticalHold::OFF:
            cInfo("Vertical hold released.");
            break;
        case VerticalHold::DEPTH:
            cInfo("Depth hold engaged at %.2f m. [Q/E] move the setpoint.", (double)depthSp_);
            break;
        case VerticalHold::ALTITUDE:
            if(state_.altitudeValid)
                cInfo("Altitude hold engaged at %.2f m. [Q/E] move the setpoint.", (double)altitudeSp_);
            else
                cWarning("Altitude hold engaged, but the DVL has no return - holding depth until it does.");
            break;
    }
}

void BlueROV2TestApp::SetHeadingHold(bool on)
{
    headingHold_ = on;
    if(!on)
    {
        cInfo("Heading hold released.");
        return;
    }

    if(state_.valid)
    {
        headingSp_ = state_.yaw;
        pendingSeed_ &= ~SEED_HEADING;
    }
    headingPid_.Reset();
    cInfo("Heading hold engaged at %.0f deg. [A/D] move the setpoint.", (double)(headingSp_ * 180.0/M_PI));
}

void BlueROV2TestApp::SetAttitudeHold(bool on)
{
    attitudeHold_ = on;
    if(!on)
    {
        cInfo("Attitude hold released.");
        return;
    }

    if(state_.valid)
    {
        pitchSp_ = btClamped(state_.pitch, -SP_MAX_ATTITUDE, SP_MAX_ATTITUDE);
        rollSp_ = btClamped(state_.roll, -SP_MAX_ATTITUDE, SP_MAX_ATTITUDE);
        pendingSeed_ &= ~SEED_ATTITUDE;
    }
    pitchPid_.Reset();
    rollPid_.Reset();
    cInfo("Attitude hold engaged, pitch %.1f roll %.1f deg. [Up/Down] and [Left/Right] move the setpoints.",
          (double)(pitchSp_ * 180.0/M_PI), (double)(rollSp_ * 180.0/M_PI));
}

void BlueROV2TestApp::SyncSetpoints()
{
    if(!state_.valid)
        return;

    depthSp_ = state_.depth;
    headingSp_ = state_.yaw;
    pitchSp_ = btClamped(state_.pitch, -SP_MAX_ATTITUDE, SP_MAX_ATTITUDE);
    rollSp_ = btClamped(state_.roll, -SP_MAX_ATTITUDE, SP_MAX_ATTITUDE);
    pendingSeed_ &= ~(SEED_DEPTH | SEED_HEADING | SEED_ATTITUDE);

    if(state_.altitudeValid)
    {
        altitudeSp_ = btClamped(state_.altitude, SP_MIN_ALTITUDE, SP_MAX_ALTITUDE);
        pendingSeed_ &= ~SEED_ALTITUDE;
    }
}

void BlueROV2TestApp::ResolveSensors()
{
    if(sensorsResolved_)
        return;
    sensorsResolved_ = true;

    sf::Robot* rov = getSimulationManager()->getRobot(ROBOT_NAME);
    if(rov == nullptr)
    {
        cError("Robot '%s' not found - the autopilot is unavailable.", ROBOT_NAME);
        return;
    }

    pressure_ = dynamic_cast<sf::Pressure*>(FindSensor(rov, PRESSURE_NAME));
    imu_ = dynamic_cast<sf::IMU*>(FindSensor(rov, IMU_NAME));
    dvl_ = dynamic_cast<sf::DVL*>(FindSensor(rov, DVL_NAME));

    if(pressure_ == nullptr || imu_ == nullptr)
        cError("The pressure sensor or the IMU could not be resolved - the autopilot is off and "
               "the vehicle flies open loop.");
    else if(dvl_ == nullptr)
        cWarning("No DVL - depth, heading and attitude hold work; altitude hold does not.");
}

void BlueROV2TestApp::ReadState(sf::Scalar dt)
{
    state_.valid = false;
    state_.altitudeValid = false;

    if(pressure_ == nullptr || imu_ == nullptr)
        return;

    //An empty sensor history reads as zero on every channel, and a submerged vehicle always has
    //a positive gauge pressure, so this doubles as "the sensors have started sampling".
    const sf::Scalar gauge = pressure_->getLastValue(0);
    if(gauge <= sf::Scalar(0))
        return;

    sf::Ocean* ocn = getSimulationManager()->getOcean();
    if(ocn == nullptr)
        return;
    const sf::Scalar rho = ocn->getLiquid().density;
    const sf::Scalar g = getSimulationManager()->getGravity().getZ();
    if(rho <= sf::Scalar(0) || g <= sf::Scalar(0))
        return;

    state_.roll = imu_->getLastValue(0);
    state_.pitch = imu_->getLastValue(1);
    state_.yaw = imu_->getLastValue(2);
    state_.p = rateP_.Update(imu_->getLastValue(3), dt);
    state_.q = rateQ_.Update(imu_->getLastValue(4), dt);
    state_.r = rateR_.Update(imu_->getLastValue(5), dt);

    sf::Matrix3 R;
    R.setEulerYPR(state_.yaw, state_.pitch, state_.roll);

    //Ocean::GetPressure is depth*density*g, so this inverts it exactly.
    const sf::Scalar headDepth = gauge / (rho * g);
    state_.depth = headDepth - (R * PRESSURE_MOUNT).z();
    state_.depthRate = depthRate_.Update(state_.depth, dt);
    state_.valid = true;

    if(dvl_ == nullptr)
        return;

    //Channel 7 is the status - 0 bottom ping, 1 water only, 2 both, 3 nothing - and channel 3
    //the altitude, which reads altitude_max when no beam returns.
    const int status = (int)std::lround((double)dvl_->getLastValue(7));
    const sf::Scalar slant = dvl_->getLastValue(3);
    if((status != 0 && status != 2) || slant <= sf::Scalar(0))
        return;

    //The DVL reports minRange*cos(beamAngle) over its four beams, measured perpendicular to the
    //SENSOR. Project that onto the vertical so a pitched or rolled vehicle still holds the
    //standoff it asked for.
    const sf::Vector3 bodyDown = R.getColumn(2);
    const sf::Scalar cosTilt = bodyDown.z();
    if(cosTilt < MIN_COS_TILT)
        return;

    //Which way the beams point sets the sign of the whole loop: rising increases the altitude
    //to the seabed and decreases the altitude to an ice canopy. beam_positive_z is left at its
    //default of false in bluerov2_heavy.scn, so the beams fire along the sensor frame's -Z.
    //Comparing that against body +Z is attitude-invariant, unlike comparing it against world
    //down, so the sign cannot flip halfway through a roll.
    const sf::Vector3 beam = -dvl_->getSensorFrame().getBasis().getColumn(2);
    state_.altitudeSign = beam.dot(bodyDown) > sf::Scalar(0) ? sf::Scalar(-1) : sf::Scalar(1);

    state_.altitude = slant * cosTilt;
    state_.altitudeValid = true;
}

sf::Scalar BlueROV2TestApp::FrameDelta()
{
    //Wall clock rather than simulation time, so the controls and the loops feel the same
    //whatever the frame rate or the real-time factor
    const uint64_t now = sf::GetTimeInMicroseconds();
    const sf::Scalar dt = lastRampTime_ == 0 ? sf::Scalar(0)
                                             : sf::Scalar(now - lastRampTime_) / sf::Scalar(1000000);
    lastRampTime_ = now;
    return std::min(dt, RAMP_MAX_DT);
}

void BlueROV2TestApp::ReadKeyboard(sf::Scalar dt)
{
    if(!vehicleControl_)
        return;

    //Polling rather than key events, so that held and simultaneous keys work naturally
    const Uint8* keys = SDL_GetKeyboardState(nullptr);

    if(keys[SDL_SCANCODE_SPACE]) //All stop, immediately - no ramp
    {
        surge_ = sway_ = heave_ = roll_ = pitch_ = yaw_ = sf::Scalar(0);
        //Park the setpoints where the vehicle is, so an engaged hold stops it here instead of
        //flying it back to a target set before the pilot asked for everything to stop. The
        //integrators go back to the buoyancy trim for the same reason.
        SyncSetpoints();
        depthPid_.Reset(VERTICAL_TRIM);
        altitudePid_.Reset(VERTICAL_TRIM);
        headingPid_.Reset();
        pitchPid_.Reset();
        rollPid_.Reset();
        return;
    }

    //An axis owned by an engaged hold is ramped to zero rather than flown: its keys move a
    //setpoint in UpdateSetpoints(), and RunHolds() writes the demand afterwards. Ramping it to
    //zero rather than freezing it means releasing the hold hands the pilot a settled axis.
    const bool holdVert = verticalHold_ != VerticalHold::OFF && state_.valid;
    const bool holdHdg = headingHold_ && state_.valid;
    const bool holdAtt = attitudeHold_ && state_.valid;

    //The keys select a target and the demand ramps toward it. Releasing a key targets zero,
    //so the vehicle eases off over the same half second rather than cutting dead.
    RampTowards(surge_, (Held(keys, SDL_SCANCODE_W)      - Held(keys, SDL_SCANCODE_S))     * GAIN_SURGE, dt);
    RampTowards(sway_,  (Held(keys, SDL_SCANCODE_PERIOD) - Held(keys, SDL_SCANCODE_COMMA)) * GAIN_SWAY,  dt);
    RampTowards(yaw_,   holdHdg  ? sf::Scalar(0)
                                 : (Held(keys, SDL_SCANCODE_D) - Held(keys, SDL_SCANCODE_A)) * GAIN_YAW, dt);
    RampTowards(heave_, holdVert ? sf::Scalar(0)
                                 : (Held(keys, SDL_SCANCODE_E) - Held(keys, SDL_SCANCODE_Q)) * GAIN_HEAVE, dt);
    RampTowards(pitch_, holdAtt  ? sf::Scalar(0)
                                 : (Held(keys, SDL_SCANCODE_UP) - Held(keys, SDL_SCANCODE_DOWN)) * GAIN_PITCH, dt);
    RampTowards(roll_,  holdAtt  ? sf::Scalar(0)
                                 : (Held(keys, SDL_SCANCODE_RIGHT) - Held(keys, SDL_SCANCODE_LEFT)) * GAIN_ROLL, dt);
}

void BlueROV2TestApp::UpdateSetpoints(sf::Scalar dt)
{
    if(!state_.valid)
        return;

    //Anything still unseeded takes the first good fix. This is also how a subclass gets the
    //defaults it wants: it sets the setpoint in its constructor and clears the bit, and the
    //seed then leaves that one alone.
    if(pendingSeed_ != 0)
    {
        if(pendingSeed_ & SEED_DEPTH)    depthSp_ = state_.depth;
        if(pendingSeed_ & SEED_HEADING)  headingSp_ = state_.yaw;
        if(pendingSeed_ & SEED_ATTITUDE) { pitchSp_ = sf::Scalar(0); rollSp_ = sf::Scalar(0); }
        if((pendingSeed_ & SEED_ALTITUDE) && state_.altitudeValid)
        {
            altitudeSp_ = btClamped(state_.altitude, SP_MIN_ALTITUDE, SP_MAX_ALTITUDE);
            pendingSeed_ &= ~SEED_ALTITUDE;
        }
        pendingSeed_ &= ~(SEED_DEPTH | SEED_HEADING | SEED_ATTITUDE);
    }

    depthSpRate_ = headingSpRate_ = pitchSpRate_ = rollSpRate_ = sf::Scalar(0);

    if(!vehicleControl_)
        return;

    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    if(keys[SDL_SCANCODE_SPACE])
        return; //ReadKeyboard() has already parked everything

    //The same keys, in the same directions. E is down, and down both deepens the depth setpoint
    //and opens the standoff below an ice canopy, so no key changes meaning between the two
    //vertical modes - only the sign of the sensor does, below.
    const sf::Scalar heaveKey = Held(keys, SDL_SCANCODE_E) - Held(keys, SDL_SCANCODE_Q);
    const sf::Scalar yawKey = Held(keys, SDL_SCANCODE_D) - Held(keys, SDL_SCANCODE_A);
    const sf::Scalar pitchKey = Held(keys, SDL_SCANCODE_UP) - Held(keys, SDL_SCANCODE_DOWN);
    const sf::Scalar rollKey = Held(keys, SDL_SCANCODE_RIGHT) - Held(keys, SDL_SCANCODE_LEFT);

    switch(verticalHold_)
    {
        case VerticalHold::OFF:
            break;

        case VerticalHold::DEPTH:
            depthSp_ = std::max(sf::Scalar(0), depthSp_ + heaveKey * SP_RATE_DEPTH * dt);
            depthSpRate_ = heaveKey * SP_RATE_DEPTH;
            break;

        case VerticalHold::ALTITUDE:
            //Going down opens the gap to an ice canopy and closes it to the seabed.
            altitudeSp_ = btClamped(altitudeSp_ + state_.altitudeSign * heaveKey * SP_RATE_DEPTH * dt,
                                    SP_MIN_ALTITUDE, SP_MAX_ALTITUDE);
            //Keep the depth fallback fresh while the bottom track is good, so losing it parks
            //the vehicle where it is rather than where it last held depth.
            if(state_.altitudeValid)
                depthSp_ = state_.depth;
            //The vertical loop works in depth error, and altitudeSign cancels itself between the
            //setpoint slew and the error, so the depth-equivalent rate is just the key rate.
            depthSpRate_ = heaveKey * SP_RATE_DEPTH;
            break;
    }

    if(headingHold_)
    {
        headingSp_ = brov2::WrapPi(headingSp_ + yawKey * SP_RATE_HEADING * dt);
        headingSpRate_ = yawKey * SP_RATE_HEADING;
    }

    if(attitudeHold_)
    {
        pitchSp_ = btClamped(pitchSp_ + pitchKey * SP_RATE_ATTITUDE * dt, -SP_MAX_ATTITUDE, SP_MAX_ATTITUDE);
        rollSp_ = btClamped(rollSp_ + rollKey * SP_RATE_ATTITUDE * dt, -SP_MAX_ATTITUDE, SP_MAX_ATTITUDE);
        //Only while the setpoint is still inside its travel - past the clamp it is not moving.
        pitchSpRate_ = btFabs(pitchSp_) < SP_MAX_ATTITUDE ? pitchKey * SP_RATE_ATTITUDE : sf::Scalar(0);
        rollSpRate_ = btFabs(rollSp_) < SP_MAX_ATTITUDE ? rollKey * SP_RATE_ATTITUDE : sf::Scalar(0);
    }
}

void BlueROV2TestApp::RunHolds(sf::Scalar dt)
{
    //No sensor fix, no autopilot: the vehicle flies open loop exactly as it did before.
    if(!state_.valid)
        return;

    if(verticalHold_ != VerticalHold::OFF)
    {
        //Both modes come down to "metres the vehicle should move down", so one controller and
        //one integrator serve both. For altitude that is altitudeSign * (setpoint - measured):
        //closing on an ice canopy overhead means going up, closing on the seabed means going
        //down, and the sign of the DVL carries the difference.
        if(verticalHold_ == VerticalHold::ALTITUDE && state_.altitudeValid)
        {
            if(altitudeLost_)
            {
                altitudeLost_ = false;
                cInfo("Altitude reacquired - holding %.2f m.", (double)altitudeSp_);
            }
            //altitudeSign turns "close the gap" into "move down": closing on an ice canopy
            //overhead means going up, closing on the seabed means going down.
            //
            //The D term is the rate from the PRESSURE sensor, not from the DVL, and this is the
            //single change that made altitude hold usable. Differentiating the DVL costs so much
            //phase - it updates at 10 Hz, steps as its four slant beams cross features, and needs
            //about a second of filtering to be usable at all - that at the 4 s period the loop
            //wanted to oscillate at, the "damping" term arrived 57 degrees late and drove the
            //oscillation instead of opposing it: a steady +-0.6 m porpoise around a 2 m standoff
            //that never settled.
            //
            //It is also the physically right signal. Damping should oppose the VEHICLE's vertical
            //motion, which is what the pressure sensor measures directly and cleanly. The DVL
            //measures the gap, which changes both when the vehicle moves and when the ice above
            //it slopes - and a D term should not fight the terrain profile.
            heave_ = altitudePid_.Update(state_.altitudeSign * (altitudeSp_ - state_.altitude),
                                         state_.depthRate - depthSpRate_, dt);
        }
        else
        {
            //ALTITUDE with no bottom track degrades to depth hold at the depth where the track
            //was lost, rather than driving on blind. Seeding the depth integrator from the
            //altitude one hands the axis over without a step in the demand.
            if(verticalHold_ == VerticalHold::ALTITUDE && !altitudeLost_)
            {
                altitudeLost_ = true;
                depthPid_.Reset(altitudePid_.getIntegrator());
                cWarning("Altitude lost - holding depth at %.2f m until the DVL sees something.",
                         (double)depthSp_);
            }
            //The integrator carries the +2.0 N of net buoyancy, so it is never near zero in trim.
            heave_ = depthPid_.Update(depthSp_ - state_.depth, state_.depthRate - depthSpRate_, dt);
        }
    }

    if(headingHold_)
    {
        //The D term is the body yaw rate rather than the derivative of the heading. The two
        //differ once the vehicle is pitched, but the attitude hold keeps it within a few
        //degrees of level, and a rate the IMU measures beats one differentiated from an angle.
        yaw_ = headingPid_.Update(brov2::WrapPi(headingSp_ - state_.yaw), state_.r - headingSpRate_, dt);
    }

    if(attitudeHold_)
    {
        pitch_ = pitchPid_.Update(pitchSp_ - state_.pitch, state_.q - pitchSpRate_, dt);
        roll_ = rollPid_.Update(rollSp_ - state_.roll, state_.p - rollSpRate_, dt);
    }
}

void BlueROV2TestApp::AllocateThrust()
{
    sf::Robot* rov = getSimulationManager()->getRobot(ROBOT_NAME);
    if(rov == nullptr)
        return;

    sf::Scalar h[4], v[4];
    for(size_t i=0; i<4; ++i)
    {
        h[i] = HORIZONTAL[i].a * surge_ + HORIZONTAL[i].b * sway_ + HORIZONTAL[i].c * yaw_;
        v[i] = VERTICAL[i].a * heave_ + VERTICAL[i].b * roll_ + VERTICAL[i].c * pitch_;
    }
    Desaturate(h);
    Desaturate(v);

    //The parser prefixes actuator names with the robot name
    for(size_t i=0; i<4; ++i)
    {
        sf::Thruster* th = dynamic_cast<sf::Thruster*>(rov->getActuator(std::string(ROBOT_NAME) + "/" + HORIZONTAL[i].name));
        if(th == nullptr)
            th = dynamic_cast<sf::Thruster*>(rov->getActuator(HORIZONTAL[i].name));
        if(th != nullptr)
            th->setSetpoint(Clamp(h[i]));
    }

    for(size_t i=0; i<4; ++i)
    {
        sf::Thruster* th = dynamic_cast<sf::Thruster*>(rov->getActuator(std::string(ROBOT_NAME) + "/" + VERTICAL[i].name));
        if(th == nullptr)
            th = dynamic_cast<sf::Thruster*>(rov->getActuator(VERTICAL[i].name));
        if(th != nullptr)
            th->setSetpoint(Clamp(v[i]));
    }
}

void BlueROV2TestApp::ApplyCameraView()
{
    if(!cameraResolved_)
    {
        cameraResolved_ = true;

        sf::Robot* rov = getSimulationManager()->getRobot(ROBOT_NAME);
        if(rov != nullptr)
        {
            //A robot's sensors live on the robot rather than in the manager's list
            camera_ = dynamic_cast<sf::ColorCamera*>(FindSensor(rov, CAMERA_NAME));
        }

        if(camera_ == nullptr)
            cError("Camera '%s' not found - the camera view is unavailable.", CAMERA_NAME);
    }

    if(camera_ == nullptr)
        return;

    if(cameraView_ == CameraView::OFF)
    {
        camera_->setDisplayOnScreen(false, 0, 0, 1.f);
        return;
    }

    //Placement is from the TOP left, the same origin as the IMGUI panels: setDisplayOnScreen
    //feeds OpenGLContent::DrawTexturedQuad, which flips into GL coordinates itself.
    unsigned int resX, resY;
    camera_->getResolution(resX, resY);
    const unsigned int w = getWindowWidth();
    const unsigned int h = getWindowHeight();

    float scale;
    if(cameraView_ == CameraView::LARGE)
        scale = (float)w / (float)resX; //Full window width, HUD panels stay visible underneath
    else
        scale = 0.35f;

    const unsigned int dispW = (unsigned int)(resX * scale);
    const unsigned int dispH = (unsigned int)(resY * scale);

    if(cameraView_ == CameraView::LARGE) //Full width, along the bottom edge
        camera_->setDisplayOnScreen(true, 0, h > dispH ? h - dispH : 0, scale);
    else //Bottom right corner, clear of the sliders and the telemetry panel
        camera_->setDisplayOnScreen(true,
                                    w > dispW + MARGIN ? w - dispW - MARGIN : 0,
                                    h > dispH + MARGIN ? h - dispH - MARGIN : 0,
                                    scale);
}

void BlueROV2TestApp::ProcessInputs()
{
    if(!cameraResolved_)
        ApplyCameraView();
    ResolveSensors();

    const sf::Scalar dt = FrameDelta();

    ReadState(dt);
    ReadKeyboard(dt);
    UpdateSetpoints(dt);
    RunHolds(dt);
    AugmentDemands(dt);
    AllocateThrust();
}


void BlueROV2TestApp::AugmentDemands(sf::Scalar dt)
{
    (void)dt;
}

void BlueROV2TestApp::DoTelemetry()
{
    sf::Robot* rov = getSimulationManager()->getRobot(ROBOT_NAME);
    if(rov == nullptr)
        return;
    sf::SolidEntity* base = rov->getBaseLink();
    if(base == nullptr)
        return;

    sf::Vector3 v = base->getLinearVelocity();
    sf::Transform T = base->getOTransform(); //origin frame: the vehicle's actual pose. getCGTransform() is the principal-axis frame and carries an arbitrary rotation.
    sf::Scalar r, p, y;
    T.getBasis().getEulerYPR(y, p, r);

    const GLfloat x = 10.f;
    const GLfloat yy = 350.f;
    getGUI()->DoPanel(x, yy, 250.f, 150.f);

    char buf[128];
    snprintf(buf, sizeof(buf), "CONTROL: %s  [Tab]", vehicleControl_ ? "VEHICLE" : "CAMERA");
    getGUI()->DoLabel(x + 10.f, yy + 12.f, std::string(buf));
    snprintf(buf, sizeof(buf), "Speed  %5.2f m/s", (double)v.length());
    getGUI()->DoLabel(x + 10.f, yy + 34.f, std::string(buf));
    snprintf(buf, sizeof(buf), "u %5.2f  v %5.2f  w %5.2f", (double)v.x(), (double)v.y(), (double)v.z());
    getGUI()->DoLabel(x + 10.f, yy + 54.f, std::string(buf));
    snprintf(buf, sizeof(buf), "Depth  %5.2f m", (double)T.getOrigin().z());
    getGUI()->DoLabel(x + 10.f, yy + 74.f, std::string(buf));
    snprintf(buf, sizeof(buf), "R %6.1f  P %6.1f  Y %6.1f deg",
             (double)(r * 180.0 / M_PI), (double)(p * 180.0 / M_PI), (double)(y * 180.0 / M_PI));
    getGUI()->DoLabel(x + 10.f, yy + 94.f, std::string(buf));
    const char* view = cameraView_ == CameraView::OFF ? "OFF"
                     : (cameraView_ == CameraView::SMALL ? "SMALL" : "LARGE");
    snprintf(buf, sizeof(buf), "CAMERA: %s  [V]", view);
    getGUI()->DoLabel(x + 10.f, yy + 114.f, std::string(buf));
}

void BlueROV2TestApp::DoAutopilotPanel()
{
    //Right hand column: the left one is full. Telemetry runs 350-500 here and UnderIceTest
    //continues down to 884 in a 900 px window. This clears UnderIceTest's sonar display above
    //and the pilot camera inset below.
    const GLfloat x = getWindowWidth() > (unsigned int)(AP_PANEL_W + MARGIN)
                    ? (GLfloat)getWindowWidth() - AP_PANEL_W - (GLfloat)MARGIN : 0.f;
    getGUI()->DoPanel(x, AP_PANEL_Y, AP_PANEL_W, AP_PANEL_H);

    const GLfloat tx = x + 10.f;
    GLfloat ty = AP_PANEL_Y + 12.f;
    char buf[128];

    getGUI()->DoLabel(tx, ty, state_.valid ? "AUTOPILOT" : "AUTOPILOT - NO SENSOR DATA");
    ty += 22.f;

    switch(verticalHold_)
    {
        case VerticalHold::OFF:
            snprintf(buf, sizeof(buf), "VERT  OFF   [Z]");
            break;
        case VerticalHold::DEPTH:
            snprintf(buf, sizeof(buf), "VERT  DEPTH [Z] %5.2f / %5.2f m",
                     (double)depthSp_, (double)state_.depth);
            break;
        case VerticalHold::ALTITUDE:
            if(altitudeLost_ || !state_.altitudeValid)
                snprintf(buf, sizeof(buf), "VERT  ALT!  [Z] no return, dep %4.1f", (double)depthSp_);
            else
                snprintf(buf, sizeof(buf), "VERT  ALT   [Z] %5.2f / %5.2f m",
                         (double)altitudeSp_, (double)state_.altitude);
            break;
    }
    getGUI()->DoLabel(tx, ty, std::string(buf));
    ty += 20.f;

    if(headingHold_)
        snprintf(buf, sizeof(buf), "HDG   ON    [X] %5.0f / %5.0f deg",
                 (double)(brov2::WrapPi(headingSp_) * 180.0/M_PI), (double)(state_.yaw * 180.0/M_PI));
    else
        snprintf(buf, sizeof(buf), "HDG   OFF   [X]");
    getGUI()->DoLabel(tx, ty, std::string(buf));
    ty += 20.f;

    if(attitudeHold_)
        snprintf(buf, sizeof(buf), "ATT   ON    [B] P%4.0f/%4.0f R%4.0f/%4.0f",
                 (double)(pitchSp_ * 180.0/M_PI), (double)(state_.pitch * 180.0/M_PI),
                 (double)(rollSp_ * 180.0/M_PI), (double)(state_.roll * 180.0/M_PI));
    else
        snprintf(buf, sizeof(buf), "ATT   OFF   [B]");
    getGUI()->DoLabel(tx, ty, std::string(buf));
    ty += 20.f;

    getGUI()->DoLabel(tx, ty, "Motion keys move the setpoints");
}

void BlueROV2TestApp::DoHUD()
{
    GraphicalSimulationApp::DoHUD();

    //Piloting itself runs in ProcessInputs(), so that hiding the HUD does not stop the vehicle.
    //The sliders below are still a second way in: the keyboard overwrites them every frame
    //while in vehicle mode, and so does an engaged hold.
    sf::Uid id;
    id.owner = 10;

    id.item = 0;
    surge_ = getGUI()->DoSlider(id, 180.f, 10.f, 250.f, sf::Scalar(-1), sf::Scalar(1), surge_, "Surge  [W/S]");
    id.item = 1;
    sway_  = getGUI()->DoSlider(id, 180.f, 65.f, 250.f, sf::Scalar(-1), sf::Scalar(1), sway_, "Sway  [, .]");
    id.item = 2;
    heave_ = getGUI()->DoSlider(id, 180.f, 120.f, 250.f, sf::Scalar(-1), sf::Scalar(1), heave_, "Heave +down  [E/Q]");
    id.item = 3;
    roll_  = getGUI()->DoSlider(id, 180.f, 175.f, 250.f, sf::Scalar(-1), sf::Scalar(1), roll_, "Roll  [Left/Right]");
    id.item = 4;
    pitch_ = getGUI()->DoSlider(id, 180.f, 230.f, 250.f, sf::Scalar(-1), sf::Scalar(1), pitch_, "Pitch  [Up/Down]");
    id.item = 5;
    yaw_   = getGUI()->DoSlider(id, 180.f, 285.f, 250.f, sf::Scalar(-1), sf::Scalar(1), yaw_, "Yaw  [A/D]");

    DoTelemetry();
    DoAutopilotPanel();
}
