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
//  UnderIceTestApp.cpp
//  Stonefish
//

#include "UnderIceTestApp.h"
#include "UnderIceTestManager.h"

#include <core/Console.h>
#include <core/Robot.h>
#include <core/SimulationManager.h>
#include <graphics/IMGUI.h>
#include <sensors/Sample.h>
#include <sensors/scalar/Multibeam.h>
#include <sensors/scalar/Profiler.h>
#include <sensors/vision/ColorCamera.h>
#include <sensors/vision/FLS.h>
#include <graphics/OpenGLView.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace
{
    //! A range this close to the sounder's maximum is a no-return: nothing overhead.
    const sf::Scalar NO_RETURN_MARGIN = sf::Scalar(0.5);

    //! Samples kept in the along-track draft trace. At the sounder's 10 Hz and the vehicle's
    //! ~1.4 m/s cruise this is a little over a minute, or about 80 m of track.
    const size_t DRAFT_TRACE_LEN = 600;

    //! Vertical extent of both ice plots, in metres of draft. The deepest keels in the site
    //! reach 14 m and clip against this, which is the right trade: a scale that fits them
    //! would push level ice into the top few pixels of the box and make the ordinary case
    //! unreadable. Fixing the scale at all beats autoscaling, which would rescale under the
    //! vehicle every time it crossed a ridge. Drafts are plotted NEGATED against this range,
    //! so deeper ice draws lower - the way it hangs.
    const sf::Scalar DRAFT_PLOT_MAX = sf::Scalar(8.0);

    //! Standoff this app starts holding, in metres below the canopy. Far enough out of the
    //! DVL's 0.3 m blanking range to keep it reporting, close enough that the camera and the
    //! lights still reach the ice. The loop itself is the inherited ALTITUDE mode, flown off
    //! the upward DVL; Z cycles it and Q/E move it.
    const sf::Scalar START_STANDOFF = sf::Scalar(2.0);

    //! Gap between a screen display and the window edge, in pixels.
    const unsigned int MARGIN = 10;
    const float FLS_SCALE = 0.35f;       //  566 x 400 -> 198 x 140
    //! Ice camera inset: 640 x 360 at 0.35 is 224 x 126, which drops into the right hand
    //! column between the autopilot panel (ends at y = 288) and the pilot view along the
    //! bottom. The bow camera is 1280 x 720, so it needs half that scale for the same box.
    const float ICE_CAM_SCALE = 0.175f;  // 1280 x 720 -> 224 x 126
    const GLfloat ICE_CAM_Y = 300.f;

    //! Detents for the bow camera's tilt, in degrees above the horizontal. The real BlueROV2
    //! carries its camera on a tilt servo with about this range, and under ice it lives at the
    //! top of it.
    const double CAM_TILT_DEG[] = { 0.0, 45.0, 90.0 };
    const unsigned int CAM_TILT_COUNT = 3;
    //! Where the bow camera sits, from bluerov2_heavy.scn: just clear of the visual hull,
    //! which reaches x = 0.222.
    const double CAM_X = 0.24;

    //! HUD panel geometry, continuing down the left column below the base telemetry panel
    //! (which occupies y = 350 .. 500).
    const GLfloat COL_X = 10.f;
    const GLfloat COL_W = 250.f;
    const GLfloat ICE_PANEL_Y = 510.f;
    const GLfloat ICE_PANEL_H = 132.f;
    const GLfloat SWATH_PLOT_Y = 652.f;
    const GLfloat TRACE_PLOT_Y = 772.f;
    const GLfloat PLOT_H = 112.f;
}

UnderIceTestApp::UnderIceTestApp(std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h,
                                 sf::SimulationManager* sim)
    : BlueROV2TestApp("UnderIceTest", dataDirPath, s, h, sim),
      payloadResolved_(false),
      altimeter_(nullptr),
      swath_(nullptr),
      fls_(nullptr),
      iceCamera_(nullptr),
      fwdCamera_(nullptr),
      showFLS_(true),
      showIceCamera_(true),
      camTilt_(2)
{
    //Start on the standoff rather than at whatever clearance the spawn happens to give. Without
    //a vertical hold the scenario does not work unattended: the vehicle is +2.0 N buoyant, so it
    //rises until it is touching the canopy, at which point the clearance is inside every upward
    //sensor's blanking range and the ice readouts go blank. Flying a standoff is also what an
    //under-ice survey does.
    //
    //Clearing SEED_ALTITUDE keeps the base class from overwriting this with the first fix.
    verticalHold_ = VerticalHold::ALTITUDE;
    altitudeSp_ = START_STANDOFF;
    pendingSeed_ &= ~SEED_ALTITUDE;

    swathPlot_.resize(1);
    draftPlot_.resize(1);
}

void UnderIceTestApp::ResolvePayload()
{
    if(payloadResolved_)
        return;
    payloadResolved_ = true;

    sf::Robot* rov = getSimulationManager()->getRobot(underice::ROBOT);
    if(rov == nullptr)
    {
        cError("Robot '%s' not found - the ice instrumentation is unavailable.", underice::ROBOT);
        return;
    }

    //These five are attached by UnderIceTestManager::AddIcePayload and are given unprefixed
    //names there, unlike everything the parser builds.
    altimeter_ = dynamic_cast<sf::Profiler*>(rov->getSensor(underice::ALTIMETER));
    swath_ = dynamic_cast<sf::Multibeam*>(rov->getSensor(underice::MULTIBEAM));
    fls_ = dynamic_cast<sf::FLS*>(rov->getSensor(underice::FLS));

    fwdCamera_ = dynamic_cast<sf::ColorCamera*>(rov->getSensor(underice::FWD_CAMERA));

    if(altimeter_ == nullptr || swath_ == nullptr || fls_ == nullptr || fwdCamera_ == nullptr)
        cError("Part of the under-ice payload could not be resolved - some readouts will be blank.");

    //Swap which camera the inherited pilot view shows.
    //
    //The base class resolves camera_ to the scenario's bow camera and lets V cycle its inset.
    //Under ice that camera is aimed straight up at the canopy, so what V ought to be cycling is
    //the forward picture instead. Resolve the base class's camera first, take it as the ice
    //camera, then repoint camera_ at the forward one - all before the base class has had a
    //chance to display anything, because ProcessInputs() calls this before its own.
    if(!cameraResolved_)
        ApplyCameraView();
    iceCamera_ = camera_;
    if(fwdCamera_ != nullptr)
    {
        if(iceCamera_ != nullptr)
            iceCamera_->setDisplayOnScreen(false, 0, 0, 1.f); //undo what ApplyCameraView just did
        camera_ = fwdCamera_;
        ApplyCameraView();
    }

    ApplyFLSView();
    ApplyIceCameraView();
    ApplyCameraTilt();
}

void UnderIceTestApp::SetViewActive(sf::VisionSensor* sensor, bool active)
{
    if(sensor == nullptr)
        return;
    sf::OpenGLView* view = sensor->getOpenGLView();
    if(view != nullptr)
        view->setEnabled(active);
}

void UnderIceTestApp::ApplyFLSView()
{
    if(fls_ == nullptr)
        return;

    SetViewActive(fls_, showFLS_);

    //An FLS display is not the beam x bin grid: getDisplayResolution widens it to the fan's
    //chord, 2*sin(fovH/2)*bins across. Placement is from the TOP left, the same origin as the
    //IMGUI panels; the base class puts the pilot camera along the bottom right, so this goes
    //in the top right corner.
    unsigned int rx, ry;
    fls_->getDisplayResolution(rx, ry);
    const unsigned int dispW = (unsigned int)(rx * FLS_SCALE);
    const unsigned int w = getWindowWidth();
    fls_->setDisplayOnScreen(showFLS_, w > dispW + MARGIN ? w - dispW - MARGIN : 0, MARGIN, FLS_SCALE);
}

void UnderIceTestApp::ApplyCameraTilt()
{
    if(iceCamera_ == nullptr)
        return;

    //bluerov2_heavy.scn derives rpy = (pi/2 - a, 0, pi/2) for a vision sensor aimed forward
    //and tilted DOWN by a, so tilting UP is the same expression with a negated. The mount
    //position is unchanged; only the aim moves, which is what a tilt servo does.
    const double a = CAM_TILT_DEG[camTilt_ % CAM_TILT_COUNT] * M_PI / 180.0;
    iceCamera_->setRelativeSensorFrame(sf::Transform(sf::Quaternion(M_PI_2, 0.0, M_PI_2 + a),
                                                     sf::Vector3(CAM_X, 0.0, 0.0)));
}

//! Ice camera inset, in the right hand column between the autopilot panel and the pilot view.
void UnderIceTestApp::ApplyIceCameraView()
{
    if(iceCamera_ == nullptr)
        return;

    SetViewActive(iceCamera_, showIceCamera_);
    if(!showIceCamera_)
    {
        iceCamera_->setDisplayOnScreen(false, 0, 0, 1.f);
        return;
    }

    unsigned int rx, ry;
    iceCamera_->getResolution(rx, ry);
    const unsigned int dispW = (unsigned int)(rx * ICE_CAM_SCALE);
    const unsigned int dispH = (unsigned int)(ry * ICE_CAM_SCALE);
    const unsigned int w = getWindowWidth();
    iceCamera_->setDisplayOnScreen(true, w > dispW + MARGIN ? w - dispW - MARGIN : 0,
                                   ICE_CAM_Y, ICE_CAM_SCALE);
}

void UnderIceTestApp::ProcessInputs()
{
    ResolvePayload();
    BlueROV2TestApp::ProcessInputs();
}

void UnderIceTestApp::KeyDown(SDL_Event* event)
{
    //C tilts the ice camera - the scenario's own bow camera, on what the real vehicle carries as
    //a tilt servo. The forward view is a separate fixed camera; see ResolvePayload().
    if(event->key.keysym.sym == SDLK_c)
    {
        camTilt_ = (camTilt_ + 1) % CAM_TILT_COUNT;
        ApplyCameraTilt();
        cInfo("Camera tilt: %.0f deg up.", CAM_TILT_DEG[camTilt_]);
        return;
    }

    if(event->key.keysym.sym == SDLK_u)
    {
        showIceCamera_ = !showIceCamera_;
        cInfo(showIceCamera_ ? "Ice camera on." : "Ice camera off.");
        ApplyIceCameraView();
        return;
    }

    if(event->key.keysym.sym == SDLK_f)
    {
        showFLS_ = !showFLS_;
        cInfo(showFLS_ ? "Forward sonar on." : "Forward sonar off.");
        ApplyFLSView();
        return;
    }

    BlueROV2TestApp::KeyDown(event);
}

sf::Scalar UnderIceTestApp::BeamDraft(const sf::Transform& frame, sf::Scalar range, double angle) const
{
    //Zero is the placeholder ScalarSensor::getLastSample() returns for an empty history, and
    //anything at the maximum is a genuine no-return. Sample carries an "invalid" flag but
    //exposes no way to read it, so the range itself has to carry both meanings.
    if(range <= sf::Scalar(0) || range >= underice::UP_RANGE_MAX - NO_RETURN_MARGIN)
        return sf::Scalar(-1);

    //Multibeam.cpp:63 and Profiler.cpp:56 both build a beam as col(0)*cos(a) + col(1)*sin(a).
    //Projecting the slant range onto world Z rather than assuming the beam is vertical keeps
    //the draft honest while the vehicle is pitched or rolled - which, pinned against a keel,
    //it usually is.
    const sf::Vector3 fwd = frame.getBasis().getColumn(0);
    const sf::Vector3 side = frame.getBasis().getColumn(1);
    const sf::Scalar dz = fwd.z() * std::cos(angle) + side.z() * std::sin(angle);
    return frame.getOrigin().z() + range * dz;
}

bool UnderIceTestApp::AnySwathReturn() const
{
    if(swath_ == nullptr)
        return false;

    const sf::Sample s = swath_->getLastSample();
    const unsigned short n = swath_->getNumOfChannels();
    for(unsigned short i=0; i<n; ++i)
    {
        const sf::Scalar r = s.getValue(i);
        if(r > sf::Scalar(0) && r < underice::UP_RANGE_MAX - NO_RETURN_MARGIN)
            return true;
    }
    return false;
}

UnderIceTestApp::Overhead UnderIceTestApp::ReadOverhead(sf::Scalar& clearance, sf::Scalar& draft) const
{
    if(altimeter_ == nullptr)
        return Overhead::NO_DATA;

    //Channel 0 is the beam angle, channel 1 the range (Profiler.cpp:41-43).
    const sf::Sample s = altimeter_->getLastSample();
    const sf::Scalar angle = s.getValue(0);
    clearance = s.getValue(1);
    if(clearance <= sf::Scalar(0))
        return Overhead::NO_DATA;

    draft = BeamDraft(altimeter_->getSensorFrame(), clearance, (double)angle);
    if(draft >= sf::Scalar(0))
        return Overhead::ICE;

    //The vertical beam found nothing. If any of the wide fan still does, the vehicle is
    //jammed under a canopy rough enough that the outer beams reach ice the vertical one is
    //already past. If none of them does, the sounder genuinely cannot tell the open lead from
    //flat ice pressed right against the transducer - the vehicle is +2.0 N buoyant, so left
    //alone it rises until it is touching, and 30 mm of clearance is far inside the 0.2 m
    //blanking range. A real upward sonar goes blind in exactly the same way, which is why an
    //under-ice survey is flown several metres below the canopy rather than against it.
    return AnySwathReturn() ? Overhead::TOO_CLOSE : Overhead::NO_RETURN;
}

void UnderIceTestApp::DoTelemetry()
{
    BlueROV2TestApp::DoTelemetry();

    getGUI()->DoPanel(COL_X, ICE_PANEL_Y, COL_W, ICE_PANEL_H);

    char buf[128];
    const GLfloat tx = COL_X + 10.f;
    GLfloat ty = ICE_PANEL_Y + 12.f;

    sf::Scalar clearance = 0, draft = 0;
    const Overhead state = ReadOverhead(clearance, draft);

    const char* label = "--";
    switch(state)
    {
        case Overhead::NO_DATA:   label = "no data"; break;
        case Overhead::NO_RETURN: label = "NO RETURN"; break;
        case Overhead::TOO_CLOSE: label = "ICE - vertical beam blind"; break;
        case Overhead::ICE:       label = "ICE"; break;
    }
    snprintf(buf, sizeof(buf), "OVERHEAD: %s", label);
    getGUI()->DoLabel(tx, ty, std::string(buf));
    ty += 22.f;

    if(state == Overhead::ICE)
    {
        snprintf(buf, sizeof(buf), "Clearance %6.2f m", (double)clearance);
        getGUI()->DoLabel(tx, ty, std::string(buf));
        ty += 20.f;
        snprintf(buf, sizeof(buf), "Ice draft %6.2f m", (double)draft);
        getGUI()->DoLabel(tx, ty, std::string(buf));
        ty += 20.f;
    }
    else
    {
        getGUI()->DoLabel(tx, ty, "Clearance    --      Ice draft  --");
        ty += 20.f;
        //Do not guess which it is. Nothing the sounder measures separates an open lead from
        //ice sitting inside the 0.2 m blanking range, and saying "open water" while the
        //vehicle is touching the canopy would be worse than saying nothing.
        getGUI()->DoLabel(tx, ty, state == Overhead::TOO_CLOSE ? "closer than 0.2 m blanking"
                                                               : "open lead, or hard against ice");
        ty += 20.f;
    }

    //The standoff itself is the inherited ALTITUDE mode and is annunciated on the autopilot
    //panel in the right hand column, off the DVL. What belongs here is the draft instrument,
    //which reads the Profiler and is a different measurement of a different thing.

    snprintf(buf, sizeof(buf), "SONAR %s [F]  ICE CAM %s [U]  TILT %.0f [C]",
             showFLS_ ? "ON" : "OFF", showIceCamera_ ? "ON" : "OFF", CAM_TILT_DEG[camTilt_]);
    getGUI()->DoLabel(tx, ty, std::string(buf));
}

void UnderIceTestApp::DoHUD()
{
    //Draws the base HUD and sliders, and calls DoTelemetry() - which is ours.
    BlueROV2TestApp::DoHUD();

    sf::Uid id;
    id.owner = 11;

    //Both plots share one fixed vertical scale. Drafts go in negated, so that deeper ice
    //draws lower in the box.
    sf::Scalar range[2] = { -DRAFT_PLOT_MAX, sf::Scalar(0) };

    //--- Across-track ice-draft swath -------------------------------------------------------
    //One frame of the upward multibeam, as a profile rather than a time series: beam i is
    //plotted at position i, port on the left. A beam that found nothing is drawn at zero
    //draft, i.e. at the surface, which is what open water above the vehicle looks like.
    if(swath_ != nullptr)
    {
        const sf::Sample s = swath_->getLastSample();
        const unsigned short n = swath_->getNumOfChannels();
        const sf::Transform frame = swath_->getSensorFrame();
        const double half = underice::SWATH_HALF_FOV * M_PI / 180.0;

        swathPlot_[0].resize(n);
        for(unsigned short i=0; i<n; ++i)
        {
            const double a = (n > 1) ? ((double)i/(double)(n-1) * 2.0 * half - half) : 0.0;
            const sf::Scalar d = BeamDraft(frame, s.getValue(i), a);
            //A beam with no return is drawn at zero draft, i.e. at the surface, which is what
            //open water above the vehicle looks like.
            swathPlot_[0][i] = (GLfloat)-std::max(sf::Scalar(0), std::min(DRAFT_PLOT_MAX, d));
        }

        id.item = 0;
        getGUI()->DoTimePlot(id, COL_X, SWATH_PLOT_Y, COL_W, PLOT_H, swathPlot_,
                             "ICE DRAFT ACROSS TRACK 0-8 m (port-stbd)", range);
    }

    //--- Along-track ice-draft trace --------------------------------------------------------
    {
        sf::Scalar clearance = 0, draft = 0;
        const Overhead state = ReadOverhead(clearance, draft);
        const GLfloat plotted = (state == Overhead::ICE)
                              ? (GLfloat)std::max(sf::Scalar(0), std::min(DRAFT_PLOT_MAX, draft))
                              : 0.f;
        draftPlot_[0].push_back(-plotted);
        if(draftPlot_[0].size() > DRAFT_TRACE_LEN)
            draftPlot_[0].erase(draftPlot_[0].begin(),
                                draftPlot_[0].begin() + (draftPlot_[0].size() - DRAFT_TRACE_LEN));

        id.item = 1;
        getGUI()->DoTimePlot(id, COL_X, TRACE_PLOT_Y, COL_W, PLOT_H, draftPlot_,
                             "ICE DRAFT ALONG TRACK 0-8 m", range);
    }
}
