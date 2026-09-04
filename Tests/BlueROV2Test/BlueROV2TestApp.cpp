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
#include <graphics/IMGUI.h>
#include <sensors/vision/ColorCamera.h>
#include <core/Console.h>
#include <utils/SystemUtil.hpp>

#include <algorithm>
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
    //! straight to its maximum.
    const sf::Scalar RAMP_MAX_DT = sf::Scalar(0.1);

    const char* ROBOT_NAME = "BLUEROV2";
    const char* CAMERA_NAME = "camera";

    //Gap between the small camera inset and the window edge, in pixels
    const unsigned int MARGIN = 10;

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
}

BlueROV2TestApp::BlueROV2TestApp(std::string title, std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h,
                                 sf::SimulationManager* sim)
    : GraphicalSimulationApp(title, dataDirPath, s, h, sim),
      vehicleControl_(true),
      lastRampTime_(0),
      cameraView_(CameraView::SMALL),
      camera_(nullptr),
      cameraResolved_(false),
      surge_(0), sway_(0), heave_(0), roll_(0), pitch_(0), yaw_(0)
{
    //The robot does not exist yet - the scenario is built during Init(), after construction -
    //so the camera is resolved on the first frame instead, in ProcessInputs().
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

void BlueROV2TestApp::ReadKeyboard()
{
    //Wall clock rather than simulation time, so the ramp feels the same whatever the frame
    //rate or the real-time factor
    const uint64_t now = sf::GetTimeInMicroseconds();
    sf::Scalar dt = lastRampTime_ == 0 ? sf::Scalar(0)
                                       : sf::Scalar(now - lastRampTime_) / sf::Scalar(1000000);
    lastRampTime_ = now;
    dt = std::min(dt, RAMP_MAX_DT);

    if(!vehicleControl_)
        return;

    //Polling rather than key events, so that held and simultaneous keys work naturally
    const Uint8* keys = SDL_GetKeyboardState(nullptr);

    if(keys[SDL_SCANCODE_SPACE]) //All stop, immediately - no ramp
    {
        surge_ = sway_ = heave_ = roll_ = pitch_ = yaw_ = sf::Scalar(0);
        return;
    }

    //The keys select a target and the demand ramps toward it. Releasing a key targets zero,
    //so the vehicle eases off over the same half second rather than cutting dead.
    RampTowards(surge_, (Held(keys, SDL_SCANCODE_W)      - Held(keys, SDL_SCANCODE_S))     * GAIN_SURGE, dt);
    RampTowards(yaw_,   (Held(keys, SDL_SCANCODE_D)      - Held(keys, SDL_SCANCODE_A))     * GAIN_YAW,   dt);
    RampTowards(heave_, (Held(keys, SDL_SCANCODE_E)      - Held(keys, SDL_SCANCODE_Q))     * GAIN_HEAVE, dt);
    RampTowards(sway_,  (Held(keys, SDL_SCANCODE_PERIOD) - Held(keys, SDL_SCANCODE_COMMA)) * GAIN_SWAY,  dt);
    RampTowards(pitch_, (Held(keys, SDL_SCANCODE_UP)     - Held(keys, SDL_SCANCODE_DOWN))  * GAIN_PITCH, dt);
    RampTowards(roll_,  (Held(keys, SDL_SCANCODE_RIGHT)  - Held(keys, SDL_SCANCODE_LEFT))  * GAIN_ROLL,  dt);
}

void BlueROV2TestApp::AllocateThrust()
{
    sf::Robot* rov = getSimulationManager()->getRobot(ROBOT_NAME);
    if(rov == nullptr)
        return;

    //The parser prefixes actuator names with the robot name
    for(size_t i=0; i<4; ++i)
    {
        sf::Thruster* th = dynamic_cast<sf::Thruster*>(rov->getActuator(std::string(ROBOT_NAME) + "/" + HORIZONTAL[i].name));
        if(th == nullptr)
            th = dynamic_cast<sf::Thruster*>(rov->getActuator(HORIZONTAL[i].name));
        if(th != nullptr)
            th->setSetpoint(Clamp(HORIZONTAL[i].a * surge_ + HORIZONTAL[i].b * sway_ + HORIZONTAL[i].c * yaw_));
    }

    for(size_t i=0; i<4; ++i)
    {
        sf::Thruster* th = dynamic_cast<sf::Thruster*>(rov->getActuator(std::string(ROBOT_NAME) + "/" + VERTICAL[i].name));
        if(th == nullptr)
            th = dynamic_cast<sf::Thruster*>(rov->getActuator(VERTICAL[i].name));
        if(th != nullptr)
            th->setSetpoint(Clamp(VERTICAL[i].a * heave_ + VERTICAL[i].b * roll_ + VERTICAL[i].c * pitch_));
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
            //The parser prefixes sensor names with the robot name, and a robot's sensors live
            //on the robot rather than in the manager's list
            sf::Sensor* s = rov->getSensor(std::string(ROBOT_NAME) + "/" + CAMERA_NAME);
            if(s == nullptr)
                s = rov->getSensor(CAMERA_NAME);
            camera_ = dynamic_cast<sf::ColorCamera*>(s);
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

    ReadKeyboard();
    AugmentDemands();
    AllocateThrust();
}

void BlueROV2TestApp::AugmentDemands()
{
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

void BlueROV2TestApp::DoHUD()
{
    GraphicalSimulationApp::DoHUD();

    //Piloting itself runs in ProcessInputs(), so that hiding the HUD does not stop the vehicle.
    //The sliders below are still a second way in: the keyboard simply overwrites them every
    //frame while in vehicle mode.
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
}
