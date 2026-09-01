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
#include <entities/SolidEntity.h>
#include <graphics/IMGUI.h>
#include <core/Console.h>

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

    const char* ROBOT_NAME = "BLUEROV2";

    inline sf::Scalar Clamp(sf::Scalar v)
    {
        return std::max(sf::Scalar(-1), std::min(sf::Scalar(1), v));
    }

    //! Returns 1 when the key is held, 0 otherwise.
    inline sf::Scalar Held(const Uint8* keys, SDL_Scancode code)
    {
        return keys[code] ? sf::Scalar(1) : sf::Scalar(0);
    }
}

BlueROV2TestApp::BlueROV2TestApp(std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h, BlueROV2TestManager* sim)
    : GraphicalSimulationApp("BlueROV2Test", dataDirPath, s, h, sim),
      vehicleControl_(true),
      surge_(0), sway_(0), heave_(0), roll_(0), pitch_(0), yaw_(0)
{
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
    if(!vehicleControl_)
        return;

    //Polling rather than key events, so that held and simultaneous keys work naturally
    const Uint8* keys = SDL_GetKeyboardState(nullptr);

    if(keys[SDL_SCANCODE_SPACE]) //All stop
    {
        surge_ = sway_ = heave_ = roll_ = pitch_ = yaw_ = sf::Scalar(0);
        return;
    }

    surge_ = Held(keys, SDL_SCANCODE_W)      - Held(keys, SDL_SCANCODE_S);
    yaw_   = Held(keys, SDL_SCANCODE_D)      - Held(keys, SDL_SCANCODE_A);
    heave_ = Held(keys, SDL_SCANCODE_E)      - Held(keys, SDL_SCANCODE_Q);
    sway_  = Held(keys, SDL_SCANCODE_PERIOD) - Held(keys, SDL_SCANCODE_COMMA);
    pitch_ = Held(keys, SDL_SCANCODE_UP)     - Held(keys, SDL_SCANCODE_DOWN);
    roll_  = Held(keys, SDL_SCANCODE_RIGHT)  - Held(keys, SDL_SCANCODE_LEFT);
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
    getGUI()->DoPanel(x, yy, 250.f, 130.f);


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
}

void BlueROV2TestApp::DoHUD()
{
    GraphicalSimulationApp::DoHUD();

    ReadKeyboard();

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
    AllocateThrust();
}
