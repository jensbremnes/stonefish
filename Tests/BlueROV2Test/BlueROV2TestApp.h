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
//  BlueROV2TestApp.h
//  Stonefish
//
//  Drives the BlueROV2 Heavy through a fixed 6 DOF thrust allocation, from the keyboard
//  or the on-screen sliders, and displays vehicle telemetry.
//

#ifndef __Stonefish__BlueROV2TestApp__
#define __Stonefish__BlueROV2TestApp__

#include <core/GraphicalSimulationApp.h>
#include "BlueROV2TestManager.h"

class BlueROV2TestApp : public sf::GraphicalSimulationApp
{
public:
    BlueROV2TestApp(std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h, BlueROV2TestManager* sim);

    void DoHUD() override;
    //! Consumes the piloting keys while in vehicle mode, so they do not also move the camera.
    void KeyDown(SDL_Event* event) override;

private:
    //! Reads the held keys and turns them into the 6 DOF demand.
    void ReadKeyboard();
    //! Distributes the 6 DOF demand onto the 8 thrusters.
    void AllocateThrust();
    //! Draws speed, depth and attitude.
    void DoTelemetry();

    bool vehicleControl_; //!< true = keys pilot the ROV, false = keys move the camera

    sf::Scalar surge_;
    sf::Scalar sway_;
    sf::Scalar heave_;
    sf::Scalar roll_;
    sf::Scalar pitch_;
    sf::Scalar yaw_;
};

#endif
