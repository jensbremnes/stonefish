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
//  or the on-screen sliders, displays vehicle telemetry, and shows the forward camera as
//  a resizable inset so the vehicle can be flown on its own picture.
//
//  Reusable: the constructor takes the window title and a plain sf::SimulationManager, so
//  any scenario that includes bluerov2_heavy.scn under the robot name BLUEROV2 can be flown
//  with it. UnderIceTest does exactly that, subclassing this to add its own instrumentation.
//

#ifndef __Stonefish__BlueROV2TestApp__
#define __Stonefish__BlueROV2TestApp__

#include <core/GraphicalSimulationApp.h>

namespace sf
{
    class ColorCamera;
    class SimulationManager;
}

class BlueROV2TestApp : public sf::GraphicalSimulationApp
{
public:
    BlueROV2TestApp(std::string title, std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h,
                    sf::SimulationManager* sim);

    void DoHUD() override;
    //! Runs the piloting loop. Called every frame, unlike DoHUD(), which stops being called
    //! when the HUD is hidden with H - and hiding the HUD is exactly what you do to fly on
    //! the camera picture.
    void ProcessInputs() override;
    //! Consumes the piloting keys while in vehicle mode, so they do not also move the camera,
    //! and cycles the camera inset on V.
    void KeyDown(SDL_Event* event) override;

protected:
    //! Size and placement of the forward camera inset.
    enum class CameraView { OFF, SMALL, LARGE };

    //! Pushes cameraView_ to the camera sensor, resolving the sensor on first use.
    void ApplyCameraView();
    //! Called every frame between reading the keyboard and allocating thrust, so a subclass
    //! can close a loop around one of the six demands. Does nothing here.
    virtual void AugmentDemands();
    //! Draws speed, depth and attitude. Virtual so a subclass can append its own readouts.
    virtual void DoTelemetry();

    bool vehicleControl_; //!< true = keys pilot the ROV, false = keys move the camera

    CameraView cameraView_;
    sf::ColorCamera* camera_;   //!< nullptr until the scenario is built, and if lookup fails
    bool cameraResolved_;       //!< lookup has been attempted; do not retry it every frame

    //! The six normalised demands, in the NED body frame. Positive heave is DOWNWARD.
    sf::Scalar surge_;
    sf::Scalar sway_;
    sf::Scalar heave_;
    sf::Scalar roll_;
    sf::Scalar pitch_;
    sf::Scalar yaw_;

private:
    //! Reads the held keys and ramps the 6 DOF demand toward what they ask for.
    void ReadKeyboard();
    //! Distributes the 6 DOF demand onto the 8 thrusters.
    void AllocateThrust();

    uint64_t lastRampTime_; //!< for the demand ramp, in microseconds; 0 until the first frame
};

#endif
