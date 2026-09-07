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
//  Carries an autopilot with three modes - vertical (depth or altitude), heading and attitude -
//  closed around the onboard pressure sensor, IMU and DVL. With a mode engaged its motion keys
//  move the SETPOINT instead of commanding thrust, so the vehicle is flown by nudging targets
//  rather than by continuous trimming.
//
//  Reusable: the constructor takes the window title and a plain sf::SimulationManager, so
//  any scenario that includes bluerov2_heavy.scn under the robot name BLUEROV2 can be flown
//  with it. UnderIceTest does exactly that, subclassing this to add its own instrumentation.
//

#ifndef __Stonefish__BlueROV2TestApp__
#define __Stonefish__BlueROV2TestApp__

#include "PIDHold.h"

#include <core/GraphicalSimulationApp.h>

namespace sf
{
    class ColorCamera;
    class DVL;
    class IMU;
    class Pressure;
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
    //! cycles the camera inset on V and the autopilot modes on Z, X and B.
    void KeyDown(SDL_Event* event) override;

protected:
    //! Size and placement of the forward camera inset.
    enum class CameraView { OFF, SMALL, LARGE };

    //! What the vertical loop is holding. Depth and altitude drive the same thrusters against
    //! the same vertical error, so they share one button and one controller.
    enum class VerticalHold { OFF, DEPTH, ALTITUDE };

    //! Setpoints still waiting to be seeded from the first good sensor fix. A subclass that
    //! wants a particular setpoint sets it in its constructor and clears the matching bit.
    enum SeedFlag
    {
        SEED_DEPTH    = 1,
        SEED_ALTITUDE = 2,
        SEED_HEADING  = 4,
        SEED_ATTITUDE = 8,
        SEED_ALL      = 15
    };

    //! The vehicle state the autopilot flies on, read from the onboard sensors rather than
    //! from the rigid body - which is what DoTelemetry() displays, and why the two can differ
    //! by a little noise.
    struct State
    {
        bool valid;             //!< the pressure sensor and IMU have both produced a sample
        sf::Scalar depth;       //!< of the vehicle ORIGIN, m, positive down
        sf::Scalar depthRate;   //!< m/s, positive sinking
        bool altitudeValid;     //!< the DVL has a bottom track
        sf::Scalar altitude;    //!< vertical distance to whatever the DVL faces, m
        sf::Scalar altitudeSign;//!< d(altitude)/d(depth): +1 looking up, -1 looking down
        sf::Scalar roll, pitch, yaw; //!< rad
        sf::Scalar p, q, r;     //!< body rates, rad/s, filtered
    };

    //! Pushes cameraView_ to the camera sensor, resolving the sensor on first use.
    void ApplyCameraView();
    //! Called every frame between the autopilot and thrust allocation, so a subclass can close
    //! a loop of its own around one of the six demands. Does nothing here.
    virtual void AugmentDemands(sf::Scalar dt);
    //! Draws speed, depth and attitude. Virtual so a subclass can append its own readouts.
    virtual void DoTelemetry();
    //! Draws the autopilot panel in the right hand column.
    void DoAutopilotPanel();

    //! Mode changes. These seed the setpoint from the current measurement and reset the
    //! integrator, so engaging a loop never kicks.
    void SetVerticalHold(VerticalHold m);
    void SetHeadingHold(bool on);
    void SetAttitudeHold(bool on);

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

    //! Autopilot mode and setpoints. Protected so a subclass can pick its own defaults;
    //! UnderIceTest starts in ALTITUDE at a fixed standoff.
    VerticalHold verticalHold_;
    bool headingHold_;
    bool attitudeHold_;
    sf::Scalar depthSp_;     //!< m, positive down
    sf::Scalar altitudeSp_;  //!< m
    sf::Scalar headingSp_;   //!< rad, wrapped
    sf::Scalar pitchSp_;     //!< rad
    sf::Scalar rollSp_;      //!< rad
    unsigned int pendingSeed_;

    //! How fast each setpoint is being slewed right now, in the units of that setpoint. Fed to
    //! the loops so their D terms measure the error rate rather than the vehicle rate: while the
    //! pilot holds a key the vehicle is SUPPOSED to be moving, and damping it as though it were
    //! a disturbance is what forces the loop to build a tracking error and then overshoot when
    //! the key is released.
    sf::Scalar depthSpRate_;
    sf::Scalar headingSpRate_;
    sf::Scalar pitchSpRate_;
    sf::Scalar rollSpRate_;

    State state_;

private:
    //! Resolves the pressure sensor, IMU and DVL. They do not exist at construction time -
    //! the scenario is built during Init() - so this runs once on the first frame.
    void ResolveSensors();
    //! Fills state_ from those sensors.
    void ReadState(sf::Scalar dt);
    //! Reads the held keys and ramps the 6 DOF demand toward what they ask for. Axes owned by
    //! an engaged hold are ramped to zero instead; the autopilot writes them afterwards.
    void ReadKeyboard(sf::Scalar dt);
    //! Moves the setpoints of the engaged modes with the same keys that would otherwise
    //! command thrust on those axes.
    void UpdateSetpoints(sf::Scalar dt);
    //! Runs the engaged loops and writes their demands.
    void RunHolds(sf::Scalar dt);
    //! Distributes the 6 DOF demand onto the 8 thrusters.
    void AllocateThrust();
    //! Wall clock seconds since the previous frame, clamped.
    sf::Scalar FrameDelta();
    //! Points every setpoint at where the vehicle is right now.
    void SyncSetpoints();

    uint64_t lastRampTime_; //!< for the demand ramp, in microseconds; 0 until the first frame

    bool sensorsResolved_;
    sf::Pressure* pressure_;
    sf::IMU* imu_;
    sf::DVL* dvl_;

    brov2::Derivative depthRate_;
    brov2::LowPass rateP_, rateQ_, rateR_;
    //! Depth and altitude get separate controllers despite driving the same axis: the DVL is a
    //! far noisier measurement than the pressure sensor and cannot take the same gains.
    brov2::PIDHold depthPid_, altitudePid_, headingPid_, pitchPid_, rollPid_;

    //! ALTITUDE is engaged but the DVL has no bottom track, so the vertical loop is flying
    //! depth instead. Latched so the console says so once rather than every frame.
    bool altitudeLost_;
};

#endif
