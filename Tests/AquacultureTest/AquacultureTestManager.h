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
//  AquacultureTestManager.h
//  Stonefish
//
//  Loads the salmon farm scenario from fish_farm.scn, bolts the net-inspection payload onto
//  the BlueROV2 Heavy that scenario includes, and drives the salmon school.
//

#ifndef __Stonefish__AquacultureTestManager__
#define __Stonefish__AquacultureTestManager__

#include <core/SimulationManager.h>

#include <vector>

namespace sf
{
    class AnimatedEntity;
    class ManualTrajectory;
    class Robot;
}

//! Site geometry and payload names, shared with AquacultureTestApp.
//!
//! These are the numbers fish_farm.scn and make_fish_farm.py are built from. They are
//! repeated here rather than read back out of the scenario because the controller needs
//! them before anything has been sensed, and because a mismatch should be a compile-time
//! edit in one place rather than a silent flight into a net.
namespace farm
{
    constexpr const char* ROBOT = "BLUEROV2";
    constexpr const char* LINK = "Vehicle";

    constexpr const char* NET_PROFILER = "net_profiler";
    constexpr const char* FLS = "farm_fls";
    constexpr const char* LIGHT_PORT = "net_light_port";
    constexpr const char* LIGHT_STBD = "net_light_stbd";

    //! The hero pen. Centre in the world, net radius, and the depths at which the
    //! cylindrical wall ends and the cone reaches the axis.
    constexpr double PEN_X = 0.0;
    constexpr double PEN_Y = 0.0;
    constexpr double PEN_RADIUS = 19.0986;   //!< a 120 m circumference collar
    constexpr double PEN_WALL_DEPTH = 15.0;  //!< m, bottom of the cylindrical wall
    constexpr double PEN_CONE_DEPTH = 20.0;  //!< m, the cone's apex

    //! The design current, in m/s. The net mesh is BAKED at this value; the manager warns
    //! at startup if fish_farm.scn disagrees, because a net baked at one current and flown
    //! in another is silently the wrong shape.
    constexpr double DESIGN_CURRENT_X = 0.0;
    constexpr double DESIGN_CURRENT_Y = 0.30;
    constexpr double CURRENT_TOLERANCE = 0.2;   //!< fractional

    //! Where the net profiler sits, in the NED body frame. INSIDE the visual hull's 0.222 m
    //! nose, unlike the cameras at 0.24: the under-ice demo's hardest-won lesson is that a
    //! head sticking out past the collision hull ends up inside whatever the hull is merely
    //! touching, and a ray cast from inside a body exits without hitting anything.
    constexpr double PROFILER_X = 0.20;

    //! Fan of the net profiler. 90 degrees over 48 beams is 1.9 degrees a beam, which at the
    //! default 1.5 m standoff resolves the net to 5 cm and spans 3.0 m of it.
    //!
    //! THE WIDTH IS SET BY THE TEAR, NOT BY THE RESOLUTION. A 60 degree fan spans only 1.7 m
    //! at that standoff, so the 0.8 m tear takes out nearly half of it at once and what is
    //! left is two short groups of beams at the edges - which fits a line badly, and badly in
    //! a way that then steers the vehicle. At 90 degrees the tear is a quarter of the fan and
    //! the beams either side of it still pin the net plane down.
    //!
    //! It costs nothing to widen: Multibeam is pure raycasting, so this is 490 rays a second
    //! against a 3000 triangle collision scene, and it never touches the render queue.
    constexpr double PROFILER_FOV = 90.0;
    constexpr unsigned int PROFILER_BEAMS = 48;
    constexpr double PROFILER_RATE = 10.0;
    constexpr double PROFILER_RANGE_MIN = 0.25;
    constexpr double PROFILER_RANGE_MAX = 14.0;

    //! Vision sensor rates. The pipeline renders every continuous view plus exactly ONE
    //! non-continuous view per frame, so what has to fit under the frame rate is the SUM of
    //! these, not their count. This scene carries about twice the ice scene's triangles, so
    //! the budget is smaller; see THE FRAME BUDGET in AquacultureTestManager.cpp.
    //! The scenario bow camera is the only rendered view, and it is the pilot view. Four
    //! updates a second against a scene that draws in about 100 ms leaves margin; see THE
    //! FRAME BUDGET in AquacultureTestManager.cpp.
    constexpr double BOW_CAM_RATE = 4.0;

    //! The school.
    constexpr unsigned int FISH_COUNT = 60;
}

class AquacultureTestManager : public sf::SimulationManager
{
public:
    AquacultureTestManager(sf::Scalar stepsPerSecond);

    void BuildScenario() override;
    //! Swims the school. Called once per physics step.
    void SimulationStepCompleted(sf::Scalar timeStep) override;

private:
    //! Attaches the net-inspection payload to an already-parsed robot.
    void AddInspectionPayload(sf::Robot* rov);
    //! Builds the salmon school and its trajectories.
    void AddSchool();
    //! Checks that the scenario's current still matches the one the net was baked at.
    void CheckBakedCurrent();

    //! One salmon's place in the school. See AddSchool() for the model.
    struct Fish
    {
        sf::ManualTrajectory* traj;
        sf::Scalar radius;      //!< m from the pen axis
        sf::Scalar depth;       //!< m
        sf::Scalar phase;       //!< rad around the pen axis
        sf::Scalar omega;       //!< rad/s, signed: most fish swim the same way
        sf::Scalar length;      //!< m, fork length
        sf::Scalar bobPhase;    //!< rad
        sf::Scalar bobRate;     //!< rad/s
    };

    std::vector<Fish> school_;
    sf::Scalar schoolTime_;
};

#endif
