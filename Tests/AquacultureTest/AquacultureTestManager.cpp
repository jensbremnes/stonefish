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
//  AquacultureTestManager.cpp
//  Stonefish
//

#include "AquacultureTestManager.h"

//For brov2::WrapPi, which the school model needs to keep its phases bounded.
#include "../BlueROV2Test/PIDHold.h"

#include <core/Console.h>
#include <core/Robot.h>
#include <core/ScenarioParser.h>
#include <entities/AnimatedEntity.h>
#include <entities/SolidEntity.h>
#include <entities/animation/ManualTrajectory.h>
#include <entities/forcefields/Ocean.h>
#include <entities/forcefields/Uniform.h>
#include <graphics/OpenGLTrackball.h>
#include <sensors/scalar/Multibeam.h>
#include <utils/SystemUtil.hpp>

#include <cmath>
#include <cstdio>
#include <random>

AquacultureTestManager::AquacultureTestManager(sf::Scalar stepsPerSecond)
    : SimulationManager(stepsPerSecond, sf::Solver::SI, sf::CollisionFilter::EXCLUSIVE),
      schoolTime_(0)
{
}

void AquacultureTestManager::BuildScenario()
{
    sf::ScenarioParser parser(this);
    bool success = parser.Parse(sf::GetDataPath() + "fish_farm.scn");
    if(success)
        cInfo("Salmon farm scenario description parsed successfully.");
    else
        cError("Errors detected when parsing scenario description!");

    //Always replay the parser log, not just on failure. The parser reports recoverable problems
    //(a malformed <cg>, say, which is silently ignored) as errors while still returning success,
    //and those are otherwise invisible.
    {
        auto log = parser.getLog();
        for(size_t i=0; i<log.size(); ++i)
        {
            switch(log[i].type)
            {
                case sf::MessageType::INFO:
                    cInfo(log[i].text.c_str());
                    break;

                case sf::MessageType::ERROR:
                    cError(log[i].text.c_str());
                    break;

                case sf::MessageType::WARNING:
                    cWarning(log[i].text.c_str());
                    break;

                case sf::MessageType::CRITICAL:
                    //Deliberately not cCritical: that calls abort(), and replaying a logged
                    //message must not kill a run that the parser itself let through.
                    cError(log[i].text.c_str());
                    break;
            }
        }
    }

    if(!success)
        return;

    //ParseEnvironment builds the velocity fields from <current> but never switches them on -
    //Ocean::EnableCurrents() has no XML hook at all. Without this the tidal current in
    //fish_farm.scn does nothing, and neither does the lift-up plume.
    if(getOcean() != nullptr)
        getOcean()->EnableCurrents();

    CheckBakedCurrent();

    sf::Robot* rov = getRobot(farm::ROBOT);
    AddInspectionPayload(rov);
    AddSchool();

    //Start the scene camera on the vehicle rather than at the world origin, which in this
    //scenario is the middle of a pen full of fish, 20 m below the camera's default look-at.
    if(rov != nullptr && getTrackball() != nullptr && rov->getBaseLink() != nullptr)
        getTrackball()->GlueToMoving(rov->getBaseLink());
}

//! Warns if the scenario's current no longer matches the one the net was baked at.
//!
//! The net is a static mesh carrying a deformation solved once, in make_fish_farm.py, at
//! farm::DESIGN_CURRENT. Nothing at runtime can reshape it, so a scenario edited to a
//! different current is silently flying a net of the wrong shape - and the error is
//! invisible, because a net has no obviously correct shape to compare against.
void AquacultureTestManager::CheckBakedCurrent()
{
    if(getOcean() == nullptr)
        return;

    sf::Uniform* tidal = dynamic_cast<sf::Uniform*>(getOcean()->getVelocityField(0));
    if(tidal == nullptr)
        return;

    const sf::Vector3 baked(farm::DESIGN_CURRENT_X, farm::DESIGN_CURRENT_Y, 0.0);
    const sf::Vector3 actual = tidal->getVelocity();
    const sf::Scalar bakedSpeed = baked.length();
    if(bakedSpeed < 1e-6)
        return;

    if((actual - baked).length() > farm::CURRENT_TOLERANCE * bakedSpeed)
    {
        cWarning("Tidal current is (%.2f, %.2f, %.2f) m/s but the net was baked at "
                 "(%.2f, %.2f, 0.00). The net shape is wrong for this current - re-run "
                 "Tests/Data/tools/make_fish_farm.py after changing it.",
                 actual.x(), actual.y(), actual.z(), baked.x(), baked.y());
    }
}

//! Attaches the net-inspection payload to the BlueROV2 Heavy.
//!
//! WHY THIS IS C++ AND NOT XML. <sensor> is only legal inside <robot>, and
//! ScenarioParser::IncludeFiles only expands <include> elements that are DIRECT CHILDREN of
//! <scenario> - it never recurses into a robot. So a parent scenario cannot add sensors to a
//! vehicle it includes, and the alternatives were to duplicate all 561 lines of
//! bluerov2_heavy.scn or to push a net profiler onto the shared vehicle and make the other
//! two demos carry it.
//!
//! Everything attached here is named WITHOUT the robot prefix, unlike everything the parser
//! builds, which is why BlueROV2TestApp looks sensors up under both names.
void AquacultureTestManager::AddInspectionPayload(sf::Robot* rov)
{
    if(rov == nullptr)
    {
        cError("Robot '%s' not found - the inspection payload cannot be attached.", farm::ROBOT);
        return;
    }

    const std::string link = std::string(farm::ROBOT) + "/" + farm::LINK;

    //---- The net profiler ----
    //
    //A horizontal fan of raycasts across the bow. Multibeam::InternalUpdate sweeps in the
    //SENSOR FRAME'S X-Y PLANE about its Z axis, with beam i at
    //(i/steps - 0.5) * fov and the direction basisX*cos + basisY*sin, so an unrotated mount
    //gives a fan lying in the body's horizontal plane, centred on the bow and positive
    //to starboard. That is exactly what is wanted, so there is no rotation here.
    //
    //Why a fan and not a pair of splayed echosounders: the fan measures the net's range AND
    //its bearing AND the tear, from one sensor, and a least-squares fit over 33 beams is far
    //quieter than a two-beam difference. It is also free against the render budget -
    //Multibeam is pure raycasting and never touches the pipeline.
    //
    //The rays hit the smooth collision shell behind the twine, not the twine itself, which is
    //what a real echosounder does to a net panel and the reason the net carries a shell at all.
    //
    //x = 0.20 is INSIDE the visual hull's 0.222 m nose, unlike the cameras at 0.24. A head
    //that sticks out past the collision hull ends up inside whatever the hull is merely
    //touching, and a ray cast from inside a body exits without hitting anything - the
    //under-ice demo lost a whole session to that.
    sf::Multibeam* profiler = new sf::Multibeam(farm::NET_PROFILER, farm::PROFILER_FOV,
                                                farm::PROFILER_BEAMS, farm::PROFILER_RATE, 1);
    profiler->setRange(farm::PROFILER_RANGE_MIN, farm::PROFILER_RANGE_MAX);
    profiler->setNoise(0.01);
    rov->AddLinkSensor(profiler, link, sf::Transform(sf::IQ(), sf::Vector3(farm::PROFILER_X, 0.0, 0.0)));
    //Robot::AddLinkSensor only pushes onto the robot's own list; the handoff to the manager
    //happens in Robot::AddToSimulation, which the parser has already run. Both calls are needed.
    AddSensor(profiler);

    //---- THE FRAME BUDGET, AND WHY THERE IS NO SECOND CAMERA ----
    //
    //OpenGLPipeline renders every CONTINUOUS view each frame plus exactly ONE non-continuous
    //view, so what has to fit under the frame rate is the SUM of the vision sensors' update
    //rates, not their number. This scene carries about twice the ice scene's triangles and
    //draws in roughly 100 ms, so the budget is about ten view updates a second - and the
    //scenario's own bow camera, at 30 Hz, is three times over it on its own. It is turned
    //down here rather than in bluerov2_heavy.scn, which two other demos share.
    //
    //UnderIceTest adds a SECOND forward camera because its bow camera is tilted up at the
    //canopy and the pilot is left with no forward view at all. Nothing here tilts anything:
    //the bow camera already looks exactly where NET HOLD points the vehicle, straight at the
    //net. A second camera would be a second full pass over the scene for the same picture.
    sf::Sensor* bowCam = rov->getSensor(std::string(farm::ROBOT) + "/camera");
    if(bowCam != nullptr)
        bowCam->setUpdateFrequency(farm::BOW_CAM_RATE);
    else
        cError("Bow camera not found - its update rate is still the scenario 30 Hz.");
}

//! Builds the salmon school.
//!
//! A pen school is not a random cloud. Salmon in a sea cage swim as a POLARISED TORUS: nearly
//! all of them circling the same way about the pen axis, at a fairly narrow band of radii and
//! depths, at half to one body length a second. So each fish is given a radius, a depth, a
//! phase and an angular rate, and SimulationStepCompleted advances the phase - which is a
//! small simulation rather than an animation, and lets the school react to the vehicle.
//!
//! ManualTrajectory rather than a spline: 60 fish times a dozen keypoints is 800 lines of
//! generated XML in a hand-written scenario file, the loop seam has to be stitched by hand,
//! and the school could not then avoid anything.
void AquacultureTestManager::AddSchool()
{
    std::mt19937 rng(20260909u);
    std::uniform_real_distribution<double> uni(0.0, 1.0);

    school_.reserve(farm::FISH_COUNT);
    for(unsigned int i=0; i<farm::FISH_COUNT; ++i)
    {
        Fish f;
        //Radii are biased outward: a pen school runs close to the net wall and leaves a
        //quiet core in the middle. Depths are biased downward for the same reason - the
        //fish avoid the bright surface layer.
        f.radius = 9.0 + 8.0 * std::sqrt(uni(rng));
        f.depth = 2.5 + 8.5 * std::pow(uni(rng), 0.7);
        f.phase = 2.0 * M_PI * uni(rng);
        f.length = 0.60 + 0.16 * uni(rng);
        //0.55 body lengths a second, which is the low end of the cruising speed of farmed
        //salmon and about right for a pen with no strong internal current. One fish in
        //twelve swims against the school, which is also what a real pen looks like.
        const sf::Scalar speed = 0.55 * f.length;
        const sf::Scalar dir = uni(rng) < 0.92 ? 1.0 : -1.0;
        f.omega = dir * speed / f.radius;
        f.bobPhase = 2.0 * M_PI * uni(rng);
        f.bobRate = 0.18 + 0.22 * uni(rng);

        sf::ManualTrajectory* traj = new sf::ManualTrajectory();
        char name[32];
        std::snprintf(name, sizeof(name), "Salmon%02u", i);
        //collides = false, and that is load-bearing rather than merely cheap: Multibeam's
        //ray test masks in MASK_ANIMATED_COLLIDING, so a colliding school between the
        //vehicle and the net would corrupt every range the net profiler reports.
        sf::AnimatedEntity* fish = new sf::AnimatedEntity(name, traj,
                                                          sf::GetDataPath() + "farm/salmon.obj",
                                                          f.length / 0.68, sf::I4(),
                                                          "Salmon", "salmon", false);
        AddAnimatedEntity(fish);
        f.traj = traj;
        school_.push_back(f);
    }

    cInfo("Salmon school: %u fish, radii 9.0-17.0 m, depths 2.5-11.0 m.", farm::FISH_COUNT);
}

void AquacultureTestManager::SimulationStepCompleted(sf::Scalar timeStep)
{
    SimulationManager::SimulationStepCompleted(timeStep);

    if(school_.empty())
        return;

    schoolTime_ += timeStep;

    //Where the vehicle is, in pen-centred cylindrical coordinates, so the school can open up
    //around it. Farmed salmon do react to an ROV: they hold off a few metres and close in
    //again behind it.
    bool rovKnown = false;
    sf::Scalar rovR = 0, rovPhi = 0, rovZ = 0;
    sf::Robot* rov = getRobot(farm::ROBOT);
    if(rov != nullptr && rov->getBaseLink() != nullptr)
    {
        const sf::Vector3 p = rov->getBaseLink()->getCGTransform().getOrigin();
        const sf::Scalar dx = p.x() - farm::PEN_X;
        const sf::Scalar dy = p.y() - farm::PEN_Y;
        rovR = std::sqrt(dx*dx + dy*dy);
        rovPhi = std::atan2(dy, dx);
        rovZ = p.z();
        rovKnown = true;
    }

    for(size_t i=0; i<school_.size(); ++i)
    {
        Fish& f = school_[i];
        f.phase = brov2::WrapPi(f.phase + f.omega * timeStep);

        sf::Scalar r = f.radius + 0.9 * std::sin(f.bobPhase + 0.61 * f.bobRate * schoolTime_);
        sf::Scalar z = f.depth + 0.7 * std::sin(f.bobPhase * 1.7 + f.bobRate * schoolTime_);

        //Avoidance: a fish inside AVOID_R of the vehicle is pushed radially away from it,
        //fading to nothing at the edge. Radially only - a salmon that met an ROV would not
        //stop swimming, it would slide around it, which is what shifting the radius does.
        if(rovKnown)
        {
            const sf::Scalar AVOID_R = 3.5;
            const sf::Scalar dPhi = brov2::WrapPi(f.phase - rovPhi);
            const sf::Scalar dArc = dPhi * r;
            const sf::Scalar dz = z - rovZ;
            const sf::Scalar dr = r - rovR;
            const sf::Scalar dist = std::sqrt(dArc*dArc + dz*dz + dr*dr);
            if(dist < AVOID_R)
            {
                const sf::Scalar push = (AVOID_R - dist) * (dr >= 0 ? 1.0 : -1.0);
                r = std::max(sf::Scalar(4.0), std::min(sf::Scalar(17.5), r + push));
            }
        }

        const sf::Scalar c = std::cos(f.phase);
        const sf::Scalar s = std::sin(f.phase);
        const sf::Vector3 pos(farm::PEN_X + r * c, farm::PEN_Y + r * s, z);

        //The mesh points along +x, so heading is yaw and the fish faces along its own
        //tangent. The tail beat is a small yaw oscillation at about 1.4 Hz, which is the
        //tail beat frequency of a salmon at this swimming speed.
        const sf::Scalar heading = f.phase + (f.omega > 0 ? M_PI_2 : -M_PI_2);
        const sf::Scalar beat = 0.10 * std::sin(8.8 * schoolTime_ + f.bobPhase);
        const sf::Scalar pitch = -0.35 * f.bobRate * 0.7
                                 * std::cos(f.bobPhase * 1.7 + f.bobRate * schoolTime_);
        f.traj->setTransform(sf::Transform(sf::Quaternion(heading + beat, pitch, 0.0), pos));
        f.traj->setLinearVelocity(sf::Vector3(-r * f.omega * s, r * f.omega * c, 0.0));
        f.traj->setAngularVelocity(sf::Vector3(0.0, 0.0, f.omega));
    }
}
