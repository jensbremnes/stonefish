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
//  UnderIceTestManager.cpp
//  Stonefish
//

#include "UnderIceTestManager.h"

#include <actuators/Light.h>
#include <core/Console.h>
#include <core/Robot.h>
#include <core/ScenarioParser.h>
#include <entities/SolidEntity.h>
#include <entities/forcefields/Ocean.h>
#include <graphics/OpenGLTrackball.h>
#include <sensors/scalar/Multibeam.h>
#include <sensors/scalar/Profiler.h>
#include <sensors/vision/ColorCamera.h>
#include <sensors/vision/FLS.h>
#include <utils/SystemUtil.hpp>

#include <cmath>

UnderIceTestManager::UnderIceTestManager(sf::Scalar stepsPerSecond)
    : SimulationManager(stepsPerSecond, sf::Solver::SI, sf::CollisionFilter::EXCLUSIVE)
{
}

void UnderIceTestManager::BuildScenario()
{
    sf::ScenarioParser parser(this);
    bool success = parser.Parse(sf::GetDataPath() + "under_ice.scn");
    if(success)
        cInfo("Under-ice scenario description parsed successfully.");
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
    //Ocean::EnableCurrents() has no XML hook at all. Without this the 0.04 m/s drift in
    //under_ice.scn does nothing.
    if(getOcean() != nullptr)
        getOcean()->EnableCurrents();

    sf::Robot* rov = getRobot(underice::ROBOT);
    AddIcePayload(rov);

    //Start the scene camera on the vehicle rather than at the world origin. Without this the
    //trackball opens looking at the sea surface from above, which is a poor first view of a
    //scene whose entire subject is 2 m under the ice. Mouse drag and wheel still work as
    //usual; the camera simply follows the vehicle around.
    if(rov != nullptr && getTrackball() != nullptr && rov->getBaseLink() != nullptr)
        getTrackball()->GlueToMoving(rov->getBaseLink());
}

//! Attaches the under-ice sensor payload to the BlueROV2 Heavy.
//!
//! WHY THIS IS C++ AND NOT XML. <sensor> is only legal inside <robot>, and
//! ScenarioParser::IncludeFiles only expands <include> elements that are DIRECT CHILDREN of
//! <scenario> - it never recurses into a robot. So a parent scenario cannot add sensors to a
//! vehicle it includes. The alternatives were to duplicate all 515 lines of
//! bluerov2_heavy.scn, or to push an FLS and a second camera onto the shared vehicle and make
//! the open-water demo pay for rendering them every frame. Neither is worth it for five
//! devices, so they are attached here instead.
//!
//! THE FRAME BUDGET. OpenGLPipeline renders every CONTINUOUS view each frame plus exactly ONE
//! non-continuous view (OpenGLPipeline.cpp, "Update the queue of views needing update"): views
//! that want an update are queued, and only one is taken off the queue per frame. Every camera
//! and sonar here is non-continuous, so the sum of their rates is a hard budget against the
//! frame rate, and this scene draws in about 67 ms - roughly 15 frames, and therefore 15 view
//! updates, per second.
//!
//! The fit was already over that budget before this camera existed: a 30 Hz bow camera and a
//! 5 Hz FLS ask for 35 updates a second against 15 available. The queue then never drains, and
//! the symptom is the one recorded when an up-looking camera was first tried and removed - the
//! continuous trackball view gets drawn after the insets and paints over them, so they flicker.
//! The conclusion drawn at the time was that three rendered views is one too many; the real
//! limit is the sum of their rates, and two 30 Hz views would have been just as bad.
//!
//! So the rates are set to fit: 8 + 3 + 3 = 14 with all three displayed, and UnderIceTestApp
//! disables the OpenGL view behind anything whose display is switched off, which is what buys
//! the margin back. A hidden camera used to render every frame for nobody.
//!
//! Robot::AddLinkSensor and AddVisionSensor only push onto the robot's own list; the handoff
//! to the manager happens in Robot::AddToSimulation, which the parser has already run by the
//! time we get here. Each device therefore needs BOTH the Add*ToRobot call AND an explicit
//! AddSensor/AddActuator, or it will exist, be attached, and never update. Attaching is also
//! what creates the OpenGL side (VisionSensor::AttachToSolid and Light::AttachToSolid both
//! call InitGraphics), so this must run after the render context exists - which it does,
//! BuildScenario is called from Init().
//!
//! MOUNTING. The parser prefixes link names with the robot name, so the link is
//! "BLUEROV2/Vehicle" even though the sensors themselves are given unprefixed names here.
//! sf::Transform takes sf::Quaternion(yaw, pitch, roll) - the REVERSE of the XML rpy order.
void UnderIceTestManager::AddIcePayload(sf::Robot* rov)
{
    if(rov == nullptr)
    {
        cError("Robot '%s' not found - the under-ice payload was not attached.", underice::ROBOT);
        return;
    }

    const std::string link = std::string(underice::ROBOT) + "/" + underice::LINK;

    //Multibeam and Profiler sweep their beam in the sensor's XY plane, centred on +X
    //(Multibeam.cpp:62, Profiler.cpp:58). Ry(+pi/2) sends +X to body -Z, i.e. straight UP,
    //and leaves +Y on body +Y, so the multibeam fan lies athwartships and sweeps from port
    //to starboard through the vertical - an across-track ice-draft swath. Getting the sign
    //of that pitch wrong points the heads at the seabed instead, and they still report
    //perfectly plausible numbers, so it is worth checking the printed draft against the
    //heightmap the first time this is touched.
    const sf::Transform mastUp(sf::Quaternion(0.0, M_PI_2, 0.0),
                               sf::Vector3(0.0, 0.0, underice::MAST_Z));

    //--- Upward single-beam ice-draft sounder ---------------------------------------------
    //A 1 degree beam, the same order as a real upward-looking sonar (an ASL IPS is 1.8 deg).
    //ice draft = depth of the head - range, which is exactly how a moored ULS measures it.
    //1000 samples of history so the HUD can plot the draft along the track.
    sf::Profiler* alt = new sf::Profiler(underice::ALTIMETER, 1.0, 2, 10.0, 1000);
    alt->setRange(0.2, underice::UP_RANGE_MAX);
    alt->setNoise(0.01);
    rov->AddLinkSensor(alt, link, mastUp);
    AddSensor(alt);

    //--- Upward multibeam, across-track ice draft ------------------------------------------
    //129 beams over 120 degrees. The 0.45 m minimum range is not cosmetic: the rays are cast
    //from the sensor origin outward and the filter mask includes MASK_DYNAMIC, so the outer
    //beams of the fan would otherwise strike the vehicle's own outboard thruster mounts,
    //which reach 0.24 m outboard at z = 0.
    sf::Multibeam* swath = new sf::Multibeam(underice::MULTIBEAM, 2.0 * underice::SWATH_HALF_FOV, 128, 5.0, 1);
    swath->setRange(0.45, underice::UP_RANGE_MAX);
    swath->setNoise(0.02);
    rov->AddLinkSensor(swath, link, mastUp);
    AddSensor(swath);

    //--- Forward-looking sonar, for finding keels ------------------------------------------
    //Tilted 12 degrees UP so the fan sweeps the underside of the canopy ahead rather than the
    //seabed. bluerov2_heavy.scn derives rpy = (pi/2 - a, 0, pi/2) for a vision sensor aimed
    //forward and tilted DOWN by a, so a negative a tilts it up. 30 m of range at 90 degrees
    //covers the approach to a keel comfortably; the ice returns strongly because the SeaIce
    //material's restitution is its acoustic reflectivity (see under_ice.scn).
    const double flsTiltUp = 12.0 * M_PI / 180.0;
    //5 Hz, not the default "every simulation step". A real 30 m FLS pings about 20 times a
    //second, but each ping is a full render of the scene, and the display cannot be read any
    //faster than this anyway.
    sf::FLS* fls = new sf::FLS(underice::FLS, 256, 400, 90.0, 20.0, 0.5, 30.0,
                               sf::ColorMap::HOT, sf::SonarOutputFormat::U8, underice::FLS_RATE);
    fls->setNoise(0.03, 0.04);
    rov->AddVisionSensor(fls, link,
                         sf::Transform(sf::Quaternion(M_PI_2, 0.0, M_PI_2 + flsTiltUp),
                                       sf::Vector3(0.24, 0.0, 0.05)));
    AddSensor(fls);

    //--- Forward pilot camera ---------------------------------------------------------------
    //The scenario's own bow camera is on a tilt servo and under ice it lives at the top of its
    //travel, aimed straight up at the canopy - which leaves the pilot with no forward view at
    //all. This is that view: fixed, forward, and never tilted.
    //
    //It looks out of the lower enclosure tube so it does not occupy the same point as the
    //scenario camera in the upper one. A vision sensor looks along its own +Z with -Y as image
    //up, so aiming it forward needs rpy = (pi/2, 0, pi/2) - and sf::Quaternion takes
    //(yaw, pitch, roll), the REVERSE of the XML rpy order.
    //
    //640x360 rather than the bow camera's 1280x720: it is shown as an inset a few hundred
    //pixels wide, and a camera render is a full pass over the scene.
    sf::ColorCamera* fwdCam = new sf::ColorCamera(underice::FWD_CAMERA,
                                                  underice::FWD_CAM_RES_X, underice::FWD_CAM_RES_Y,
                                                  80.0, underice::FWD_CAM_RATE);
    rov->AddVisionSensor(fwdCam, link,
                         sf::Transform(sf::Quaternion(M_PI_2, 0.0, M_PI_2),
                                       sf::Vector3(underice::FWD_CAM_X, 0.0, underice::FWD_CAM_Z)));
    AddSensor(fwdCam);

    //--- Turn the bow camera down -----------------------------------------------------------
    //See THE FRAME BUDGET below. It is watching ice drift slowly past a metre or two overhead;
    //it does not need the 30 Hz the open-water pilot view is given.
    sf::Sensor* bowCam = rov->getSensor(std::string(underice::ROBOT) + "/camera");
    if(bowCam != nullptr)
        bowCam->setUpdateFrequency(underice::ICE_CAM_RATE);
    else
        cWarning("Bow camera not found - it will keep its open-water rate and may cost frames.");

    //--- Up-looking light --------------------------------------------------------------------
    //A light is an actuator, not a view, so this one costs nothing in the frame budget. It has to
    //exist: the two Lumens on the standard fit are aimed forward and 15 degrees DOWN, and
    //light none of the canopy. A <light> emits along its own +Z, so it takes the same rotation
    //a camera would - Rz(-pi/2)Rx(pi) puts that axis on body -Z, straight up.
    const sf::Quaternion lookUp(-M_PI_2, 0.0, M_PI);
    sf::Light* iceLight = new sf::Light(underice::LIGHT, 0.02, 100.0, sf::Color::BlackBody(5000.f), 15000.0);
    rov->AddLinkActuator(iceLight, link, sf::Transform(lookUp, sf::Vector3(0.10, 0.0, underice::LIGHT_Z)));
    AddActuator(iceLight);

    cInfo("Under-ice payload attached: upward sounder, %d-beam upward swath, forward sonar,"
          " forward camera and up-light.", (int)(swath->getNumOfChannels()));
}
