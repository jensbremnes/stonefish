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
//  main.cpp
//  AquacultureTest
//

#include <cstdio>
#include "AquacultureTestApp.h"
#include "AquacultureTestManager.h"

int main(int argc, const char * argv[])
{
    //Unbuffered stdout: a cCritical() calls abort(), and buffered output is lost on the way
    //out, which makes startup failures look like a silent crash with no message at all.
    setvbuf(stdout, nullptr, _IONBF, 0);
    sf::RenderSettings s;
    s.windowW = 1200;
    s.windowH = 900;
    //Set low for the same reason UnderIceTest is: OpenGLPipeline submits every object in the
    //scene for every pass with no frustum culling, and this scene carries about twice the ice
    //scene's triangles - a 143k triangle seabed, 67k of netting and 60 fish. When the frame
    //rate drops below the sum of the vision sensors' update rates the view queue stops
    //draining and the on-screen insets flicker, so frame rate here buys correctness.
    //
    //Unlike under the ice there IS a sun worth having: the scene is 25 degrees of elevation
    //in coastal water and the surface is in shot. Atmosphere and ocean therefore stay at
    //MEDIUM where the ice scene drops them to LOW.
    s.aa = sf::RenderQuality::MEDIUM;
    //DISABLED, and this is the single biggest frame-time knob in the scene. OpenGLPipeline
    //bakes a shadowmap for every ACTIVE light before it draws anything, gated only on this
    //setting, and OpenGLSpotLight is the only light that actually overrides BakeShadowmap -
    //so the vehicle two Lumens cost two extra full passes over a 230k triangle scene every
    //frame, on top of the sun cascades. Underwater at 3 m in coastal water with the sun at
    //25 degrees of elevation there is almost nothing for them to cast onto.
    s.shadows = sf::RenderQuality::DISABLED;
    s.ao = sf::RenderQuality::LOW;
    s.atmosphere = sf::RenderQuality::MEDIUM;
    s.ocean = sf::RenderQuality::MEDIUM;
    s.ssr = sf::RenderQuality::DISABLED;
    s.verticalSync = true;

    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    //OFF, unlike UnderIceTest. Ticking "Sensors" in the DEBUG panel draws the net
    //profiler's fan against the net, which is the quickest way to see what the
    //net hold is ranging on - but ManualTrajectory::Render() emits a SENSOR_CS
    //renderable, so the same switch also stamps a coordinate cross on all sixty salmon.
    //It is left to the panel rather than defaulted on.
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;

    AquacultureTestManager* simulationManager = new AquacultureTestManager(200.0);
    AquacultureTestApp app(std::string(DATA_DIR_PATH), s, h, simulationManager);
    app.Run();

    return 0;
}
