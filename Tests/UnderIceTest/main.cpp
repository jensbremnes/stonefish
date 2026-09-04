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
//  UnderIceTest
//

#include <cstdio>
#include "UnderIceTestApp.h"
#include "UnderIceTestManager.h"

int main(int argc, const char * argv[])
{
    //Unbuffered stdout: a cCritical() calls abort(), and buffered output is lost on the way out,
    //which makes startup failures look like a silent crash with no message at all.
    setvbuf(stdout, nullptr, _IONBF, 0);
    sf::RenderSettings s;
    s.windowW = 1200;
    s.windowH = 900;
    //Lower than BlueROV2Test across the board, and deliberately so. This scene renders five
    //views instead of two - the main one, the pilot camera and the forward sonar, on top of
    //the sun's shadow cascades - over an ice canopy that OpenGLPipeline submits in full for
    //every one of them, because it does no frustum culling. When the frame rate drops far
    //enough that the cameras cannot be serviced every frame, the pipeline starts round-robin
    //updating them and the on-screen insets visibly flicker, so frame rate here buys
    //correctness and not just smoothness.
    //Nothing lost is worth much under the ice: there is no sun to speak of at 6 degrees of
    //elevation under an opaque canopy, and screen-space reflections have nothing to reflect.
    s.aa = sf::RenderQuality::MEDIUM;
    s.shadows = sf::RenderQuality::LOW;
    s.ao = sf::RenderQuality::LOW;
    s.atmosphere = sf::RenderQuality::LOW;
    s.ocean = sf::RenderQuality::LOW;
    s.ssr = sf::RenderQuality::DISABLED;
    s.verticalSync = true;

    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    //On, unlike the other tests: this draws the upward multibeam's 129 beams in the 3D view,
    //so the swath fanning out against the ice is visible from the free camera. It is the
    //quickest way to see that the upward heads really are pointing up.
    h.showSensors = true;
    h.showActuators = false;
    h.showForces = false;

    UnderIceTestManager* simulationManager = new UnderIceTestManager(200.0);
    UnderIceTestApp app(std::string(DATA_DIR_PATH), s, h, simulationManager);
    app.Run();

    return 0;
}
