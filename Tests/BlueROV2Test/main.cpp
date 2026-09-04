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
//  BlueROV2Test
//

#include <cstdio>
#include "BlueROV2TestApp.h"
#include "BlueROV2TestManager.h"

int main(int argc, const char * argv[])
{
    //Unbuffered stdout: a cCritical() calls abort(), and buffered output is lost on the way out,
    //which makes startup failures look like a silent crash with no message at all.
    setvbuf(stdout, nullptr, _IONBF, 0);
    sf::RenderSettings s;
    s.windowW = 1200;
    s.windowH = 900;
    s.aa = sf::RenderQuality::MEDIUM;
    s.shadows = sf::RenderQuality::MEDIUM;
    s.ao = sf::RenderQuality::MEDIUM;
    s.atmosphere = sf::RenderQuality::MEDIUM;
    s.ocean = sf::RenderQuality::MEDIUM;
    s.ssr = sf::RenderQuality::LOW;
    s.verticalSync = true;

    sf::HelperSettings h;
    h.showFluidDynamics = false;
    h.showCoordSys = false;
    h.showBulletDebugInfo = false;
    h.showSensors = false;
    h.showActuators = false;
    h.showForces = false;

    BlueROV2TestManager* simulationManager = new BlueROV2TestManager(200.0);
    BlueROV2TestApp app("BlueROV2Test", std::string(DATA_DIR_PATH), s, h, simulationManager);
    app.Run();

    return 0;
}
