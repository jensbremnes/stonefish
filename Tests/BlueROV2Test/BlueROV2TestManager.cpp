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
//  BlueROV2TestManager.cpp
//  Stonefish
//

#include "BlueROV2TestManager.h"

#include <core/ScenarioParser.h>
#include <core/Console.h>
#include <utils/SystemUtil.hpp>

BlueROV2TestManager::BlueROV2TestManager(sf::Scalar stepsPerSecond)
    : SimulationManager(stepsPerSecond, sf::Solver::SI, sf::CollisionFilter::EXCLUSIVE)
{
}

void BlueROV2TestManager::BuildScenario()
{
    sf::ScenarioParser parser(this);
    bool success = parser.Parse(sf::GetDataPath() + "bluerov2_test.scn");
    if(success)
        cInfo("BlueROV2 scenario description parsed successfully.");
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
}
