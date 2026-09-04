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
//  UnderIceTestManager.h
//  Stonefish
//
//  Loads the Arctic under-ice scenario from under_ice.scn and bolts the under-ice sensor
//  payload onto the BlueROV2 Heavy that scenario includes.
//

#ifndef __Stonefish__UnderIceTestManager__
#define __Stonefish__UnderIceTestManager__

#include <core/SimulationManager.h>

namespace sf
{
    class Robot;
}

//! Names and mounting geometry of the under-ice payload, shared with UnderIceTestApp.
namespace underice
{
    constexpr const char* ROBOT = "BLUEROV2";
    constexpr const char* LINK = "Vehicle";

    constexpr const char* ALTIMETER = "ice_altimeter";
    constexpr const char* MULTIBEAM = "ice_multibeam";
    constexpr const char* FLS = "ice_fls";
    constexpr const char* LIGHT = "ice_light";

    //! Height of the upward-looking heads above the vehicle origin, in the NED body frame,
    //! so negative is up. The physics hull spans z = -0.052 .. 0.184, so this sits the
    //! transducers just INSIDE the top of the frame, flush with the top plate.
    //!
    //! They were on a 70 mm bracket above the frame at first, which put them 50 mm INSIDE the
    //! ice whenever the vehicle rose and pinned itself against the canopy. Rays cast from
    //! inside a height field exit without hitting anything, so the sounder reported open water
    //! while the vehicle was physically touching the ice. Sensors must not stick out past the
    //! collision hull.
    constexpr double MAST_Z = -0.04;

    //! Height of the up-looking light, same frame, also negative-is-up. It cannot share the
    //! sounders' mount: the visual hull reaches z = -0.066 on the centreline, so anything at
    //! MAST_Z is inside the vehicle's own mesh. 34 mm clear of the top of the hull.
    constexpr double LIGHT_Z = -0.10;

    //! Largest range the upward heads report. Anything at this value is a NO RETURN, which
    //! under a canopy means open water overhead - the lead.
    constexpr double UP_RANGE_MAX = 60.0;

    //! Half-width of the upward multibeam swath, in degrees.
    constexpr double SWATH_HALF_FOV = 60.0;
}

class UnderIceTestManager : public sf::SimulationManager
{
public:
    UnderIceTestManager(sf::Scalar stepsPerSecond);

    void BuildScenario();

private:
    //! Attaches the under-ice payload to an already-parsed robot.
    void AddIcePayload(sf::Robot* rov);
};

#endif
