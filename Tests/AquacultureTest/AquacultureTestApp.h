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
//  AquacultureTestApp.h
//  Stonefish
//
//  Flies the BlueROV2 Heavy around a salmon pen. Subclasses BlueROV2TestApp for the
//  piloting, thrust allocation and autopilot, and adds NET HOLD: a bow multibeam fits the
//  net surface, one loop holds a standoff off it, and another keeps the vehicle square to it.
//
//  NET HOLD IS A HOLD, NOT A SURVEY. It takes the two axes that are tedious to fly - range
//  and alignment - and leaves the pilot flying ALONG the net on sway and depth, which is what
//  an inspection actually consists of. The standoff is a setpoint on W/S in exactly the way
//  the inherited altitude hold puts a standoff on Q/E, so the distance can be opened up to
//  take in a whole panel and closed down to look at one mesh. N releases the whole thing and
//  hands every axis straight back, so the site can also be roamed freely.
//

#ifndef __Stonefish__AquacultureTestApp__
#define __Stonefish__AquacultureTestApp__

#include "../BlueROV2Test/BlueROV2TestApp.h"

#include <vector>

namespace sf
{
    class Multibeam;
    class SimulationManager;
}

class AquacultureTestApp : public BlueROV2TestApp
{
public:
    AquacultureTestApp(std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h,
                       sf::SimulationManager* sim);

    void DoHUD() override;
    void ProcessInputs() override;
    //! Adds N on top of the inherited bindings, and re-purposes W/S and A/D while engaged.
    void KeyDown(SDL_Event* event) override;

protected:
    //! Vehicle telemetry, then the net hold panel.
    void DoTelemetry() override;
    //! Runs the net hold loops, between the inherited holds and thrust allocation.
    void AugmentDemands(sf::Scalar dt) override;

private:
    //! What the mode is doing.
    //!
    //! ARMED exists so that the estimator and the whole HUD run - and can be watched, and
    //! trusted - before anything is allowed to write a demand. It is also the free-roaming
    //! state: every axis behaves exactly as it does in BlueROV2Test.
    enum class NetHold { ARMED, ON };

    //! What the net profiler can say about the net right now, in the order of how bad it is.
    enum class NetState
    {
        NO_DATA,     //!< the profiler has not produced a sample yet
        LOST,        //!< too few returns to fit anything
        TOO_CLOSE,   //!< inside the safe standoff; back off before doing anything else
        CONE,        //!< below the cylindrical wall, where a plane fit means nothing
        PARTIAL,     //!< a fit, but with a run of missing beams in it - a hole
        GOOD
    };

    //! One frame of the net profiler, reduced to the geometry the loops need.
    struct NetFix
    {
        bool valid;
        sf::Scalar distance;      //!< perpendicular, from the VEHICLE origin, m
        sf::Scalar bearing;       //!< of the net normal in the body frame, rad, +ve to stbd
        sf::Scalar chord;         //!< how much of the net the fan spans, m
        unsigned int good;        //!< beams with a return
        unsigned int missing;     //!< beams reporting no return
        bool straddles;           //!< returns reach both sides of the boresight
        bool hole;                //!< a run of missing beams flanked by good ones
        sf::Scalar holeBearing;   //!< rad, body frame
    };

    //! Resolves the net profiler. It does not exist at construction time - the scenario is
    //! built during Init() - so this runs once on the first frame.
    void ResolvePayload();
    //! Reads the profiler and fits the net.
    NetFix ReadNet() const;
    //! Engages or releases the mode, seeding the setpoints so nothing kicks.
    void SetNetHold(NetHold m);
    //! Moves the standoff and look-angle setpoints from the keys the mode borrows.
    void UpdateNetSetpoints(sf::Scalar dt);

    bool payloadResolved_;
    //! The mode has engaged itself once, on the first good fix. See AugmentDemands().
    bool autoEngaged_;
    sf::Multibeam* profiler_;

    NetHold netHold_;
    NetState netState_;
    NetFix fix_;

    sf::Scalar standoffSp_;     //!< m, moved by W/S while engaged
    sf::Scalar lookSp_;         //!< rad, how far off the net normal the bow is held, on A/D
    //! How fast the standoff setpoint is being slewed right now, fed to the loop's D term so
    //! that it measures the ERROR rate rather than the vehicle rate - the same reason the
    //! inherited holds carry depthSpRate_ and friends.
    sf::Scalar standoffSpRate_;
    sf::Scalar lostFor_;        //!< seconds the fit has been unusable

    brov2::Derivative distRate_;
    brov2::PIDHold standoffPid_;

    //! One frame of the fan, as a profile, and a rolling trace of the standoff.
    std::vector<std::vector<GLfloat> > profilePlot_;
    std::vector<std::vector<GLfloat> > standoffPlot_;
};

#endif
