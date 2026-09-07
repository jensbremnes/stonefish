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
//  UnderIceTestApp.h
//  Stonefish
//
//  BlueROV2TestApp plus the under-ice instrumentation: ice clearance and draft on the HUD,
//  an across-track ice-draft swath, a draft trace along the track, screen displays for the
//  forward sonar, and a tilt on the bow camera so it can be aimed at the canopy.
//
//  Piloting, thrust allocation, the demand ramp, the autopilot and the forward camera inset
//  are all inherited unchanged, so Tab / W A S D / Q E / , . / arrows / Space / V / H / Z / X /
//  B behave exactly as they do in BlueROV2Test.
//
//  The standoff below the canopy is the inherited ALTITUDE mode, which this app simply starts
//  in: bluerov2_heavy.scn is included here with dvl_up="1", so the DVL looks at the ice and its
//  altitude channel is the clearance below it. The upward Profiler and Multibeam that remain
//  here measure ice DRAFT for the readouts and the plots, which is a different quantity and
//  not part of any loop.
//

#ifndef __Stonefish__UnderIceTestApp__
#define __Stonefish__UnderIceTestApp__

#include "../BlueROV2Test/BlueROV2TestApp.h"

#include <vector>

namespace sf
{
    class FLS;
    class Multibeam;
    class Profiler;
    class SimulationManager;
}

class UnderIceTestApp : public BlueROV2TestApp
{
public:
    UnderIceTestApp(std::string dataDirPath, sf::RenderSettings s, sf::HelperSettings h,
                    sf::SimulationManager* sim);

    void DoHUD() override;
    //! Resolves the payload on the first frame, then pilots as the base class does.
    void ProcessInputs() override;
    //! Adds C (camera tilt) and F (forward sonar) on top of the inherited bindings.
    void KeyDown(SDL_Event* event) override;

protected:
    //! Vehicle telemetry, then the ice panel.
    void DoTelemetry() override;

private:
    //! Resolves the payload sensors. They do not exist at construction time - the scenario is
    //! built during Init() - so this runs once on the first frame that needs them.
    void ResolvePayload();
    //! Pushes the FLS display state to the sensor.
    void ApplyFLSView();
    //! Re-aims the bow camera to the current tilt detent.
    void ApplyCameraTilt();
    //! What the upward heads can say about the canopy right now.
    enum class Overhead
    {
        NO_DATA,      //!< the sounder has not produced a sample yet
        NO_RETURN,    //!< nothing anywhere in the 120 degree fan
        TOO_CLOSE,    //!< the fan sees ice but the vertical beam does not - pinned against it
        ICE           //!< a clean vertical return, so clearance and draft are meaningful
    };

    //! Reads the upward sounder, and cross-checks it against the wide fan so that being
    //! pressed against a rough canopy is distinguishable from being under the open lead.
    Overhead ReadOverhead(sf::Scalar& clearance, sf::Scalar& draft) const;
    //! Ice draft under one beam, or a negative value if that beam has no return.
    sf::Scalar BeamDraft(const sf::Transform& frame, sf::Scalar range, double angle) const;
    //! True if any beam of the upward fan has a return.
    bool AnySwathReturn() const;

    bool payloadResolved_;
    sf::Profiler* altimeter_;
    sf::Multibeam* swath_;
    sf::FLS* fls_;

    bool showFLS_;
    unsigned int camTilt_;  //!< index into the bow camera's tilt detents

    //! Scratch for the across-track swath plot, kept between frames so the HUD does not
    //! reallocate it every time it draws.
    std::vector<std::vector<GLfloat> > swathPlot_;

    //! Rolling ice-draft trace along the track. Kept here rather than plotted straight off
    //! the sounder's own history, because the interesting quantity is draft - depth of the
    //! head minus range - and not the raw range the sensor stores.
    std::vector<std::vector<GLfloat> > draftPlot_;
};

#endif
