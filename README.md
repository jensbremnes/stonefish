![Stonefish logo](https://github.com/patrykcieslak/stonefish/blob/master/Library/shaders/logo_64.png)
# ***Stonefish***
### An advanced simulation tool developed for marine robotics.

### BlueROV2 Heavy (this fork)

A complete **BlueROV2 Heavy** - the Blue Robotics observation-class ROV in its 8-thruster
configuration - with a forward-looking camera you can fly on. It is meant for practising manual
piloting on camera visuals, the way a real pilot flies off a topside display.

```console
$ ./build/Tests/BlueROV2Test.exe
```

#### Piloting

The keyboard flies the **vehicle**, not the scene camera. `Tab` hands the keys back to the scene
camera and back again; the active mode is shown on the HUD along with speed, depth and attitude.

| Key | Action |
| --- | --- |
| `W` / `S` | Surge - ahead / astern |
| `A` / `D` | Yaw - port / starboard |
| `,` / `.` | Sway - port / starboard |
| `Q` / `E` | Heave - up / down |
| `Up` / `Down` | Pitch trim |
| `Left` / `Right` | Roll trim |
| `Space` | All stop, and park every autopilot setpoint where the vehicle is |
| `Z` | Vertical hold: off / depth / altitude |
| `X` | Heading hold on / off |
| `B` | Attitude hold on / off (pitch and roll) |
| `V` | Cycle the onboard camera - bottom-right inset, full width, off |
| `Tab` | Swap the keys between the vehicle and the scene camera |
| `H` | Hide the HUD; piloting carries on |

Mouse control of the scene camera (drag to rotate, wheel to zoom) works in both modes, and the six
demand sliders on the HUD are a second way in.

**When a hold is engaged, its keys move the setpoint instead of commanding thrust** - see
[Autopilot](#autopilot) below.

#### Control feel

A held key **ramps** to its target over about half a second rather than stepping to it, and each
axis is capped below full thruster authority:

| Axis | Cap | Steady result |
| --- | --- | --- |
| Surge | 0.80 | 1.45 m/s |
| Sway | 0.60 | 0.90 m/s |
| Heave | 0.50 | 0.81 m/s |
| Yaw | 0.12 | 47 deg/s |
| Pitch / roll | 0.30 | trim only |

The caps are there because full authority is not flyable: at a yaw demand of 1.0 the model turns at
**380 deg/s**, more than a revolution per second. That is a property of the model rather than of the
vehicle - rotational damping is derived from the geometry and cannot be matched to an identified
`Nrr`, while every thruster makes its full datasheet bollard thrust with no interaction losses. The
surge cap is picked so that full ahead gives the BlueROV2's published ~1.5 m/s instead of the
model's unthrottled 1.83 m/s. All of these are named constants at the top of
`Tests/BlueROV2Test/BlueROV2TestApp.cpp`, and the sliders still reach 1.0 if you want the rest.

#### Autopilot

Depth, altitude, heading, pitch and roll all hold themselves, closed around the onboard pressure
sensor, IMU and DVL. All three modes are **on at startup** - the vehicle is +2.0 N buoyant with only
2.30 Nm/rad of restoring stiffness, so left open-loop it rises and wanders and the pilot spends the
whole time trimming.

The point is that **the motion keys become setpoint controls**. You do not fly the thrusters, you
fly the targets, and the vehicle gets itself there and stays:

| Mode | Key | Engaged, these move the setpoint | Rate |
| --- | --- | --- | --- |
| Vertical: depth | `Z` | `Q` / `E` - shallower / deeper | 0.3 m/s |
| Vertical: altitude | `Z` | `Q` / `E` - closer / further from the surface the DVL faces | 0.3 m/s, 0.5-15 m |
| Heading | `X` | `A` / `D` - port / starboard | 30 deg/s |
| Pitch | `B` | `Up` / `Down` | 15 deg/s, +-30 deg |
| Roll | `B` | `Left` / `Right` | 15 deg/s, +-30 deg |

Surge and sway stay fully manual in every mode. Turning a mode off hands that axis straight back.
`Space` stops the vehicle *and* parks every setpoint where it currently is, so it holds position
there rather than flying back to a target you set a minute ago.

Measured, against `bluerov2_test.scn`:

| | |
| --- | --- |
| Depth, tracking a 2 m move at the key rate | within 0.26 m, 0.15 m overshoot, settled in ~4 s |
| Depth, holding | +-0.02 m |
| Heading, 90 deg turn at the key rate | arrives with no overshoot, holds +-1.5 deg |
| Pitch, 15 deg held through full surge | +-1 deg |
| Ice standoff, 2 m under the canopy | +-0.02 m |

That pitch figure is worth singling out. The vehicle settles about **9 degrees nose-down at full
surge** open-loop, an unexplained trim documented at length in the header of `bluerov2_heavy.scn`.
With attitude hold on it sits at whatever you asked for instead.

Every gain is a named constant at the top of `Tests/BlueROV2Test/BlueROV2TestApp.cpp`, with the
measurement that set it written beside it. Three are worth knowing about because they are not
obvious:

- **The vertical integrator is seeded at 0.098, not 0.010.** Thrust is quadratic in the normalised
  setpoint, so offsetting +2.0 N of buoyancy across four 51.5 N units needs `sqrt(2.0/(4*51.5))`.
  Reading it as linear leaves the vehicle rising while the integral winds in.
- **The altitude loop takes its damping from the pressure sensor, not the DVL.** Differentiating a
  10 Hz altitude that steps as its four slant beams cross ice costs so much phase that the D term
  arrived 57 degrees late and drove a +-0.6 m porpoise instead of damping it. Damping should oppose
  the *vehicle's* vertical motion, which is what a pressure sensor measures directly - and it should
  not fight the slope of the terrain, which is the other half of what the DVL sees.
- **Roll runs at 0.55 of the pitch gains.** The vertical thrusters sit at `x = +-0.12` but
  `y = +-0.218`, so the same demand makes 1.8x the roll moment. One set of gains on both axes held
  pitch to +-1 deg while roll sat in a 2 Hz limit cycle.

If the DVL loses its return, altitude hold **degrades to depth hold** at the depth where the track
was lost, says so on the HUD, and picks the altitude back up by itself. That makes it safe to leave
engaged over an open lead.

#### Onboard camera

A forward-looking 1280x720 low-light camera sits in the bow at the height of the electronics
enclosure, with the two Lumen lights aimed forward and tilted 15 degrees down. `V` cycles the view
between a bottom-right inset, a full-width pilot view and off.

Worth knowing if you add your own: a vision sensor looks along the **+Z** axis of its own frame with
**-Y** as image up, an optical convention sitting inside an X-forward NED body frame. A
forward-facing camera therefore needs `rpy="${pi/2} 0.0 ${pi/2}"`, not `rpy="0 0 0"` - which aims it
straight down. `<light>` emits along its own +Z as well.

#### Reusing the vehicle

`Tests/Data/bluerov2_heavy.scn` holds the vehicle alone and takes arguments, so it can be dropped
into any scene:

```xml
<include file="bluerov2_heavy.scn">
    <arg name="robot_name" value="BLUEROV2"/>
    <arg name="robot_position" value="0.0 0.0 3.0"/>
    <arg name="dvl_up" value="0"/>
</include>
```

`dvl_up` picks which way the DVL looks: `0` down at the seabed, `1` up at an ice canopy. It decides
what the altitude channel means and therefore what altitude hold flies against. Do not set it to `1`
in a scene with nothing overhead - the ocean surface is not a collision body, so the beams find
nothing at all. All three arguments are required; the parser only substitutes `$(arg ...)` when the
include passes at least one, and there are no defaults.

`Tests/Data/bluerov2_test.scn` is the demo scene that includes it - a shallow-water site with a
seabed, two reference blocks and a neutrally buoyant marker to fly around. The header of
`bluerov2_heavy.scn` documents where every mass, drag and thruster coefficient came from, which
figures were measured and which limitations remain.

### Under the ice (this fork)

An Arctic under-ice site for the same vehicle: a 200 x 200 m box of first-year pack ice north of
Svalbard in 48 m of water, cut by one open lead.

```console
$ ./build/Tests/UnderIceTest.exe
```

The ice is a **ceiling, not scenery**. It is solid - the ROV cannot pass through it and will pin
itself against it if you let go of the controls - and it is visible to every sensor that looks at
it: an upward-looking sounder measures ice draft, an upward multibeam maps the draft across track,
the forward sonar picks keels out ahead, and the **DVL is turned over to look up**, so it reports
clearance below the ice and speed relative to it rather than over the ground. The vehicle spawns under level ice of 1.7 m draft, 6 m
from the lead edge, flying a 2 m standoff below the canopy.

Everything in [BlueROV2 Heavy](#bluerov2-heavy-this-fork) still applies - `UnderIceTest` reuses that
app's piloting, thrust allocation and HUD unchanged - plus:

| Key | Action |
| --- | --- |
| `C` | Tilt the bow camera: 0 / 45 / 90 degrees up. The real vehicle's camera is on a tilt servo, and under ice it lives at the top of its travel. |
| `F` | Forward sonar display on / off |
| `Z` | The inherited vertical hold, which here starts in **altitude** at a 2 m standoff below the canopy, flown off an upward-looking DVL. Without it the vehicle, which is +2.0 N buoyant, simply rises until it is touching the ice - at which point the clearance is a few centimetres, inside every upward sensor's blanking range, and the ice readouts go blank. Flying a standoff is what an under-ice survey actually does. `Q` / `E` move the standoff; the measured hold is +-0.02 m. |

The left column of the HUD adds ice clearance and draft, the across-track draft swath from the
upward multibeam, and the draft along the track. `showSensors` is on, so the upward fan is drawn in
the 3D view - the quickest way to see that the upward heads really are pointing up.

Running straight ahead from the spawn the transect is level ice, then a 5-7 m keel about 60 m out,
then level ice again, then a 10-13 m keel at 110-125 m. The deepest keel in the site is 14 m.

#### The ice model

`Tests/Data/tools/make_sea_ice.py` generates the assets; they are committed, so nothing in the
build needs Python. Ice draft is isostatic - 1.6 m of first-year ice under 0.2 m of snow gives
1.49 m - with band-limited thickness variability, refrozen nilas at 0.35 m, rafted patches at
3.0 m, and pressure ridge keels drawn from an exponential distribution with a triangular
cross-section and a 26-36 degree keel slope. The script's header documents every number and where
it came from, including which choices are scenario decisions rather than statistics.

Each floe is a `<static type="terrain">` **rotated 180 degrees about X** so it hangs as a ceiling.
That rotation is not optional: `BuildTerrain` winds its faces to face the sky, and with the
pipeline's back-face culling an unflipped sheet is invisible from below - to the pilot camera and to
every rendered sonar - while still colliding perfectly. A height field is also the physically right
representation, since ice draft genuinely is single-valued in position; the price is that overhangs
and rafted undercuts cannot be expressed at all.

The open lead is the one thing a single height field cannot do, so there are two floes with a gap
between them. `Tests/Data/under_ice.scn` has the rest: the heightmap encoding arithmetic, why the
water surface must be flat, why nothing may sit above `z = 0`, and why a material's `restitution`
is doing double duty as its acoustic reflectivity.

Worth knowing if you change the resolution: `make_sea_ice.py --resolution` only changes how finely
the site is sampled, not the site itself, but it does change the `<dimensions>` and
`<world_transform>` values the script prints - paste them, do not hand-edit them. The image-to-world
row order was **measured**, not derived: raycasting up at 35 known points and fitting the four
candidate row/column orders against the heightmap gives an rms of 3 mm for one of them and over a
metre for the rest. If a future Stonefish changes that, redo the fit rather than reasoning about it.

Frame rate is the thing to watch. `OpenGLPipeline` does no frustum culling, so the whole canopy is
submitted for every render pass, and this scene has five. If it drops far enough that the cameras
cannot be serviced every frame, the on-screen insets start to flicker. The knobs, in order of
effect, are `--resolution` on the generator, the `RenderSettings` in `Tests/UnderIceTest/main.cpp`,
and the number of vision sensors in the payload.

### Windows Support (this fork)

Upstream _Stonefish_ is Linux-only. This fork builds and runs natively on **Windows x64** using the
**MSYS2 / MinGW-w64 (UCRT64)** toolchain. Every change is guarded by `#ifdef _WIN32`, `if(WIN32)` or
`if(MSVC)`, so the Linux build is unaffected and upstream can still be merged in.

#### Building on Windows

1. Install [MSYS2](https://www.msys2.org), e.g. `winget install MSYS2.MSYS2`.
2. From the **UCRT64** shell (`C:\msys64\ucrt64.exe`, *not* the MSYS or MINGW64 shell) install the
   toolchain and dependencies:

```console
$ pacman -S --needed mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake \
                     mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-SDL2 \
                     mingw-w64-ucrt-x86_64-freetype mingw-w64-ucrt-x86_64-glm
```

3. Configure and build, still from the UCRT64 shell:

```console
$ mkdir build && cd build
$ cmake -G Ninja -DBUILD_TESTS=ON -DCMAKE_BUILD_TYPE=Release ..
$ ninja
```

Do not use a `cmake`, `g++` or `ninja` found on the ordinary Windows `PATH` (the ones bundled with
Strawberry Perl, for instance) - they do not provide SDL2, Freetype or GLM.

Bullet, GLAD, TinyXML-2, TinySpline, TinyExpr, stb and rapidobj are vendored in `3rdparty/`, so
SDL2, Freetype and GLM are the only external dependencies.

#### Getting started

**Run the examples.** The thirteen example applications land in `build/Tests/`, together with the
sixteen runtime DLLs they depend on (SDL2, Freetype and the MinGW / HarfBuzz / GLib chain), which
the build copies there automatically. So they start from any shell, or by double-clicking them in
Explorer - MSYS2 does not have to be on `PATH`, and the folder can be copied to a machine that has
no MSYS2 installation at all.

```console
$ ./build/Tests/UnderwaterTest.exe
```

| Example | What it demonstrates |
| --- | --- |
| `ConsoleTest` | Headless physics, no window - scenario loaded from an XML file |
| `FallingTest` | Rigid bodies, collisions, materials |
| `FloatingTest` | Surface vessel, buoyancy, a thruster - the shortest example to read |
| `UnderwaterTest` | Full ocean, an AUV, sonars and cameras - the heaviest |
| `BlueROV2Test` | BlueROV2 Heavy ROV, 8 thrusters, keyboard-flown on its onboard camera - see [above](#bluerov2-heavy-this-fork) |
| `UnderIceTest` | The same ROV under Arctic sea ice - solid canopy, upward ice-draft sonar, forward sonar - see [above](#under-the-ice-this-fork) |
| `FlyingTest`, `SlidingTest`, `JointsTest`, `CableTest` | Aerodynamics, friction, joints, cables |
| `FluidDynamicsTest`, `CameraTest`, `LearningTest` | Hydrodynamics, vision sensors, ML interfacing |

**Controls.** Press `K` inside any graphical example to show the full keymap.

| Input | Action |
| --- | --- |
| `W` `S` `A` `D` | Move the camera forward / back / left / right |
| `Q` `Z` | Move the camera up / down |
| Mouse drag | Rotate the camera (trackball); wheel zooms |
| `H` / `K` / `P` / `C` | Toggle the HUD / keymap / performance monitor / console |
| `Esc` | Quit |

In `BlueROV2Test` and `UnderIceTest` the keyboard flies the **vehicle** instead of the camera - see
[BlueROV2 Heavy](#bluerov2-heavy-this-fork) and [Under the ice](#under-the-ice-this-fork) above for the full control lists.


**Write your own simulation.** Subclass `sf::SimulationManager`, implement `BuildScenario()`, and
hand it to a `sf::GraphicalSimulationApp`:

```cpp
// MyManager.h
#include <core/SimulationManager.h>

class MyManager : public sf::SimulationManager
{
public:
    MyManager(sf::Scalar stepsPerSecond) : sf::SimulationManager(stepsPerSecond) {}
    void BuildScenario() override;
};
```

```cpp
// main.cpp
#include <core/GraphicalSimulationApp.h>
#include "MyManager.h"

int main(int argc, const char* argv[])
{
    sf::RenderSettings s;   // windowW, windowH, aa, shadows, ao, atmosphere, ocean, ssr, verticalSync
    s.windowW = 1200;
    s.windowH = 900;

    sf::HelperSettings h;   // showCoordSys, showForces, showSensors, showActuators, ...

    MyManager* manager = new MyManager(500.0);   // physics steps per second
    sf::GraphicalSimulationApp app("MyApp", "path/to/my/data/", s, h, manager);
    app.Run();
    return 0;
}
```

`BuildScenario()` is where the world is defined - `CreateMaterial()`, `CreateLook()`,
`EnableOcean()`, then entities, robots, sensors and actuators.
`Tests/FloatingTest/FloatingTestManager.cpp` is a short, complete one to copy from. For a headless
run use `sf::ConsoleSimulationApp app("MyApp", "path/to/my/data/", manager);` instead.

**Or describe the scene in XML.** Instead of building the scenario in C++, parse a `.scn` file:

```cpp
void MyManager::BuildScenario()
{
    sf::ScenarioParser parser(this);
    parser.Parse(sf::GetDataPath() + "my_scenario.scn");
}
```

`Tests/Data/console_test.scn` is a worked example; the
[scenario file documentation](https://stonefish.readthedocs.io/en/latest/scenario.html) has the full
format.

**Compiling your own application.** The simplest route today is to drop your app next to the
examples in `Tests/CMakeLists.txt` and link `Stonefish_test`:

```cmake
add_executable(MyApp MyApp/main.cpp MyApp/MyManager.cpp)
target_link_libraries(MyApp Stonefish_test)
```

Linking against a system-wide installed copy via `find_package(Stonefish)` is not yet verified on
Windows - see *Status* below.

#### What was changed for Windows

| File | Change |
| --- | --- |
| `Library/include/utils/SystemUtil.hpp` | Removed `<windows.h>` from this widely included header. It leaked the `ERROR`, `TRANSPARENT`, `near` and `far` macros into `MessageType::ERROR`, `LookType::TRANSPARENT` and the camera clip-plane members, breaking all 21 translation units that include it. Also removed three dead, Windows-broken functions: `GetDataPathPrefix` (referenced an undefined `CEGUI_SAMPLE_DATAPATH`), `CheckForExtension` (called `glewIsSupported`, but GLEW was replaced by GLAD) and `GetCWD`. |
| `Library/src/utils/SystemUtil.cpp` | **New.** Holds the platform-specific `GetPhysicalCores()` implementations, so that no Stonefish header pulls in a platform header. |
| `Library/src/utils/GeometryFileUtil.cpp` | Guarded `#undef` of the Win32 macros after `rapidobj.hpp` (which includes `<windows.h>`). Including it last is not sufficient - the `cError()` macro *expands* below that include. |
| `Library/src/graphics/OpenGLCamera.cpp` | Replaced a variable-length array with `std::vector`. VLAs are a GCC extension and are rejected by MSVC. |
| `Library/src/core/GraphicalSimulationApp.cpp` | Call `SDL_SetMainReady()` before `SDL_Init()`, required because `SDL_MAIN_HANDLED` is defined. |
| `Tests/CMakeLists.txt`, `Tests/BundleRuntimeDLLs.cmake` | **New script.** Post-build step that copies the runtime DLLs the executables need next to them, resolved with `file(GET_RUNTIME_DEPENDENCIES)`. Without it the applications fail to start with *"libfreetype-6.dll was not found"* unless MSYS2 is on `PATH`. |
| `CMakeLists.txt` | GCC-only flags moved behind `if(MSVC)`; `_USE_MATH_DEFINES` and `SDL_MAIN_HANDLED` defined on Windows; a static library is built on Windows, because a DLL exports no symbols without `__declspec(dllexport)` annotations; `SDL2::SDL2main` removed from the link line, as the applications provide their own `main()`. |

#### Status

All 11 test applications in `Tests/` build and run. Measured on an Intel Core Ultra 5 235U with
integrated Arc graphics (OpenGL 4.6): roughly 68 FPS on `FallingTest` and 30 FPS on `UnderwaterTest`
at `RenderQuality::HIGH`, with physics running on 12 threads.

Not yet verified on Windows: the system-wide install path (`-DBUILD_TESTS=OFF` plus
`cmake --install`), whose rules are Linux-shaped. An MSVC + vcpkg build has not been set up either,
but the changes above are written to be toolchain-neutral.

Stonefish is a C++ library combining a physics engine and a lightweight rendering pipeline. The physics engine is based on the core functionality of the [Bullet Physics](https://pybullet.org) library, extended to deliver realistic simulation of marine robots. It is directed towards researchers in the field of marine robotics but can as well be used as a general purpose robot simulator. 

Stonefish includes advanced hydrodynamic computations based on actual geometry of bodies, to better approximate hydrodynamic forces and allow for effects not possible when using symbolic models. The rendering pipeline, developed from the ground up, delivers realistic rendering of atmosphere, ocean and underwater environment. Special focus was put on the latter, where effects of wavelength-dependent light absorption and scattering were considered (other simulators often use only blue fog). 

Stonefish can be used to create standalone applications or combined with a [Robot Operating System](https://www.ros.org) (ROS) package [_stonefish_ros_](https://github.com/patrykcieslak/stonefish_ros), which implements 
standard simulator node and facilitates easy integration with ROS architecture.

There are two sources of documentation for the library: [html documentation generated with Sphinx](https://stonefish.readthedocs.io) and code documentation generated with Doxygen, based on comments in the code (instructions below).

### Requirements

The simulation is CPU heavy and requires a recent GPU. The minimum requirement is the support for *OpenGL 4.3*. 

Install official manufacturer drivers for your graphics card before using _Stonefish_!

The software is developed and tested on *Linux Ubuntu*. It should work on any Unix based platform. This fork also builds and runs on Windows - see [Windows Support](#windows-support-this-fork) above. MacOS is not supported due to its lack of support for OpenGL 4.3.

### Installation
1. Dependencies
    * **OpenGL Mathematics library** (libglm-dev, version >= 0.9.9.0)
    * **SDL2 library** (libsdl2-dev)
    * **Freetype library** (libfreetype6-dev)

2. Building
    1. Clone _stonefish_ repository.
    2. `cd stonefish`
    3. `mkdir build`
    4. `cd build`
    5. `cmake ..`
    6. `make -jX` (where X is the number of threads)
    8. `sudo make install`

3. Documentation
    1. Go to "stonefish" directory.
    2. `doxygen doxygen`
    3. Open "docs/html/index.html".
    
### Credits
This software was written and is continuously developed by Patryk Cieślak. Parts of the software based on code developed by other authors are clearly marked as such.

If you find this software useful in your research, please cite:

*Patryk Cieślak, "Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface", In Proceedings of MTS/IEEE OCEANS 2019, June 2019, Marseille, France*
```
@inproceedings{stonefish,
   author = {Cie{\'s}lak, Patryk},
   booktitle = {Proceedings of MTS/IEEE OCEANS 2019},
   title = {{Stonefish: An Advanced Open-Source Simulation Tool Designed for Marine Robotics, With a ROS Interface}},
   month = jun,
   year = {2019},
   doi={10.1109/OCEANSE.2019.8867434}}
```

*Michele Grimaldi, Patryk Cieślak, Eduardo Ochoa, Vibhav Bharti, Hayat Rajani, Ignacio Carlucho, Maria Koskinopoulou, Yvan R. Petillot, and Nuno Gracias, "Stonefish: Supporting Machine Learning Research in Marine Robotics", In Proceedings of IEEE ICRA 2025, May 2025, Atlanta, USA*

```
@inproceedings{stonefish_ml,
   author = {Michele Grimaldi and Patryk Cieslak and Eduardo Ochoa and Vibhav Bharti and Hayat Rajani and Ignacio Carlucho and Maria Koskinopoulou and Yvan R. Petillot and Nuno Gracias},
   title = {Stonefish: Supporting Machine Learning Research in Marine Robotics},
   booktitle = {Proceedings of the IEEE International Conference on Robotics and Automation},
   month = may,
   year = {2025},
   eprint = {2502.11887},
   archivePrefix = {arXiv},
   url = {https://arxiv.org/abs/2502.11887},
   organization = {IEEE}}
```

### Funding
Currently there is no funding of this work. It is developed by the author following his needs and requests from other users. The work was started during his PhD studies and was mainly developed in his free time. Parts of this work were developed in the context of the project titled ”Force/position control system to enable compliant manipulation from a floating I-AUV”, which received funding from the European Community H2020 Programme, under the Marie Sklodowska-Curie grant agreement no. 750063. The work was also extended under a project titled ”EU Marine Robots”, which received funding from the European Community H2020 Programme, grant agreement no. 731103. 

### License
This is free software, published under the General Public License v3.0.
