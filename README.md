![Stonefish logo](https://github.com/patrykcieslak/stonefish/blob/master/Library/shaders/logo_64.png)
# ***Stonefish***
### An advanced simulation tool developed for marine robotics.

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

**Run the examples.** The eleven example applications land in `build/Tests/`, together with the
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
