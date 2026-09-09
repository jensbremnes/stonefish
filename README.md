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

**There are two cameras.** The scenario's bow camera is the *ice* camera: it tilts, and it starts
aimed straight up at the canopy, which leaves nothing to fly on. So `UnderIceTest` adds a second,
fixed **forward-looking camera** in the lower of the bow's two enclosure tubes, and that is the one
the pilot view (`V`) shows. The ice camera gets its own smaller inset above it.

Everything in [BlueROV2 Heavy](#bluerov2-heavy-this-fork) still applies - `UnderIceTest` reuses that
app's piloting, thrust allocation, autopilot and HUD unchanged - plus:

| Key | Action |
| --- | --- |
| `C` | Tilt the **ice camera**: 0 / 45 / 90 degrees up. This is the scenario's bow camera; the real vehicle carries it on a tilt servo, and under ice it lives at the top of its travel. |
| `U` | Ice camera inset on / off |
| `F` | Forward sonar display on / off |
| `V` | Cycle the **forward camera** inset, as it does in the open-water demo. Under ice this is the second, fixed camera rather than the tiltable one. |
| `Z` | The inherited vertical hold, which here starts in **altitude** at a 2 m standoff below the canopy, flown off an upward-looking DVL. Without it the vehicle, which is +2.0 N buoyant, simply rises until it is touching the ice - at which point the clearance is a few centimetres, inside every upward sensor's blanking range, and the ice readouts go blank. Flying a standoff is what an under-ice survey actually does. `Q` / `E` move the standoff; the measured hold is +-0.02 m. |

The left column of the HUD adds ice clearance and draft, the across-track draft swath from the
upward multibeam, and the draft along the track. The right column carries the sonar, the autopilot
panel, the ice camera and the pilot view, top to bottom. `showSensors` is on, so the upward fan is
drawn in the 3D view - the quickest way to see that the upward heads really are pointing up.

#### The frame budget

Worth knowing before adding a fourth camera. `OpenGLPipeline` renders every *continuous* view each
frame plus **exactly one** non-continuous view: cameras and sonars queue up, and one comes off the
queue per frame. Every rendered sensor here is non-continuous, so **the sum of their rates is a hard
budget against the frame rate**, and this scene draws in about 67 ms - roughly 15 frames, so 15 view
updates, per second.

The fit was already over that budget before the forward camera existed: a 30 Hz bow camera and a
5 Hz sonar ask for 35 updates a second against 15 available, the queue never drains, and the insets
flicker. So the rates are set to fit - 8 Hz forward camera, 3 Hz ice camera, 3 Hz sonar - and
anything whose display is switched off has its view disabled outright, which is where the margin
comes from. A hidden camera used to render every frame for nobody. Turning the ice camera (`U`) or
the sonar (`F`) off hands its share back to the pilot view.

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

### A salmon farm in a fjord (this fork)

`AquacultureTest` puts the same vehicle on the job it actually does most often in Norway: inspecting
the net of a sea cage. A 400 x 400 m site on the Trøndelag coast, three circular pens on a shared
mooring grid over a 66 m basin, the fjord wall rising to the shore along the west edge, and a moored
feed barge. The vehicle starts 2.5 m off the middle pen with its bow on the net.

The scene is defined in `Tests/Data/fish_farm.scn`, which includes `bluerov2_heavy.scn` unchanged.
Every asset is generated by `Tests/Data/tools/make_fish_farm.py`, and the generated files are
committed - nothing in the build needs Python.

#### Net hold

`N` engages **NET HOLD**, and it is a *hold*, not a survey. It takes the two axes that are tedious
to fly and leaves the rest to the pilot:

| Axis | Owner while NET HOLD is on |
| --- | --- |
| Surge | the standoff loop - holds the setpoint distance off the net |
| Yaw | the bearing loop - keeps the vehicle square to the net |
| Sway | **yours** - this is how you fly along the net |
| Heave / depth | **yours**, through the inherited depth or altitude hold |

So the distance behaves exactly like the inherited altitude hold does over the seabed: a setpoint
you move while the loop flies it. `N` again hands every axis straight back, and the whole site can
be roamed freely.

| Key | Action while NET HOLD is on |
| --- | --- |
| `N` | engage / release |
| `W` `S` | move the standoff setpoint in / out, 0.30 m/s, 0.8-6.0 m |
| `[` `]` | trim the standoff setpoint by 0.25 m |
| `A` `D` | swing the bow off the net normal, ±45° - look along the net without leaving it |
| `,` `.` | **fly along the net** (sway, manual throughout) |
| `Q` `E` | depth setpoint, as in the other demos |

Everything else - `Z` `X` `B` `V` `Tab` `Space` `H` - behaves exactly as it does in
[BlueROV2 Heavy](#bluerov2-heavy-this-fork).

The mode engages itself on the first good fix, for the same reason `UnderIceTest` flies its ice
standoff from the first frame: the vehicle is +2.0 N buoyant and sits in a 0.30 m/s current, so left
alone it drifts to 6.5 m of standoff inside a minute, which is past the visibility of this water.

Measured over two minutes at a 2.40 m setpoint, no pilot input:

| | |
| --- | --- |
| standoff | 2.40-2.44 m, i.e. ±0.04 m, settling to ±0.02 |
| net bearing | ±1.0°, typically ±0.5° |
| depth | ±0.03 m |
| beams on net | 49 of 49 |

#### How the net is sensed

A bow-mounted `sf::Multibeam` - 49 beams over 90°, 10 Hz - fires a horizontal fan at the net, and a
total-least-squares line through the returns gives both the perpendicular distance and the bearing.
It is pure raycasting, so it costs nothing against the render budget, and it ranges against the
net's smooth *collision* shell rather than its twine, which is what a real echosounder does to a net
panel.

Four things in that estimator were not obvious:

- **Two passes.** One beam through the tear that hits something on the far side is a leverage point
  on a total-least-squares fit and drags the whole plane with it. The second pass refits only the
  beams that landed on the first plane.
- **The target is a cylinder.** The fitted line is a chord of a 19.1 m circle, so it sits
  `chord²/8R` farther away than the nearest point of the net - 6 cm at a 1.5 m standoff, 4 % of the
  setpoint. It is a bias, not noise, so it is removed rather than tuned out.
- **The fan width is set by the tear, not by the resolution.** A 60° fan spans 1.7 m at 1.5 m, so
  the 0.8 m tear takes out half of it and leaves two short groups at the edges - which fit a line
  badly, and badly in a way that then steers the vehicle. At 90° the tear is a quarter of the fan.
- **A one-sided fit is the dangerous one.** Returns all on one side fit a line perfectly and point
  it almost anywhere. Those get a third of the correction and a `1-SIDED` flag rather than being
  either trusted or ignored - freezing the heading instead leaves the vehicle flying along the net
  with no way back.

The bearing loop writes the *heading setpoint* rather than a yaw demand, so it reuses the tuned
heading hold. That setpoint is an **absolute heading, not a per-frame nudge**: writing
`headingSp = yaw + 30°/s · dt` looks like a slew and is not one, because it re-anchors to wherever
the vehicle has got to every frame, so the heading hold never sees more than a degree of error and -
thrust being quadratic - turns at about half a degree a second. A vehicle 60° off the net took
minutes to come back.

The standoff loop's D term *is* taken on the measurement, which the under-ice altitude loop
specifically must not do. That loop differentiated an altitude built from four slant beams crossing
rough ice; this is a fit of 49 beams against a smooth shell, and there is no alternative anyway -
the DVL only reports velocity on a bottom ping, and the bottom is 45 m below the pen.

#### The net model

There is **no transparency anywhere in Stonefish** - `LookType::TRANSPARENT` is declared and never
used, the underwater material shader multiplies rgb by `albedo.a` and then forces `a = 1.0`, and
albedo textures are loaded with three channels so the sampled alpha is always 1. A netting texture
on a solid shell would be an opaque wall. So the twine is real geometry, and the salmon are visible
through it.

Real salmon netting is 25 mm bar with 2.4 mm twine, which is about three million meshes on one pen.
The generator coarsens the pitch to 0.30 m and **widens the twine to preserve solidity**:

```
open fraction = (1 - d/λ)²        Sn = 1 - open fraction = 0.183 clean, 0.35 fouled
w(λ') = λ' · (1 - √(1 - Sn))      → 29 mm clean, 58 mm fouled at λ' = 0.30 m
```

Solidity is what decides how much light the net passes, and therefore how grey it looks at range -
which is what an inspection actually judges, and what makes the fouled top band read as fouled
rather than merely browner. A close-up shows meshes an order of magnitude too big, and that is the
deception.

Each strand is a **triangular prism, not a flat ribbon**. A ribbon lying in the net surface has a
radial normal, so flying *along* the net - which is most of an inspection - puts every strand
edge-on and the net turns transparent exactly when it is being looked at. A prism also sidesteps
`OpenGLContent::CheckAndRepairFaceVertexOrder`, which silently turns a back face built by re-winding
the same vertices into a coincident duplicate of its front face.

The pen is **four objects, not sixteen panels**: there is no mesh cache in `OpenGLContent`, and
`DrawObjects` walks the whole queue with no frustum culling, so instancing would save nothing and a
merged ring is smaller on disk. What is split is what has to be: a fouled band, a clean band, one
torn sector, and the cone. The two neighbouring pens carry a coarse 1.0 m lattice - a deliberate
level of detail, 70 m away and well past the visibility of this water.

**The tear** is a 0.80 × 0.65 m hole at 180° azimuth, 5.40-6.05 m deep. Its *collision shell*
carries the same hole as its twine, from one parameter in one function, so the vehicle can fly
through it into the pen and the profiler loses its returns over it - which the HUD reports as
`HOLE IN NET`. A partial loss like that is deliberately not treated as a failure: the beams either
side still fix the net plane.

**The net is frozen.** `make_fish_farm.py` bakes a steady deformation at the design current - the
bottom ring sits 1.72 m downstream at 0.30 m/s - and a static mesh cannot respond to `<current>` at
runtime. Change the current in the scenario and the net shape is wrong; `AquacultureTestManager`
warns at startup if the two disagree by more than 20 %.

#### The waterline

`under_ice.scn` states that nothing may sit above `z = 0` or the underwater material shader blows it
out white. **That is true of the first pass and not of the finished frame**, and this scene depends
on the difference: the collar, the handrail, the buoys and the feed barge all have real freeboard.

When the eye is above water, `OpenGLPipeline` draws every object in `UNDERWATER` mode, blends the
ocean surface over it weighted by the Fresnel term, and then *redraws everything in `FULL` mode*
with depth testing. The redraw lands on exactly equal logarithmic depth values and the global depth
function is `GL_LEQUAL`, so the correct above-water shading overwrites the blown-out pass;
submerged fragments are rejected because the surface already wrote depth in front of them. From an
underwater eye the ocean backsurface is opaque and always nearer than anything above the surface, so
freeboard is hidden rather than blown out - which is also what a diver sees.

Two rules survive. Nothing may be *tangent* to `z = 0`, so every structure is given a definite
crossing; and the vehicle's cameras must stay below the surface, because the pipeline chooses its
path from the eye point alone.

Waves are off, but not for the reason the ice scene gives. With waves the water level is a per-pixel
FFT displacement and a collar that is a *fixed* body cannot heave with it - the sea would visibly
saw through a rigid ring - and the wave path also runs a compute dispatch per view per frame.

#### The frame budget

The pipeline renders every continuous view each frame plus exactly **one** non-continuous view, so
what has to fit under the frame rate is the *sum* of the vision sensors' update rates, not their
count. This scene draws in about 85 ms, so the budget is roughly ten view updates a second, and the
bow camera takes four of them. There is deliberately no second camera: unlike under the ice, nothing
here is tilted away from where the vehicle is pointed.

Two findings paid for most of that frame time:

- **Shadows are disabled, and that is the single biggest knob.** `OpenGLPipeline` bakes a shadowmap
  for every *active* light before it draws anything, gated only on `RenderSettings::shadows`, and
  `OpenGLSpotLight` is the only light that actually overrides `BakeShadowmap`. The vehicle's two
  Lumens therefore cost two extra full passes over a 230k triangle scene every frame. Turning them
  off took the scene from 6 fps to 10.
- **Point lights are free, spot lights are not.** The pen's photoperiod LEDs are point lights for
  exactly that reason.

`showSensors` is off by default. Ticking **Sensors** in the DEBUG panel draws the profiler's fan
against the net, which is the quickest way to see what the hold is ranging on - but
`ManualTrajectory::Render()` emits a `SENSOR_CS`, so the same switch stamps a coordinate cross on
all sixty salmon.

#### The site

Bathymetry, the seabed albedo and every mesh come from
`python Tests/Data/tools/make_fish_farm.py`, which is deterministic and prints the `<dimensions>`
and `<world_transform>` lines to paste back into the scenario - paste them, do not hand-edit them.
`--resolution` is the main performance knob for the seabed and `--mesh-pitch` for the netting.

The seabed albedo is site-scale at `uv_scale="1.0"`, which is what carries the **organic deposition
footprint** under each pen: the anoxic black core, the white Beggiatoa mats and the enriched halo an
MOM-B survey photographs, offset downstream and elongated by the current. The price is that there is
no tiling normal map for the seabed, a look having only one `uv_scale` for both textures. The ice
scene made the opposite trade.

The seabed look being *textured* is load-bearing for more than appearance: `LoadTexture` sets
`stbi_set_flip_vertically_on_load(true)` and nothing ever resets it, so whether the heightmap loads
bottom-up depends on a texture having been loaded first. `<looks>` is parsed before `<static>`, so
it holds - but reducing that look to a flat colour would silently mirror the seabed in y.

The salmon are 60 animated bodies on `ManualTrajectory`, driven by an analytic model of the
polarised torus a pen school really swims in, rather than by canned keypoints - which also lets them
open up around the vehicle. They are non-colliding, and that is load-bearing rather than merely
cheap: `Multibeam` masks in `MASK_ANIMATED_COLLIDING`, so a colliding school between the vehicle and
the net would corrupt every range the profiler reports.

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
| `AquacultureTest` | The same ROV inspecting a Norwegian salmon farm - see-through netting, a torn panel, a net-hold mode, soft-body moorings, 60 salmon - see [above](#a-salmon-farm-in-a-fjord-this-fork) |
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

In `BlueROV2Test`, `UnderIceTest` and `AquacultureTest` the keyboard flies the **vehicle** instead of
the camera - see [BlueROV2 Heavy](#bluerov2-heavy-this-fork), [Under the ice](#under-the-ice-this-fork)
and [A salmon farm in a fjord](#a-salmon-farm-in-a-fjord-this-fork) above for the full control lists.


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
