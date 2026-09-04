#!/usr/bin/env python3
"""
Arctic sea ice asset generator for Tests/Data/under_ice.scn.

Writes, into Tests/Data/ice/:

    ice_draft_south_16b.png   16-bit grayscale ice-draft heightmap, southern floe
    ice_draft_north_16b.png   16-bit grayscale ice-draft heightmap, northern floe
    ice_albedo.png            tileable RGB albedo for the ice underside
    ice_normal.png            tileable tangent-space normal map for the ice underside

Deterministic: a fixed seed, so re-running reproduces the committed assets byte for byte.
The generated PNGs ARE committed - nothing in the build needs Python.

================================ THE ENCODING ================================
Stonefish reads a heightmap through stb_image, forces one channel, and INVERTS it
(Library/src/entities/statics/Terrain.cpp:36-84):

    heightfield = (1 - pixel/65535) * height          [16-bit branch]
    maxHeight   = max(heightfield) over the image     [NOT the XML 'height']

and Terrain::AddToSimulation composes the recentring offset on the RIGHT of the body
transform, so it is applied in the ROTATED local frame:

    motionState(origin * Transform(IQ(), Vector3(0, 0, -maxHeight/2)))

For an ice CEILING the terrain must be flipped 180 deg about X (see under_ice.scn for
why: back-face culling). Working the composition through for rpy="pi 0 0":

    world_z = origin.z + maxHeight - heightfield
            = origin.z + (pixel/65535) * height       [given maxHeight == height]

so BLACK = shallowest, WHITE = deepest keel - the opposite of the seabed convention.
maxHeight == height only holds if a pure-black pixel exists, so each floe is encoded
against ITS OWN draft extrema rather than a shared pair. That guarantees both a 0 and a
65535 pixel, and spends the full 16 bits on the range that floe actually covers:

    height   = d_max - d_min      per floe
    origin.z = d_min              per floe

which is what the tail of the run prints, one block per floe - the two floes do NOT share
these numbers. 16-bit is not optional: 8 bit over a 15 m range quantises to 6 cm and
terraces visibly in the upward-looking sonar.

============================== THE ICE MODEL ==============================
Level first-year ice, late winter, Arctic basin. Draft is by isostasy:

    d = (rho_i * h_i + rho_s * h_s) / rho_w
      = (917 * 1.60 + 330 * 0.20) / 1027 = 1.49 m

for 1.60 m of columnar first-year ice under 0.20 m of snow, in seawater of S = 34.5 at
its freezing point (-1.8 C, rho = 1027). Freeboard is 1.80 - 1.49 = 0.31 m and is NOT
modelled: geometry above z = 0 is exponentially over-brightened by the underwater
material shader (Library/shaders/materialU.frag:63-89), so only the submerged part of
the ice exists in this scene.

Superimposed on that base:

  - Undeformed thickness variability, band-limited noise at 20-80 m and 2-10 m scales.
    First-year ice thickness has a roughly 15% coefficient of variation over floe scales.
  - Refrozen leads (nilas), 8-15 m wide strips of 0.40 m ice -> 0.35 m draft.
  - Rafted ice, patches at doubled thickness -> ~3.0 m draft, with step edges.
  - Pressure ridge keels. Draft D = KEEL_MIN + Exp(KEEL_SCALE) clipped to KEEL_MAX, so
    shallow ridges are common and deep ones rare, as observed. Triangular cross-section
    with keel slope theta in [26, 36] deg, so the half-width is D/tan(theta): a 5 m keel
    comes out 17 m wide at the base and a 12 m keel 40 m, which is the right order for
    the Arctic. Ridge count and width together decide how much of the site is deformed -
    see the draft distribution the run prints, and aim for roughly 55-65 % of the area
    still at level-ice draft.
    The draw is conditioned on at least one keel reaching KEEL_MAJOR, so the site always
    has one ridge worth avoiding.
    Draft is modulated along the ridge axis by D(s) = D * (0.55 + 0.45 * noise(s)), so
    keels vary in depth and occasionally nearly pinch out, as real ones do. Blocky
    +/-0.4 m noise on the flanks stands in for the ice blocks a keel is built from.
    Keels combine with the base field by max(), not by addition.
  - Underside micro-roughness, +/-0.05 m at 0.5-3 m scales. Anything finer than the grid
    is carried by the normal map instead.
  - A level-ice taper over the last LEAD_MARGIN metres beside the open lead, so the floe
    edges the scenario has to cap are all the same known depth.

RIDGE COUNT IS A SCENARIO CHOICE, NOT A STATISTIC. The Arctic average is 5-10 ridges per
kilometre with keels deeper than 5 m, i.e. one or two in a 200 m box. This generator puts
in RIDGE_COUNT (10) so that a short transect actually meets something. Set it to 2 for a
statistically honest site.

The sail above the keel is not modelled (it is above z = 0). Overhangs and rafted
undercuts cannot be modelled at all: a height field is single-valued in z. That is a fair
trade - ice draft genuinely is a single-valued function of position, and no ROV sensor in
this scene resolves a re-entrant surface.
"""

import argparse
import os

import numpy as np
from PIL import Image
from scipy.ndimage import distance_transform_edt, gaussian_filter, map_coordinates

# ----------------------------------------------------------------------------- geometry
SITE_HALF = 100.0     # site is 200 x 200 m, centred on the origin
LEAD_HALF = 6.0       # open lead is |y| < 6 m, i.e. 12 m wide, running along x
LEAD_MARGIN = 4.0     # m of floe next to the lead tapered back to level ice (see below)

# ------------------------------------------------------------------------------ physics
RHO_WATER = 1027.0    # kg/m3, S = 34.5 at the freezing point
RHO_ICE = 917.0       # kg/m3, columnar first-year ice
RHO_SNOW = 330.0      # kg/m3, wind-packed late-winter snow
H_ICE = 1.60          # m, level first-year ice thickness
H_SNOW = 0.20         # m, snow cover

LEVEL_DRAFT = (RHO_ICE * H_ICE + RHO_SNOW * H_SNOW) / RHO_WATER   # 1.49 m

D_MIN = 0.30          # m, model floor: thinner than nilas is not modelled
D_MAX = 16.00         # m, model ceiling: deeper than this is clipped

NILAS_DRAFT = 0.35    # m, 0.40 m of nilas
NILAS_COUNT = 4
RAFT_DRAFT = 3.00     # m, doubled thickness
RIDGE_COUNT = 5       # see the note above: a scenario choice, not a statistic
KEEL_MIN = 3.0        # m, smallest feature still called a ridge here
KEEL_SCALE = 3.0      # m, mean of the exponential tail above KEEL_MIN
KEEL_MAX = 14.0       # m
KEEL_MAJOR = 10.0     # m, the site must contain at least one keel this deep (see below)
KEEL_SLOPE = (28.0, 38.0)   # deg; half-width is D/tan(theta), so 1.3-1.9 times the draft

SEED = 20260904

# The draft field is always built on this grid and then resampled to whatever the output asks
# for, so --resolution changes only the fidelity of the sampling, never the site. Building
# directly at the output resolution would redraw every noise field at a different array shape
# and hand back a different Arctic each time the grid changed - which quietly invalidates
# everything under_ice.scn says about where the keels are.
REF_RES = 0.25        # m per sample of the reference field

# Row order of the written PNG.
#
# MEASURED, NOT DERIVED. Reading Terrain.cpp, BuildTerrain and btHeightfieldTerrainShape says
# image row 0 sits at local y = -(rows-1)*scaleY/2, which the 180 deg flip about X should then
# mirror to the HIGH-y edge of the floe. It does not. Raycasting straight up at 35 known points
# and fitting the four candidate row/column orders against the heightmap gives an rms of 3 mm
# for "row 0 at the LOW-y edge, no mirror in x or y" and over a metre for every other
# combination - so the image goes to the world unmirrored, and the array here is written in the
# order it was built. If a future Stonefish changes this, redo that fit rather than reasoning
# about it; the verification section of the README says how.
FLIP_ROWS = False


def band_noise(rng, shape, sigma_px, mode="reflect"):
    """Zero-mean unit-variance band-limited noise. mode='wrap' makes it tile.

    The mean must be removed explicitly. At the coarse end sigma is a good fraction of the
    domain, so the filtered field is barely more than one low-frequency realisation and its
    mean over the finite window is a random offset of the same order as its standard
    deviation - which showed up as level ice sitting 0.33 m deeper than isostasy says.
    """
    n = gaussian_filter(rng.standard_normal(shape), sigma_px, mode=mode)
    n = n - n.mean()
    s = n.std()
    return n / s if s > 1e-12 else n


def smoothstep(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def build_draft_field(debug_marker):
    """Ice draft in metres over the whole site on the REF_RES grid, as d[row=y, col=x]."""
    res = REF_RES
    n = int(round(2.0 * SITE_HALF / res)) + 1
    x = np.linspace(-SITE_HALF, SITE_HALF, n)
    y = np.linspace(-SITE_HALF, SITE_HALF, n)
    X, Y = np.meshgrid(x, y)
    rng = np.random.default_rng(SEED)

    # 1. Level ice plus undeformed thickness variability.
    d = np.full((n, n), LEVEL_DRAFT)
    d += 0.25 * band_noise(rng, (n, n), 40.0 / res)   # 20-80 m scales
    d += 0.08 * band_noise(rng, (n, n), 5.0 / res)    # 2-10 m scales

    # 2. Refrozen leads. Blend rather than overwrite so the edges ramp over ~1 m.
    for _ in range(NILAS_COUNT):
        phi = rng.uniform(0.0, np.pi)
        x0, y0 = rng.uniform(-SITE_HALF, SITE_HALF, 2)
        half = rng.uniform(4.0, 7.5)
        t = -(X - x0) * np.sin(phi) + (Y - y0) * np.cos(phi)
        m = 1.0 - smoothstep(half - 1.0, half, np.abs(t))
        d = d * (1.0 - m) + NILAS_DRAFT * m

    # 3. Rafted patches: doubled thickness with step edges.
    raft = band_noise(rng, (n, n), 8.0 / res)
    raft_mask = raft > np.quantile(raft, 0.93)
    d = np.where(raft_mask, RAFT_DRAFT + 0.20 * band_noise(rng, (n, n), 2.0 / res), d)

    # 4. Pressure ridge keels, combined by max().
    #    The drafts are an exponential draw CONDITIONED on the site holding at least one
    #    major keel - without that condition five draws from a mean-3 m tail leave the
    #    deepest keel around 7 m more often than not, and the site has nothing worth
    #    avoiding. Conditioning preserves the shape of the distribution; it only discards
    #    the realisations that would not make a scenario.
    depths = None
    for _ in range(200):
        cand = np.clip(KEEL_MIN + rng.exponential(KEEL_SCALE, RIDGE_COUNT), KEEL_MIN, KEEL_MAX)
        if cand.max() >= KEEL_MAJOR:
            depths = cand
            break
    if depths is None:
        raise SystemExit("no ridge draw with a keel deeper than %.1f m in 200 tries" % KEEL_MAJOR)

    s_axis = np.arange(-2.0 * SITE_HALF, 2.0 * SITE_HALF + 1.0, 1.0)
    for depth in depths:
        theta = np.radians(rng.uniform(*KEEL_SLOPE))
        half = depth / np.tan(theta)
        phi = rng.uniform(0.0, np.pi)
        x0, y0 = rng.uniform(-SITE_HALF * 0.85, SITE_HALF * 0.85, 2)

        t = -(X - x0) * np.sin(phi) + (Y - y0) * np.cos(phi)   # across the ridge
        s = (X - x0) * np.cos(phi) + (Y - y0) * np.sin(phi)    # along the ridge

        prof = band_noise(rng, s_axis.shape, 12.0)
        prof = 0.55 + 0.45 * np.clip(0.5 + 0.5 * prof, 0.0, 1.0)
        depth_s = depth * np.interp(s, s_axis, prof)

        keel = depth_s * np.clip(1.0 - np.abs(t) / half, 0.0, 1.0)
        blocks = 0.45 * band_noise(rng, (n, n), 1.2 / res)
        keel = np.where(keel > 0.15, keel + blocks * (keel / depth), 0.0)
        d = np.maximum(d, keel)

    # 5. Lead margins are level ice. A lead opens along a crack, and a crack runs through
    #    level ice far more readily than through a ridge, so the last LEAD_MARGIN metres of
    #    each floe are tapered back to the level-ice draft. This is also what lets the
    #    scenario cap the floe edges with a single 1.5 m box: a height field ends at a
    #    knife edge with no thickness, and a cap only looks right if the ice it caps is a
    #    known, constant depth. Ridges still run up to within a few metres of the lead.
    margin = smoothstep(LEAD_HALF, LEAD_HALF + LEAD_MARGIN, np.abs(Y))
    d = LEVEL_DRAFT * (1.0 - margin) + d * margin

    # 6. Underside micro-roughness. Finer than the grid goes in the normal map instead.
    d += 0.05 * band_noise(rng, (n, n), 1.0 / res)

    if debug_marker:
        # An L in the world +x/+y quadrant corner: the long arm runs along +x. Used once
        # to settle the image-row -> world-y direction; see FLIP_ROWS.
        d[(X > 60.0) & (X < 95.0) & (Y > 85.0) & (Y < 92.0)] = D_MAX
        d[(X > 60.0) & (X < 67.0) & (Y > 60.0) & (Y < 92.0)] = D_MAX

    return np.clip(d, D_MIN, D_MAX)


def sample_floe(ref, y0, y1, res):
    """Resample the reference field onto a floe's own grid.

    The grid is snapped so that the floe spans exactly [y0, y1] and x spans exactly the site,
    whatever resolution was asked for - the returned scalex/scaley are the ones that produces,
    and they are what belongs in <dimensions>. Snapping matters: 94 m of floe does not divide
    by every grid, and letting the extent drift instead would move the floe edge away from the
    lead the scenario caps with a fixed box.
    """
    cols = int(round(2.0 * SITE_HALF / res)) + 1
    rows = int(round((y1 - y0) / res)) + 1
    scale_x = 2.0 * SITE_HALF / (cols - 1)
    scale_y = (y1 - y0) / (rows - 1)

    xs = np.linspace(-SITE_HALF, SITE_HALF, cols)
    ys = np.linspace(y0, y1, rows)
    # reference index = (world + SITE_HALF) / REF_RES, rows are y and columns are x
    ri = (ys[:, None] + SITE_HALF) / REF_RES + np.zeros((1, cols))
    ci = np.zeros((rows, 1)) + (xs[None, :] + SITE_HALF) / REF_RES
    return map_coordinates(ref, [ri, ci], order=1, mode="nearest"), scale_x, scale_y


def write_draft_png(path, d):
    """Encode against this floe's own extrema. Returns (origin_z, height) for the XML."""
    d_min = float(d.min())
    d_max = float(d.max())
    q = np.rint((d - d_min) / (d_max - d_min) * 65535.0).astype(np.uint16)
    assert q.min() == 0 and q.max() == 65535, "encoding must saturate both ends"
    if FLIP_ROWS:
        q = q[::-1, :]
    img = Image.fromarray(q)
    assert img.mode == "I;16", "heightmap must stay 16-bit, got mode %s" % img.mode
    img.save(path, optimize=True)
    return d_min, d_max - d_min


def cell_field(size, cells, rng):
    """Tileable Worley F1 distance and cell-boundary mask, for brine channels."""
    seeds = np.zeros((size, size), dtype=bool)
    pts = rng.integers(0, size, size=(cells, 2))
    seeds[pts[:, 0], pts[:, 1]] = True

    tiled = np.tile(seeds, (3, 3))
    dist, (iy, ix) = distance_transform_edt(~tiled, return_indices=True)
    lo, hi = size, 2 * size
    dist = dist[lo:hi, lo:hi]
    label = iy[lo:hi, lo:hi].astype(np.int64) * (3 * size) + ix[lo:hi, lo:hi]

    edge = np.zeros_like(dist, dtype=bool)
    for ax in (0, 1):
        edge |= label != np.roll(label, 1, axis=ax)
        edge |= label != np.roll(label, -1, axis=ax)

    return dist / max(dist.max(), 1e-6), edge


def write_textures(out_dir, size=1024):
    rng = np.random.default_rng(SEED + 1)

    cells, edge = cell_field(size, 140, rng)
    veins = gaussian_filter(edge.astype(np.float64), 1.6, mode="wrap")
    veins /= max(veins.max(), 1e-6)

    coarse = band_noise(rng, (size, size), 40.0, mode="wrap")
    fine = band_noise(rng, (size, size), 4.0, mode="wrap")
    bubbles = np.clip(band_noise(rng, (size, size), 1.5, mode="wrap"), 1.6, None) - 1.6

    # Albedo: bluish white, darkened along grain boundaries, brightened by frazil specks.
    base = np.array([0.80, 0.88, 0.93])
    vein_tint = np.array([0.42, 0.56, 0.68])
    shade = np.clip(1.0 + 0.10 * coarse + 0.05 * fine, 0.75, 1.20)

    rgb = base[None, None, :] * shade[:, :, None]
    v = 0.75 * veins[:, :, None]
    rgb = rgb * (1.0 - v) + vein_tint[None, None, :] * v
    rgb = np.clip(rgb + 0.9 * bubbles[:, :, None], 0.0, 1.0)
    Image.fromarray((rgb * 255.0 + 0.5).astype(np.uint8), mode="RGB").save(
        os.path.join(out_dir, "ice_albedo.png"), optimize=True)

    # Normal map from the same structure: cells dish inward, boundaries stand proud.
    h = 0.55 * (1.0 - cells) + 0.30 * veins + 0.15 * fine + 0.8 * bubbles
    h = gaussian_filter(h, 0.8, mode="wrap")
    # OpenGL tangent space: +X right, +Y up, +Z out of the surface.
    dhdx = (np.roll(h, -1, axis=1) - np.roll(h, 1, axis=1)) * 0.5
    dhdy = (np.roll(h, -1, axis=0) - np.roll(h, 1, axis=0)) * 0.5
    strength = 14.0
    nx, ny, nz = -dhdx * strength, dhdy * strength, np.ones_like(h)
    inv = 1.0 / np.sqrt(nx * nx + ny * ny + nz * nz)
    nrm = np.stack([nx * inv, ny * inv, nz * inv], axis=-1) * 0.5 + 0.5
    Image.fromarray((nrm * 255.0 + 0.5).astype(np.uint8), mode="RGB").save(
        os.path.join(out_dir, "ice_normal.png"), optimize=True)


def main():
    ap = argparse.ArgumentParser(description="Generate the Arctic sea ice assets.")
    ap.add_argument("--resolution", type=float, default=0.75,
                    help="metres per heightmap pixel (default 0.75). The scene is drawn once "
                         "per render pass with no frustum culling, so this is the main "
                         "performance knob; the triangle count goes as its inverse square. "
                         "The site itself does not change with it - the field is built at "
                         "REF_RES and resampled - but the printed <dimensions> do, so paste "
                         "them again after changing it.")
    ap.add_argument("--debug-marker", action="store_true",
                    help="stamp an L-shaped deep notch in the world +x/+y corner, to settle "
                         "the image-row to world-y direction (see FLIP_ROWS)")
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "ice"))
    ap.add_argument("--no-textures", action="store_true")
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)
    res = args.resolution

    ref = build_draft_field(args.debug_marker)

    floes = []
    for name, y0, y1 in (("south", -SITE_HALF, -LEAD_HALF), ("north", LEAD_HALF, SITE_HALF)):
        sub, scale_x, scale_y = sample_floe(ref, y0, y1, res)
        path = os.path.join(out_dir, "ice_draft_%s_16b.png" % name)
        origin_z, height = write_draft_png(path, sub)
        floes.append((name, 0.5 * (y0 + y1), origin_z, height, scale_x, scale_y))
        print("%-30s %4d x %-4d px  y in [%+.1f, %+.1f]  centre y %+.2f  draft %.2f-%.2f m"
              % (os.path.basename(path), sub.shape[1], sub.shape[0], y0, y1,
                 0.5 * (y0 + y1), sub.min(), sub.max()))

    if not args.no_textures:
        write_textures(out_dir)
        print("%-30s 1024 x 1024 px, tileable" % "ice_albedo.png / ice_normal.png")

    print()
    print("Paste into under_ice.scn:")
    for name, cy, origin_z, height, scale_x, scale_y in floes:
        print("  %s floe" % name)
        print('    <dimensions scalex="%.6g" scaley="%.6g" height="%.6g"/>' % (scale_x, scale_y, height))
        print('    <world_transform xyz="0.0 %+.2f %.4f" rpy="3.14159265 0.0 0.0"/>'
              % (cy, origin_z))
    print()
    hist, edges = np.histogram(ref, bins=[0.3, 0.5, 1.0, 1.4, 1.7, 2.5, 4.0, 6.0, 9.0, 16.0])
    total = ref.size
    print("draft distribution over the whole site (%d reference samples at %.2f m):" % (total, REF_RES))
    for k in range(len(hist)):
        print("  %5.2f - %5.2f m  %6.2f %%" % (edges[k], edges[k + 1], 100.0 * hist[k] / total))
    print("  level-ice isostatic draft = %.3f m" % LEVEL_DRAFT)


if __name__ == "__main__":
    main()
