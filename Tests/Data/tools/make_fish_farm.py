#!/usr/bin/env python3
"""
Norwegian salmon farm asset generator for Tests/Data/fish_farm.scn.

Writes, into Tests/Data/farm/:

    bathymetry_16b.png     16-bit grayscale fjord bathymetry for the whole site
    seabed_albedo.png      site-scale RGB albedo, including the deposition footprint

Deterministic: a fixed seed, so re-running reproduces the committed assets byte for byte.
The generated PNGs ARE committed - nothing in the build needs Python.

================================ THE SITE ================================
A 400 x 400 m box on the Trondelag coast, x and y in [-200, +200], NED with z DOWN and
the water surface at z = 0.

    x = -200 .. -192     shore shelf, 5 m               bedrock and kelp holdfasts
    x = -192 .. -116     the fjord wall, 5 -> 64 m      ~38 deg, bedrock and talus
    x = -116 .. +200     the basin, 64 -> 70 m          soft olive-grey mud

    pens at (0, -70), (0, 0), (0, +70)                  67 m of water under the hero pen
    vehicle spawn (-21.6, 0, 3)                         due south of the hero pen

The row of pens runs along y, which is the fjord axis and the direction of the tidal
current, so each pen sits in its neighbour's wake - which is the real reason the
deposition footprints below are elongated and offset along y rather than circular.

============================== THE ENCODING ==============================
Stonefish reads a heightmap through stb_image, forces one channel, and INVERTS it
(Library/src/entities/statics/Terrain.cpp:44-60):

    heightfield = (1 - px/65535) * height
    maxHeight   = max(heightfield) over the image        [NOT the XML 'height']

Terrain::AddToSimulation composes the recentring offset on the RIGHT of the body
transform (Terrain.cpp:104), and btHeightfieldTerrainShape centres a [0, maxHeight]
field about its own origin, so for an UNFLIPPED seabed the two -maxHeight/2 terms add:

    world_z = origin.z - maxHeight/2 + (heightfield - maxHeight/2)
            = origin.z + heightfield - maxHeight
            = origin.z - (px/65535) * height             [given maxHeight == height]

so BLACK = DEEPEST and WHITE = SHALLOWEST, and the terrain spans
[origin.z - height, origin.z]. Which is exactly what under_ice.scn's own seabed does:
height="5.0" at origin.z = 50 spans z = 45..50. Hence

    height   = d_max - d_min
    origin.z = d_max
    px       = (d_max - d) / (d_max - d_min) * 65535

maxHeight == height only holds if a pure-black pixel exists, so the field is encoded
against its own extrema, which guarantees one.

16-bit is not optional: 8 bit over a 65 m range quantises to 25 cm, and the DVL
altitude then steps visibly as the vehicle flies.

============================== ROW AND COLUMN ORDER ==============================
DERIVED HERE, and then MEASURED - see the verification section of the README.

OpenGLContent::BuildTerrain (OpenGLContent.cpp:2536) places image row i at local
y = i*scaleY - offsetY, so DATA row 0 is the LOW-y edge. But the data is not the image:
OpenGLContent::LoadTexture calls stbi_set_flip_vertically_on_load(true)
(OpenGLContent.cpp:1446) and nothing in the tree ever resets it, so by the time
Terrain.cpp calls stbi_load_16 the loader is returning rows bottom-up. Chain:

    PIL row 0  ->  PNG top  ->  stbi row N-1  ->  local y HIGH  ->  world y HIGH

so this script writes row 0 at y = +SITE_HALF and descends. Columns are unaffected:
column 0 is x = -SITE_HALF. The same applies to the albedo, which BuildTerrain maps with
uv = (j/(nx-1), i/(ny-1)) * uvScale over the same data grid.

THAT CHAIN HAS AN ORDERING DEPENDENCY. It only holds because <looks> is parsed before
<static> (ScenarioParser.cpp:197 and :213) and at least one look carries a texture, so
the flip flag is already set when the terrain loads. The seabed look in fish_farm.scn is
textured, so it holds - but if the site albedo were ever dropped for a flat colour, the
rows would silently mirror. There is a note to that effect in fish_farm.scn.

Run with --debug-marker to stamp an L-shaped notch in the world +x/+y quadrant and
re-measure rather than re-reasoning.

============================== THE BATHYMETRY ==============================
Fjord bathymetry is not a landscape. It is a glacially overdeepened trough: a near-flat
basin of post-glacial mud, walls that are bare rock close to their angle of repose, and
a sharp break of slope between the two. The model is:

  - A profile in x only for the large scale: shelf, wall, basin. The wall is 38 deg,
    which is steep for a subaerial slope and ordinary for a fjord side.
  - A terrace at 28-34 m, the sort of bedrock bench a fjord wall usually carries.
  - Bedrock relief on the wall only, +/- 1.6 m at 8-25 m scales, tapering to nothing in
    the basin. Mud drapes and smooths; rock does not.
  - Talus: a scatter of boulders at the foot of the wall, decaying eastwards.
  - Basin bedforms, +/- 0.35 m at 60-150 m scales, and a fine +/- 0.06 m roughness.
  - A deposition mound under each pen. Feed and faeces build a real mound of a few tens
    of centimetres over a production cycle, elongated downstream. This is geometry and
    not just colour, because the DVL altitude should see it.

============================== THE FOOTPRINT ==============================
The albedo carries what an MOM-B survey would photograph. Under and immediately
downstream of each pen the sediment goes anoxic and black and is patched with the white
mats of Beggiatoa; further out it grades through a darkened, enriched halo to ordinary
basin mud. The footprint is offset downstream by the current and elongated along it.

This is albedo only. A Terrain has ONE material, so the footprint is invisible to sonar
even though gassy organic mud is acoustically distinctive. A separate patch terrain was
considered and rejected: the seam is worse than the omission.
"""

import argparse
import os

import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter

# ----------------------------------------------------------------------------- geometry
SITE_HALF = 200.0          # the site is 400 x 400 m, centred on the origin

SHELF_X = -192.0           # shore shelf ends here
WALL_X = -116.0            # break of slope: wall meets basin
SHELF_DEPTH = 5.0          # m
WALL_FOOT_DEPTH = 64.0     # m
BASIN_EAST_DEPTH = 70.0    # m at x = +SITE_HALF

TERRACE_D0, TERRACE_D1 = 28.0, 34.0   # the bench on the wall
TERRACE_FLATTEN = 0.55                # how much of the local gradient it removes

# ------------------------------------------------------------------------------ the farm
PEN_RADIUS = 19.0986       # m, a 120 m circumference collar
PEN_CENTRES = ((0.0, -70.0), (0.0, 0.0), (0.0, 70.0))
CURRENT = (0.0, 0.30)      # m/s, the design current; the net is baked at this too

FOOTPRINT_CORE = 26.0      # m, radius of the anoxic core
FOOTPRINT_EDGE = 62.0      # m, radius at which it is gone
FOOTPRINT_LAG = 11.0       # m, how far downstream the core sits
FOOTPRINT_MOUND = 0.28     # m of accumulated waste at the centre

SEED = 20260909


def band_noise(rng, shape, sigma_px, mode="reflect"):
    """Zero-mean unit-variance band-limited noise. mode='wrap' makes it tile.

    The mean is removed explicitly: at the coarse end sigma is a good fraction of the
    domain, so the filtered field is barely more than one low-frequency realisation and
    its mean over the finite window is a random offset of the same order as its own
    standard deviation.
    """
    n = gaussian_filter(rng.standard_normal(shape), sigma_px, mode=mode)
    n = n - n.mean()
    s = n.std()
    return n / s if s > 1e-12 else n


def smoothstep(edge0, edge1, x):
    t = np.clip((x - edge0) / (edge1 - edge0), 0.0, 1.0)
    return t * t * (3.0 - 2.0 * t)


def site_grid(res):
    """World coordinates on the output grid. Row 0 is y = +SITE_HALF (see the header)."""
    n = int(round(2.0 * SITE_HALF / res)) + 1
    x = np.linspace(-SITE_HALF, SITE_HALF, n)
    y = np.linspace(SITE_HALF, -SITE_HALF, n)      # DESCENDING - row 0 is the high-y edge
    return np.meshgrid(x, y), n


def footprint_weight(X, Y):
    """0..1 organic enrichment, summed over the pens, offset downstream by the current."""
    ux, uy = CURRENT
    speed = float(np.hypot(ux, uy))
    dx, dy = (ux / speed, uy / speed) if speed > 1e-9 else (0.0, 0.0)

    w = np.zeros_like(X)
    for cx, cy in PEN_CENTRES:
        # centre displaced downstream, and stretched 1.6x along the current
        ox, oy = cx + dx * FOOTPRINT_LAG, cy + dy * FOOTPRINT_LAG
        ax, ay = X - ox, Y - oy
        along = ax * dx + ay * dy
        across = -ax * dy + ay * dx
        r = np.hypot(along / 1.6, across)
        w = np.maximum(w, 1.0 - smoothstep(FOOTPRINT_CORE, FOOTPRINT_EDGE, r))
    return w


def build_depth_field(res, debug_marker):
    """Water depth in metres, positive down, as d[row=y descending, col=x ascending]."""
    (X, Y), n = site_grid(res)
    rng = np.random.default_rng(SEED)

    # 1. The large-scale profile: shelf, wall, basin. A cosine ramp rather than a
    #    smoothstep across the wall, so the break of slope at its foot stays sharp.
    t = np.clip((X - SHELF_X) / (WALL_X - SHELF_X), 0.0, 1.0)
    d = SHELF_DEPTH + (WALL_FOOT_DEPTH - SHELF_DEPTH) * (0.5 - 0.5 * np.cos(np.pi * t))
    d = d + (BASIN_EAST_DEPTH - WALL_FOOT_DEPTH) * smoothstep(WALL_X, SITE_HALF, X)

    # 2. The bench. Pull depths inside the terrace band towards its middle, which
    #    flattens the slope there and steepens it just below - which is what a bench is.
    mid = 0.5 * (TERRACE_D0 + TERRACE_D1)
    band = 1.0 - smoothstep(0.0, 0.5 * (TERRACE_D1 - TERRACE_D0), np.abs(d - mid))
    d = d + TERRACE_FLATTEN * band * (mid - d)

    # 3. Rock relief, on the wall only. `wall` is 1 on the slope and 0 in the basin.
    wall = (smoothstep(SHELF_X - 6.0, SHELF_X + 10.0, X)
            * (1.0 - smoothstep(WALL_X - 14.0, WALL_X + 20.0, X)))
    d = d + wall * 1.6 * band_noise(rng, (n, n), 14.0 / res)
    d = d + wall * 0.5 * band_noise(rng, (n, n), 4.0 / res)

    # 4. Talus at the foot of the wall, decaying eastwards over 40 m.
    talus = np.exp(-np.clip(X - WALL_X, 0.0, None) / 40.0) * (X > WALL_X - 30.0)
    blocks = np.clip(band_noise(rng, (n, n), 2.5 / res), 0.9, None) - 0.9
    d = d - talus * 3.0 * blocks

    # 5. Basin bedforms and fine roughness. Mud drapes, so this is gentle.
    basin = smoothstep(WALL_X - 10.0, WALL_X + 30.0, X)
    d = d + basin * 0.35 * band_noise(rng, (n, n), 60.0 / res)
    d = d + 0.06 * band_noise(rng, (n, n), 2.0 / res)

    # 6. The waste mound under each pen. Real, and the DVL should see it.
    d = d - FOOTPRINT_MOUND * footprint_weight(X, Y) ** 1.5

    if debug_marker:
        # An L in the world +x/+y quadrant, long arm along +x. Used to settle the
        # image-row -> world-y direction; see ROW AND COLUMN ORDER in the header.
        d[(X > 120.0) & (X < 185.0) & (Y > 165.0) & (Y < 178.0)] = 20.0
        d[(X > 120.0) & (X < 133.0) & (Y > 120.0) & (Y < 178.0)] = 20.0

    return d


def write_bathymetry(path, d):
    """Encode against the field's own extrema. Returns (origin_z, height) for the XML."""
    d_min = float(d.min())
    d_max = float(d.max())
    q = np.rint((d_max - d) / (d_max - d_min) * 65535.0).astype(np.uint16)
    assert q.min() == 0 and q.max() == 65535, "encoding must saturate both ends"
    img = Image.fromarray(q)
    assert img.mode == "I;16", "heightmap must stay 16-bit, got mode %s" % img.mode
    img.save(path, optimize=True)
    return d_max, d_max - d_min


def write_seabed_albedo(path, size):
    """Site-scale albedo: depth-graded sediment plus the deposition footprint.

    uv_scale = 1 maps this once over the whole 400 m site, so a pixel is 400/size metres.
    That is the trade this look makes: a site-scale feature instead of a tiling normal
    map. See the header.
    """
    rng = np.random.default_rng(SEED + 1)
    x = np.linspace(-SITE_HALF, SITE_HALF, size)
    y = np.linspace(SITE_HALF, -SITE_HALF, size)
    X, Y = np.meshgrid(x, y)

    # Approximate depth on the texture grid - only the profile is needed for the tinting.
    t = np.clip((X - SHELF_X) / (WALL_X - SHELF_X), 0.0, 1.0)
    d = SHELF_DEPTH + (WALL_FOOT_DEPTH - SHELF_DEPTH) * (0.5 - 0.5 * np.cos(np.pi * t))
    d = d + (BASIN_EAST_DEPTH - WALL_FOOT_DEPTH) * smoothstep(WALL_X, SITE_HALF, X)

    # Three sediment end members, blended by depth.
    kelp_rock = np.array([0.17, 0.20, 0.13])    # shallow bedrock under kelp
    bare_rock = np.array([0.32, 0.31, 0.29])    # bare wall rock below the photic zone
    basin_mud = np.array([0.34, 0.32, 0.24])    # olive-grey mud

    w_kelp = 1.0 - smoothstep(8.0, 22.0, d)
    w_mud = smoothstep(46.0, 62.0, d)
    w_rock = np.clip(1.0 - w_kelp - w_mud, 0.0, 1.0)
    rgb = (kelp_rock[None, None, :] * w_kelp[:, :, None]
           + bare_rock[None, None, :] * w_rock[:, :, None]
           + basin_mud[None, None, :] * w_mud[:, :, None])

    # Texture: coarse mottling everywhere, plus a finer grain.
    coarse = band_noise(rng, (size, size), 24.0)
    fine = band_noise(rng, (size, size), 3.0)
    shade = np.clip(1.0 + 0.16 * coarse + 0.07 * fine, 0.7, 1.35)
    rgb = rgb * shade[:, :, None]

    # The deposition footprint: an anoxic black core patched with white Beggiatoa mats,
    # grading out through a darkened, organically enriched halo.
    fw = footprint_weight(X, Y)
    anoxic = np.array([0.055, 0.048, 0.040])
    core = smoothstep(0.35, 0.92, fw)
    rgb = rgb * (1.0 - core[:, :, None]) + anoxic[None, None, :] * core[:, :, None]
    rgb = rgb * (1.0 - 0.35 * np.clip(fw - core, 0.0, 1.0))[:, :, None]

    mats = np.clip(band_noise(rng, (size, size), 5.0), 1.15, None) - 1.15
    mats = np.clip(mats * 2.6, 0.0, 1.0) * smoothstep(0.45, 0.75, fw)
    beggiatoa = np.array([0.80, 0.79, 0.72])
    rgb = rgb * (1.0 - mats[:, :, None]) + beggiatoa[None, None, :] * mats[:, :, None]

    rgb = np.clip(rgb, 0.0, 1.0)
    Image.fromarray((rgb * 255.0 + 0.5).astype(np.uint8), mode="RGB").save(path, optimize=True)


# =============================================================================== geometry
#
# Every .obj this script writes is TRIANGLES ONLY. rapidobj::Triangulate inverts the winding
# of any face with more than four vertices when the file carries no vertex normals, which
# silently guts hydrodynamic drag and is invisible from the outside - see the PHYSICS MESH
# section of bluerov2_heavy.scn. Emitting triangles with explicit normals sidesteps it, and
# write_obj asserts the closure of every physics mesh it writes.


class ObjMesh:
    """A triangle soup with shared positions, per-face normals and optional UVs.

    Normals are written explicitly and per face. That is not just for the n-gon trap: it
    also keeps OpenGLContent::CheckAndRepairFaceVertexOrder off the geometry. That function
    (OpenGLContent.cpp:2599) flips any triangle whose geometric normal opposes the stored
    normals of all three of its vertices, which is exactly what a back face built by
    re-winding the same vertices looks like - so naive double-sided geometry is silently
    turned back into a coincident duplicate of its front face.
    """

    def __init__(self, textured=False):
        self.v = []
        self.vn = []
        self.vt = []
        self.f = []            # (vi, ti, ni) triples, 1-based, ti None when untextured
        self.textured = textured

    def add_v(self, p):
        self.v.append((float(p[0]), float(p[1]), float(p[2])))
        return len(self.v)

    def add_vn(self, n):
        n = np.asarray(n, dtype=float)
        ln = float(np.linalg.norm(n))
        n = n / ln if ln > 1e-12 else np.array([0.0, 0.0, 1.0])
        self.vn.append((n[0], n[1], n[2]))
        return len(self.vn)

    def add_vt(self, uv):
        self.vt.append((float(uv[0]), float(uv[1])))
        return len(self.vt)

    def tri(self, a, b, c, n, ta=None, tb=None, tc=None):
        self.f.append(((a, ta, n), (b, tb, n), (c, tc, n)))

    def quad(self, a, b, c, d, n, uvs=None):
        """A planar quad as two triangles, wound a-b-c and a-c-d."""
        if uvs is None:
            self.tri(a, b, c, n)
            self.tri(a, c, d, n)
        else:
            self.tri(a, b, c, n, uvs[0], uvs[1], uvs[2])
            self.tri(a, c, d, n, uvs[0], uvs[2], uvs[3])

    def face_normal(self, a, b, c):
        pa, pb, pc = (np.array(self.v[i - 1]) for i in (a, b, c))
        return np.cross(pb - pa, pc - pa)

    def closure(self):
        """sum(area * normal) over the mesh. Zero for a closed surface; the check that
        catches inverted winding on a physics mesh before it reaches the simulator."""
        s = np.zeros(3)
        for f in self.f:
            pa, pb, pc = (np.array(self.v[k[0] - 1]) for k in f)
            s = s + 0.5 * np.cross(pb - pa, pc - pa)
        return s

    def n_tris(self):
        return len(self.f)

    def write(self, path, comment):
        out = ["# %s" % comment,
               "# Generated by make_fish_farm.py - do not edit by hand.",
               "# %d vertices, %d triangles" % (len(self.v), len(self.f))]
        for p in self.v:
            out.append("v %.5f %.5f %.5f" % p)
        if self.textured:
            for t in self.vt:
                out.append("vt %.5f %.5f" % t)
        for n in self.vn:
            out.append("vn %.5f %.5f %.5f" % n)
        if self.textured:
            for f in self.f:
                out.append("f %d/%d/%d %d/%d/%d %d/%d/%d"
                           % (f[0][0], f[0][1], f[0][2],
                              f[1][0], f[1][1], f[1][2],
                              f[2][0], f[2][1], f[2][2]))
        else:
            for f in self.f:
                out.append("f %d//%d %d//%d %d//%d"
                           % (f[0][0], f[0][2], f[1][0], f[1][2], f[2][0], f[2][2]))
        with open(path, "w", newline="\n") as fh:
            fh.write("\n".join(out) + "\n")
        return os.path.getsize(path)


def frame_from(tangent, hint):
    """An orthonormal (t, n, b) with n as close to `hint` as the tangent allows."""
    t = np.asarray(tangent, dtype=float)
    t = t / max(np.linalg.norm(t), 1e-12)
    n = np.asarray(hint, dtype=float) - t * float(np.dot(hint, t))
    ln = float(np.linalg.norm(n))
    if ln < 1e-9:                                   # hint parallel to the tangent
        alt = np.array([0.0, 0.0, 1.0]) if abs(t[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        n = alt - t * float(np.dot(alt, t))
        ln = float(np.linalg.norm(n))
    n = n / ln
    return t, n, np.cross(t, n)


def strand(mesh, pts, width, hints, closed=False, skip=None):
    """A twine as a triangular prism swept along a polyline.

    A TRIANGULAR PRISM, NOT A FLAT RIBBON. A ribbon lying in the net surface has a radial
    normal, so viewing the net edge-on - which is what flying ALONG it does, and that is
    most of an inspection - presents every strand at zero width and the net vanishes exactly
    when it is being looked at. A prism has a real silhouette from every direction, needs no
    back faces, and costs 6 triangles a segment against a double-sided ribbon's 4.

    `width` is the side of the equilateral cross-section; `hints` gives the outward direction
    at each point, so one vertex of the triangle points away from the net surface.
    `skip(i)` may reject the segment from pts[i] to pts[i+1] - that is how the tear is cut.
    """
    rho = width / np.sqrt(3.0)                      # circumradius of the cross-section
    n_pts = len(pts)
    rings = []
    for i in range(n_pts):
        if closed:
            a, b = pts[i - 1], pts[(i + 1) % n_pts]
        else:
            a = pts[max(i - 1, 0)]
            b = pts[min(i + 1, n_pts - 1)]
        t, n, bn = frame_from(np.asarray(b) - np.asarray(a), hints[i])
        ring = []
        for k in range(3):
            phi = 2.0 * np.pi * k / 3.0
            ring.append(mesh.add_v(np.asarray(pts[i]) + rho * (np.cos(phi) * n + np.sin(phi) * bn)))
        rings.append((ring, n, bn))

    last = n_pts if closed else n_pts - 1
    for i in range(last):
        if skip is not None and skip(i):
            continue
        j = (i + 1) % n_pts
        ra, na, ba = rings[i]
        rb, _, _ = rings[j]
        for k in range(3):
            k2 = (k + 1) % 3
            # outward normal of this side face: the bisector of the two cross-section angles
            phi = 2.0 * np.pi * (k + 0.5) / 3.0
            ni = mesh.add_vn(np.cos(phi) * na + np.sin(phi) * ba)
            mesh.quad(ra[k], ra[k2], rb[k2], rb[k], ni)


def tube(mesh, pts, radius, sides=8, closed=False, cap=False, hint=(0.0, 0.0, 1.0)):
    """A round tube swept along a polyline: ropes, pipes, handrails, mooring lines."""
    n_pts = len(pts)
    rings = []
    for i in range(n_pts):
        if closed:
            a, b = pts[i - 1], pts[(i + 1) % n_pts]
        else:
            a = pts[max(i - 1, 0)]
            b = pts[min(i + 1, n_pts - 1)]
        t, n, bn = frame_from(np.asarray(b) - np.asarray(a), hint)
        ring = [mesh.add_v(np.asarray(pts[i])
                           + radius * (np.cos(2.0 * np.pi * k / sides) * n
                                       + np.sin(2.0 * np.pi * k / sides) * bn))
                for k in range(sides)]
        rings.append((ring, n, bn))

    last = n_pts if closed else n_pts - 1
    for i in range(last):
        j = (i + 1) % n_pts
        ra, na, ba = rings[i]
        rb, _, _ = rings[j]
        for k in range(sides):
            k2 = (k + 1) % sides
            phi = 2.0 * np.pi * (k + 0.5) / sides
            ni = mesh.add_vn(np.cos(phi) * na + np.sin(phi) * ba)
            mesh.quad(ra[k], ra[k2], rb[k2], rb[k], ni)

    if cap and not closed:
        for ring, idx, sgn in ((rings[0][0], 0, -1.0), (rings[-1][0], -1, 1.0)):
            c = mesh.add_v(pts[idx])
            t, _, _ = frame_from(np.asarray(pts[1]) - np.asarray(pts[0]), hint)
            ni = mesh.add_vn(sgn * t)
            for k in range(sides):
                k2 = (k + 1) % sides
                if sgn > 0:
                    mesh.tri(c, ring[k], ring[k2], ni)
                else:
                    mesh.tri(c, ring[k2], ring[k], ni)


def torus(mesh, centre, radius, tube_radius, n_major=64, n_minor=8, axis=(0.0, 0.0, 1.0)):
    """A closed torus about `axis`: the collar tubes, the handrail, the sinker tube."""
    pts = []
    hint = np.array([0.0, 0.0, 1.0]) if abs(axis[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    for k in range(n_major):
        a = 2.0 * np.pi * k / n_major
        pts.append(np.asarray(centre) + radius * (np.cos(a) * np.array([1.0, 0.0, 0.0])
                                                  + np.sin(a) * np.array([0.0, 1.0, 0.0])))
    tube(mesh, pts, tube_radius, sides=n_minor, closed=True, hint=hint)


def box(mesh, centre, half, yaw=0.0):
    """An axis-aligned box, optionally yawed about z. Closed, so it passes the closure test."""
    c, s = np.cos(yaw), np.sin(yaw)
    R = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    corners = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            for sz in (-1, 1):
                p = R.dot(np.array([sx * half[0], sy * half[1], sz * half[2]]))
                corners.append(mesh.add_v(np.asarray(centre) + p))
    # index = 4*ix + 2*iy + iz with ix,iy,iz in {0,1} mapping to {-1,+1}
    def idx(ix, iy, iz):
        return corners[4 * ix + 2 * iy + iz]
    faces = [((0, 0, 0), (0, 1, 0), (0, 1, 1), (0, 0, 1), R.dot([-1, 0, 0])),
             ((1, 0, 0), (1, 0, 1), (1, 1, 1), (1, 1, 0), R.dot([1, 0, 0])),
             ((0, 0, 0), (0, 0, 1), (1, 0, 1), (1, 0, 0), R.dot([0, -1, 0])),
             ((0, 1, 0), (1, 1, 0), (1, 1, 1), (0, 1, 1), R.dot([0, 1, 0])),
             ((0, 0, 0), (1, 0, 0), (1, 1, 0), (0, 1, 0), R.dot([0, 0, -1])),
             ((0, 0, 1), (0, 1, 1), (1, 1, 1), (1, 0, 1), R.dot([0, 0, 1]))]
    for a, b, cc, d, n in faces:
        ni = mesh.add_vn(n)
        mesh.quad(idx(*a), idx(*b), idx(*cc), idx(*d), ni)


# --------------------------------------------------------------------------- the net model
#
# SOLIDITY, NOT MESH SIZE, IS WHAT IS PRESERVED.
#
# Real knotless PA netting for salmon is 25 mm bar with 2.4 mm twine. Over the 2900 square
# metres of net on one pen that is about three million meshes, which cannot be drawn. So the
# pitch is coarsened and THE TWINE IS WIDENED TO KEEP THE SOLIDITY, the fraction of the net
# that is material:
#
#     open fraction = (1 - d/lambda)^2         Sn = 1 - open fraction
#     w(lambda') = lambda' * (1 - sqrt(1 - Sn))
#
# Solidity is what decides how much light the net passes and therefore how grey it looks at
# range - which is what an inspection actually judges, and what makes the fouled band read as
# fouled. A close-up shows meshes an order of magnitude too big, and that is the deception;
# it is stated in fish_farm.scn and in the README.
NET_BAR = 0.025            # m, real bar length
NET_TWINE = 0.0024         # m, real twine diameter
SN_CLEAN = 1.0 - (1.0 - NET_TWINE / NET_BAR) ** 2       # 0.1825
SN_FOULED = 0.35           # hydroids and mussels on the top few metres

PEN_WALL_DEPTH = 15.0      # m, bottom of the cylindrical wall
PEN_CONE_DEPTH = 20.0      # m, the cone's apex
FOUL_DEPTH = 5.0           # m, bottom of the fouled band

SINKER_MASS = 5000.0       # kg, submerged mass of the bottom ring and its weights
SINKER_RADIUS = 0.1575     # m, a 315 mm HDPE pipe
NET_AREAL_MASS = 0.41      # kg/m2 of netting, dry
RHO_NYLON = 1140.0
RHO_SEA = 1026.0
CD_SCREEN = 1.2            # cylinder drag coefficient for the twine
WAKE_SHIELDING = 0.6       # what the lee half of the net feels, relative to the weather half
G = 9.81

TEAR_AZIMUTH = np.pi       # rad, the tear faces the vehicle's spawn
TEAR_HALF_AZ = 0.0209      # rad, 0.40 m of arc at the net radius
TEAR_Z0, TEAR_Z1 = 5.40, 6.05   # m, on the second lap of the default survey helix


def net_deflection():
    """Downstream displacement of the net, in metres, as a function of depth.

    The net wall is a curtain of vertical strips hanging from the collar with the sinker
    tube's submerged weight at the bottom. The tension at depth z carries everything below
    it, so the local slope of the curtain is

        dy/dz = H(z) / V(z)

    with H the accumulated horizontal drag below z and V the accumulated submerged weight
    below z. Drag on netting is the drag of its twine, so it goes with the solidity:

        q_normal = 0.5 * rho * Cd * Sn * u^2         per square metre of net, flow normal

    THE INCIDENCE FACTORS ARE THE PART THAT IS EASY TO GET WRONG. Applying q_normal to the
    whole netting area over-predicts the deflection by a factor of about three, because most
    of the net is not square to the flow:

      - The wall is a cylinder. A strip at azimuth phi has its normal at incidence to the
        flow, and a plane screen carries a normal force going as cos^2 of that incidence.
        Averaged around the circle that is 1/2. The lee half then sits in the weather half's
        wake and feels WAKE_SHIELDING of the free-stream load, so

            K_wall = 0.5 * (1 + WAKE_SHIELDING) / 2 = 0.40

      - The cone is 14.7 degrees from horizontal, so its normal is within 15 degrees of
        vertical and the flow is nearly along it. Both the incidence factor and the
        projection of the resulting normal force onto the horizontal are small:

            K_cone = nr^3,   nr = (cone depth) / (slant length) = 0.253   ->  0.016

        which is 2 % of the wall's. The cone contributes essentially nothing horizontally,
        and it is kept in only so that the term is visible rather than silently dropped.

    The sinker tube is a stiff HDPE ring, so it translates rather than distorting, and the
    net is displaced in ONE direction rather than deforming per azimuth. That is the
    modelling choice worth knowing about: a real net also narrows and loses volume, and this
    one does not - the reported retention therefore only counts the shortening.

    Returns (defl(z) callable, delta_max, volume_retention).
    """
    u = float(np.hypot(*CURRENT))
    q = 0.5 * RHO_SEA * CD_SCREEN * SN_CLEAN * u * u              # N/m2 of net, flow normal
    w_net = NET_AREAL_MASS * (1.0 - RHO_SEA / RHO_NYLON) * G      # N/m2, submerged
    ring_drag = 0.5 * RHO_SEA * 1.0 * (2.0 * SINKER_RADIUS) * u * u   # N/m of circumference
    ring_weight = SINKER_MASS * G / (2.0 * np.pi * PEN_RADIUS)        # N/m of circumference

    cone_height = PEN_CONE_DEPTH - PEN_WALL_DEPTH
    slant = np.hypot(PEN_RADIUS, cone_height)
    k_wall = 0.5 * (1.0 + WAKE_SHIELDING) / 2.0
    k_cone = (cone_height / slant) ** 3

    zs = np.linspace(0.0, PEN_CONE_DEPTH, 400)
    slope = np.zeros_like(zs)
    for i, z in enumerate(zs):
        wall_below = max(PEN_WALL_DEPTH - z, 0.0)
        cone_below = slant * (PEN_CONE_DEPTH - max(z, PEN_WALL_DEPTH)) / cone_height
        H = q * (k_wall * wall_below + k_cone * cone_below)
        V = w_net * (wall_below + cone_below)
        if z <= PEN_WALL_DEPTH:
            H += ring_drag
            V += ring_weight
        slope[i] = H / max(V, 1e-6)
    defl = np.concatenate(([0.0], np.cumsum(0.5 * (slope[1:] + slope[:-1]) * np.diff(zs))))

    ux, uy = CURRENT
    speed = max(float(np.hypot(ux, uy)), 1e-9)
    dirn = np.array([ux / speed, uy / speed, 0.0])

    def f(z):
        return dirn * float(np.interp(z, zs, defl))

    d_max = float(np.interp(PEN_WALL_DEPTH, zs, defl))
    eff_depth = np.sqrt(max(PEN_WALL_DEPTH ** 2 - d_max ** 2, 0.0))
    return f, d_max, eff_depth / PEN_WALL_DEPTH


def wall_point(r, phi, z, defl):
    return np.array([r * np.cos(phi), r * np.sin(phi), z]) + defl(z)


def build_net_band(gra, shell, z0, z1, sn, pitch, n_v_seg, n_az, defl,
                   az0=0.0, az1=2.0 * np.pi, tear=False):
    """One depth band of the cylindrical wall, as twine into `gra` and a shell into `shell`.

    Vertical strands sit just outside the nominal radius and horizontal strands just inside,
    so they interleave at the crossings instead of coplanar-fighting. The shell is on the
    nominal radius, which is therefore what the net profiler ranges against.
    """
    w = pitch * (1.0 - np.sqrt(1.0 - sn))
    r = PEN_RADIUS
    full = abs((az1 - az0) - 2.0 * np.pi) < 1e-9

    def in_tear(phi, z):
        if not tear:
            return False
        dphi = (phi - TEAR_AZIMUTH + np.pi) % (2.0 * np.pi) - np.pi
        return abs(dphi) < TEAR_HALF_AZ and TEAR_Z0 < z < TEAR_Z1

    # --- vertical strands
    n_strands = max(int(round(r * (az1 - az0) / pitch)), 1)
    zs = np.linspace(z0, z1, n_v_seg + 1)
    for k in range(n_strands):
        phi = az0 + (az1 - az0) * (k + 0.5) / n_strands if not full \
            else az0 + 2.0 * np.pi * k / n_strands
        pts = [wall_point(r + 0.5 * w, phi, z, defl) for z in zs]
        hints = [np.array([np.cos(phi), np.sin(phi), 0.0])] * len(pts)
        strand(gra, pts, w, hints,
               skip=lambda i, phi=phi: in_tear(phi, 0.5 * (zs[i] + zs[i + 1])))

    # --- horizontal strands
    n_rings = max(int(round((z1 - z0) / pitch)), 1)
    phis = ([az0 + 2.0 * np.pi * k / n_az for k in range(n_az)] if full
            else list(np.linspace(az0, az1, max(int(round(n_az * (az1 - az0) / (2.0 * np.pi))), 2))))
    for k in range(n_rings):
        z = z0 + (z1 - z0) * (k + 0.5) / n_rings
        pts = [wall_point(r - 0.5 * w, p, z, defl) for p in phis]
        hints = [np.array([np.cos(p), np.sin(p), 0.0]) for p in phis]
        strand(gra, pts, w, hints, closed=full,
               skip=lambda i, z=z: in_tear(0.5 * (phis[i] + phis[(i + 1) % len(phis)]), z))

    # --- the smooth physics shell
    n_rows = max(int(round((z1 - z0) / 2.0)), 2)
    sphis = ([az0 + 2.0 * np.pi * k / n_az for k in range(n_az + 1)] if full
             else list(np.linspace(az0, az1, max(int(round(n_az * (az1 - az0) / (2.0 * np.pi))) + 1, 2))))
    szs = np.linspace(z0, z1, n_rows + 1)
    grid = [[shell.add_v(wall_point(r, p, z, defl)) for p in sphis] for z in szs]
    for i in range(n_rows):
        for j in range(len(sphis) - 1):
            pm = 0.5 * (sphis[j] + sphis[j + 1])
            zm = 0.5 * (szs[i] + szs[i + 1])
            if in_tear(pm, zm):
                continue
            ni = shell.add_vn([np.cos(pm), np.sin(pm), 0.0])
            shell.quad(grid[i][j], grid[i][j + 1], grid[i + 1][j + 1], grid[i + 1][j], ni)


def build_net_cone(gra, shell, sn, pitch, n_slant_seg, n_az, defl):
    """The conical bottom, hanging from the sinker tube down to the apex."""
    w = pitch * (1.0 - np.sqrt(1.0 - sn))
    z0, z1 = PEN_WALL_DEPTH, PEN_CONE_DEPTH

    def cone_point(t, phi, dr=0.0):
        """t in [0,1] from the sinker tube to the apex."""
        r = PEN_RADIUS * (1.0 - t) + dr
        z = z0 + (z1 - z0) * t
        return np.array([r * np.cos(phi), r * np.sin(phi), z]) + defl(z)

    # outward normal of the cone surface (pointing away from the axis and slightly down)
    slant = np.hypot(PEN_RADIUS, z1 - z0)
    nr, nz = (z1 - z0) / slant, PEN_RADIUS / slant

    ts = np.linspace(0.0, 1.0, n_slant_seg + 1)
    n_strands = max(int(round(2.0 * np.pi * PEN_RADIUS / pitch)), 1)
    for k in range(n_strands):
        phi = 2.0 * np.pi * k / n_strands
        pts = [cone_point(t, phi, 0.5 * w) for t in ts]
        hints = [np.array([nr * np.cos(phi), nr * np.sin(phi), nz])] * len(pts)
        strand(gra, pts, w, hints)

    n_rings = max(int(round(slant / pitch)), 1)
    for k in range(n_rings):
        t = (k + 0.5) / n_rings
        # keep the ring pitch roughly constant by dropping segments as the radius shrinks
        m = max(int(round(n_az * (1.0 - t))), 6)
        phis = [2.0 * np.pi * j / m for j in range(m)]
        pts = [cone_point(t, p, -0.5 * w) for p in phis]
        hints = [np.array([nr * np.cos(p), nr * np.sin(p), nz]) for p in phis]
        strand(gra, pts, w, hints, closed=True)

    sts = np.linspace(0.0, 1.0, max(n_slant_seg, 4) + 1)
    sphis = [2.0 * np.pi * k / n_az for k in range(n_az + 1)]
    grid = [[shell.add_v(cone_point(t, p)) for p in sphis] for t in sts]
    for i in range(len(sts) - 1):
        for j in range(n_az):
            pm = 0.5 * (sphis[j] + sphis[j + 1])
            ni = shell.add_vn([nr * np.cos(pm), nr * np.sin(pm), nz])
            shell.quad(grid[i][j], grid[i][j + 1], grid[i + 1][j + 1], grid[i + 1][j], ni)


# ------------------------------------------------------------------------------- the salmon
#
# A 4 kg Atlantic salmon: 68 cm fork length, fusiform, with the counter-shading that is most
# of what makes a fish read as a fish - dark olive back, silver flank, white belly, and the
# scatter of black spots above the lateral line. The body is a lofted surface of elliptical
# sections; the fins are flat blades. The mesh points along +x, which is the vehicle frame's
# forward, so a fish's yaw is its heading.
FISH_LENGTH = 0.68
FISH_RINGS = 22
FISH_SIDES = 10


def fish_profile(t):
    """Half-height and half-width, in fractions of fork length, at t from nose to tail."""
    # a nose that comes to a rounded point, the deepest section a third of the way back,
    # and a caudal peduncle that is narrow but not zero
    h = 0.128 * np.sin(np.pi * np.clip(t, 0.0, 1.0) ** 0.62) ** 1.15
    h = np.maximum(h * (1.0 - 0.72 * smoothstep(0.68, 1.0, t)), 0.012)
    w = 0.62 * h * (1.0 - 0.35 * smoothstep(0.55, 1.0, t))
    return h, w


def build_salmon(mesh):
    ts = np.linspace(0.0, 1.0, FISH_RINGS)
    rings = []
    for i, t in enumerate(ts):
        h, w = fish_profile(t)
        x = FISH_LENGTH * (0.5 - t)
        ring = []
        for k in range(FISH_SIDES):
            a = 2.0 * np.pi * k / FISH_SIDES
            y = FISH_LENGTH * w * np.sin(a)
            z = -FISH_LENGTH * h * np.cos(a)          # z is DOWN, so -cos puts k=0 on the back
            ring.append((mesh.add_v((x, y, z)),
                         mesh.add_vt((t, k / float(FISH_SIDES)))))
        rings.append(ring)

    for i in range(FISH_RINGS - 1):
        for k in range(FISH_SIDES):
            k2 = (k + 1) % FISH_SIDES
            a, b = rings[i][k], rings[i][k2]
            c, d = rings[i + 1][k2], rings[i + 1][k]
            n = mesh.face_normal(a[0], b[0], c[0])
            ni = mesh.add_vn(n)
            mesh.tri(a[0], b[0], c[0], ni, a[1], b[1], c[1])
            ni = mesh.add_vn(mesh.face_normal(a[0], c[0], d[0]))
            mesh.tri(a[0], c[0], d[0], ni, a[1], c[1], d[1])

    def blade(quad_pts, uv):
        """A flat, two-sided fin. Two-sided is safe here because each side gets its own
        outward normal, so CheckAndRepairFaceVertexOrder has nothing to repair."""
        vs = [mesh.add_v(p) for p in quad_pts]
        vts = [mesh.add_vt(u) for u in uv]
        n = mesh.face_normal(vs[0], vs[1], vs[2])
        for sgn in (1.0, -1.0):
            ni = mesh.add_vn(sgn * n)
            if sgn > 0:
                mesh.quad(vs[0], vs[1], vs[2], vs[3], ni, [vts[0], vts[1], vts[2], vts[3]])
            else:
                mesh.quad(vs[3], vs[2], vs[1], vs[0], ni, [vts[3], vts[2], vts[1], vts[0]])

    L = FISH_LENGTH
    fin_uv = [(0.96, 0.05), (0.99, 0.05), (0.99, 0.20), (0.96, 0.20)]
    # caudal, forked
    blade([(-0.50 * L, 0.0, -0.005), (-0.62 * L, 0.0, -0.105),
           (-0.56 * L, 0.0, 0.0), (-0.62 * L, 0.0, 0.105)], fin_uv)
    # dorsal
    blade([(0.06 * L, 0.0, -0.115), (0.02 * L, 0.0, -0.205),
           (-0.09 * L, 0.0, -0.175), (-0.10 * L, 0.0, -0.095)], fin_uv)
    # adipose
    blade([(-0.36 * L, 0.0, -0.048), (-0.39 * L, 0.0, -0.085),
           (-0.44 * L, 0.0, -0.075), (-0.43 * L, 0.0, -0.040)], fin_uv)
    # anal
    blade([(-0.24 * L, 0.0, 0.058), (-0.28 * L, 0.0, 0.135),
           (-0.36 * L, 0.0, 0.105), (-0.35 * L, 0.0, 0.048)], fin_uv)
    # pectorals, one each side
    for s in (1.0, -1.0):
        blade([(0.24 * L, s * 0.012, 0.045), (0.16 * L, s * 0.075, 0.115),
               (0.10 * L, s * 0.070, 0.095), (0.16 * L, s * 0.012, 0.048)], fin_uv)


def write_salmon_texture(path, size=(512, 256)):
    """Counter-shaded flank: dark olive back, silver side, white belly, black spots.

    v runs around the section with v = 0 on the back, so the gradient is a function of v
    alone; u runs nose to tail.
    """
    rng = np.random.default_rng(SEED + 2)
    w, h = size
    u = np.linspace(0.0, 1.0, w)[None, :]
    v = np.linspace(0.0, 1.0, h)[:, None]
    ring = np.minimum(v, 1.0 - v) * 2.0          # 0 on the back, 1 on the belly

    back = np.array([0.16, 0.19, 0.16])
    flank = np.array([0.66, 0.69, 0.72])
    belly = np.array([0.90, 0.91, 0.90])
    t1 = smoothstep(0.12, 0.42, ring)
    t2 = smoothstep(0.62, 0.95, ring)
    rgb = (back[None, None, :] * (1.0 - t1)[:, :, None]
           + flank[None, None, :] * (t1 - t1 * t2)[:, :, None]
           + belly[None, None, :] * (t1 * t2)[:, :, None])
    rgb = np.broadcast_to(rgb, (h, w, 3)).copy()

    # lateral line and a faint iridescent banding
    rgb *= (1.0 - 0.10 * np.exp(-((ring - 0.46) / 0.035) ** 2))[:, :, None]
    rgb *= (1.0 + 0.05 * np.sin(u * 42.0))[:, :, None]

    # the black spots a salmon carries above the lateral line
    spots = np.zeros((h, w))
    for _ in range(90):
        su, sv = rng.uniform(0.05, 0.92), rng.uniform(0.0, 0.34)
        sv = sv if rng.random() < 0.5 else 1.0 - sv
        rad = rng.uniform(0.010, 0.022)
        spots = np.maximum(spots, np.exp(-(((u - su) / rad) ** 2 + ((v - sv) / (rad * 2.0)) ** 2)))
    rgb *= (1.0 - 0.80 * np.clip(spots, 0.0, 1.0))[:, :, None]

    # the head is darker and the caudal fin darker still
    rgb *= (1.0 - 0.25 * smoothstep(0.10, 0.0, u))[:, :, None]
    rgb *= (1.0 - 0.35 * smoothstep(0.94, 1.0, u))[:, :, None]

    rgb = np.clip(rgb * (1.0 + 0.05 * band_noise(rng, (h, w), 2.0))[:, :, None], 0.0, 1.0)
    Image.fromarray((rgb * 255.0 + 0.5).astype(np.uint8), mode="RGB").save(path, optimize=True)


# ---------------------------------------------------------------------- the farm structure
#
# Dimensions are those of an ordinary Norwegian plastic pen and a 400 tonne feed barge. The
# collar is the piece that matters most for looks: it is what makes the site read as a farm
# from the surface, and it is the only structure the pilot can use to tell one pen from
# another. Everything here is a static body - the collar does not heave and the pens do not
# swing on their moorings.
COLLAR_R_INNER = 18.60     # m, centreline of the inner HDPE tube
COLLAR_R_OUTER = 19.30     # m, centreline of the outer tube
COLLAR_TUBE = 0.20         # m, radius: a 400 mm pipe
COLLAR_Z = -0.06           # m, tube centres just proud of the surface
RAIL_R = 19.60             # m
RAIL_Z = -0.95             # m, 0.9 m above the walkway
STANCHIONS = 50

BARGE_XY = (0.0, -145.0)   # the feed barge, 75 m off the southern pen
BARGE_L, BARGE_W = 22.0, 12.0
BARGE_DRAFT = 2.5          # m below the waterline
BARGE_FREEBOARD = 2.0      # m above it
SILO_R, SILO_H = 1.5, 7.0

GRID_X = 34.0              # m, the mooring frame's half-width
GRID_Y = (-105.0, -35.0, 35.0, 105.0)
GRID_Z = 10.0              # m, depth of the frame ropes
ANCHOR_REACH = 90.0        # m, horizontal run from a grid node to its anchor
ROPE_R = 0.024             # m, a 48 mm mooring rope


def build_collar(gra, phy):
    """Twin HDPE flotation tubes, brackets, stanchions and a handrail.

    Everything here crosses z = 0 rather than sitting on it, which is the rule the
    waterline section of fish_farm.scn sets out: the pipes are centred 60 mm above the
    surface with a 200 mm radius, so no facet lies in the plane where the underwater and
    above-water shading regimes meet.
    """
    torus(gra, (0.0, 0.0, COLLAR_Z), COLLAR_R_INNER, COLLAR_TUBE, 72, 10)
    torus(gra, (0.0, 0.0, COLLAR_Z), COLLAR_R_OUTER, COLLAR_TUBE, 72, 10)
    torus(gra, (0.0, 0.0, RAIL_Z), RAIL_R, 0.045, 72, 6)

    for k in range(STANCHIONS):
        a = 2.0 * np.pi * k / STANCHIONS
        c, s = np.cos(a), np.sin(a)
        # the upright, from the walkway to the handrail
        box(gra, (RAIL_R * c, RAIL_R * s, 0.5 * (RAIL_Z + 0.10)),
            (0.035, 0.035, 0.5 * abs(RAIL_Z - 0.10)), yaw=a)
        # the bracket that clamps the two tubes together
        mid = 0.5 * (COLLAR_R_INNER + COLLAR_R_OUTER)
        box(gra, (mid * c, mid * s, COLLAR_Z - 0.22),
            (0.5 * (COLLAR_R_OUTER - COLLAR_R_INNER) + 0.30, 0.06, 0.03), yaw=a)

    # The physics is the two tubes only, at a quarter of the resolution. Nothing else is
    # reachable by a vehicle that has to stay below the surface, and the stanchions would
    # cost 50 collision trees for a handrail nobody can touch.
    torus(phy, (0.0, 0.0, COLLAR_Z), COLLAR_R_INNER, COLLAR_TUBE, 24, 6)
    torus(phy, (0.0, 0.0, COLLAR_Z), COLLAR_R_OUTER, COLLAR_TUBE, 24, 6)


def build_sinker_tube(mesh, defl):
    """The bottom ring: a water-filled 315 mm HDPE pipe, five tonnes submerged.

    It is what holds the net open, and its downstream offset is the visible signature of
    the baked current - the ring is 1.7 m out of plumb at the design current, and the whole
    lower net leans with it.
    """
    z = PEN_WALL_DEPTH
    d = defl(z)
    torus(mesh, (d[0], d[1], z), PEN_RADIUS, SINKER_RADIUS, 64, 8)


def build_pen_ropes(mesh, defl):
    """Lifting straps, cone ropes and the oxygen sensor string.

    The straps are what actually carry the net: the twine takes almost no load. They run
    down the OUTSIDE of the wall, which is also where an ROV meets them, and they are the
    reason an inspection transect has to be flown with something in it to hit.
    """
    n_straps = 16
    zs = np.linspace(0.0, PEN_WALL_DEPTH, 9)
    for k in range(n_straps):
        a = 2.0 * np.pi * k / n_straps
        pts = [np.array([(PEN_RADIUS + 0.07) * np.cos(a), (PEN_RADIUS + 0.07) * np.sin(a), z])
               + defl(z) for z in zs]
        tube(mesh, pts, 0.012, sides=5)

    ts = np.linspace(0.0, 1.0, 6)
    for k in range(n_straps):
        a = 2.0 * np.pi * k / n_straps
        pts = []
        for t in ts:
            r = (PEN_RADIUS + 0.07) * (1.0 - t)
            z = PEN_WALL_DEPTH + (PEN_CONE_DEPTH - PEN_WALL_DEPTH) * t
            pts.append(np.array([r * np.cos(a), r * np.sin(a), z]) + defl(z))
        tube(mesh, pts, 0.012, sides=5)

    # The oxygen and temperature string, hanging inside the pen off the collar. Every site
    # has one; it is the thing that actually decides whether the fish are fed today.
    zs = np.linspace(0.0, 15.0, 9)
    pts = [np.array([12.0, 0.0, z]) + defl(z) * (z / PEN_WALL_DEPTH) for z in zs]
    tube(mesh, pts, 0.008, sides=4)
    box(mesh, np.array([12.0, 0.0, 15.0]) + defl(15.0), (0.09, 0.06, 0.20))


def build_mortality(mesh, defl):
    """The dead-fish collection cone and its lift-up pipe.

    Mortalities settle into the cone at the bottom of the net and are airlifted to the
    surface in the pipe. Both are real, both are things an ROV is sent to look at, and the
    pipe is the only structure that crosses the middle of the pen.
    """
    apex = defl(PEN_CONE_DEPTH)
    ts = np.linspace(0.0, 1.0, 5)
    for k in range(24):
        a0 = 2.0 * np.pi * k / 24
        a1 = 2.0 * np.pi * (k + 1) / 24
        for a in (a0,):
            pts = [np.array([(1.7 * (1.0 - t) + 0.28 * t) * np.cos(a),
                             (1.7 * (1.0 - t) + 0.28 * t) * np.sin(a),
                             PEN_CONE_DEPTH - 0.6 + 1.6 * t]) + apex for t in ts]
            tube(mesh, pts, 0.02, sides=4)
        del a1
    for rr, zz in ((1.7, PEN_CONE_DEPTH - 0.6), (0.95, PEN_CONE_DEPTH + 0.2),
                   (0.28, PEN_CONE_DEPTH + 1.0)):
        torus(mesh, (apex[0], apex[1], zz), rr, 0.025, 24, 4)

    # the airlift, from the cone up past the collar
    zs = np.linspace(PEN_CONE_DEPTH + 0.9, -0.4, 14)
    pts = [np.array([0.0, 0.0, z]) + defl(max(z, 0.0)) for z in zs]
    tube(mesh, pts, 0.055, sides=8, cap=True)


def build_barge(gra, phy):
    """A 22 x 12 m feed barge with eight silos, in its own local frame.

    It straddles the waterline by design: 2.5 m of draft and 2.0 m of freeboard. That is
    the scene's main test of the above-water rendering path, which is why it is worth
    having rather than a hull cut off at z = 0.
    """
    hz = 0.5 * (BARGE_DRAFT + BARGE_FREEBOARD)
    cz = 0.5 * (BARGE_DRAFT - BARGE_FREEBOARD)
    box(gra, (0.0, 0.0, cz), (0.5 * BARGE_L, 0.5 * BARGE_W, hz))
    box(gra, (-7.0, 0.0, -BARGE_FREEBOARD - 1.6), (3.0, 3.5, 1.6))          # deckhouse
    box(gra, (-7.0, 0.0, -BARGE_FREEBOARD - 3.6), (1.2, 1.6, 0.4))          # mast base
    for i, x in enumerate((-2.0, 1.5, 5.0, 8.5)):
        for y in (-3.2, 3.2):
            zs = np.linspace(-BARGE_FREEBOARD - 0.1, -BARGE_FREEBOARD - SILO_H, 2)
            tube(gra, [(x, y, z) for z in zs], SILO_R, sides=14, cap=True,
                 hint=(1.0, 0.0, 0.0))
        del i
    box(phy, (0.0, 0.0, cz), (0.5 * BARGE_L, 0.5 * BARGE_W, hz))


def catenary(a, b, sag, n=14):
    """A rope from a to b with `sag` metres of droop at its middle, as a point list."""
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    ts = np.linspace(0.0, 1.0, n)
    return [a + (b - a) * t + np.array([0.0, 0.0, sag * 4.0 * t * (1.0 - t)]) for t in ts]


def grid_nodes():
    return [(sx * GRID_X, y) for y in GRID_Y for sx in (-1.0, 1.0)]


def anchor_positions(depth_at):
    """Where each grid node's anchor sits, and how deep. Sampled from this run's own
    bathymetry, so the anchors cannot end up buried or hanging in mid-water."""
    out = []
    for nx, ny in grid_nodes():
        ox = 1.0 if nx > 0 else -1.0
        oy = 0.0
        if abs(ny) > 100.0:                       # the corner nodes pull outward in y too
            oy = 1.0 if ny > 0 else -1.0
        n = np.hypot(ox, oy)
        ax = nx + ANCHOR_REACH * ox / n
        ay = ny + ANCHOR_REACH * oy / n
        out.append(((nx, ny), (ax, ay), depth_at(ax, ay)))
    return out


def build_moorings(mesh, anchors, live):
    """The mooring frame, and every anchor line that is NOT a live soft-body cable.

    Four of the twelve lines are <cable> elements in fish_farm.scn: real Bullet soft bodies
    that hang in a catenary and lean with the current. The rest are this static mesh. A
    CableEntity is solved every physics step, so four short ones are a demonstration and
    forty are a stall - and the four live ones are the four the vehicle flies past.
    """
    nodes = grid_nodes()
    for sx in (-GRID_X, GRID_X):
        pts = catenary((sx, GRID_Y[0], GRID_Z), (sx, GRID_Y[-1], GRID_Z), 2.5, 22)
        tube(mesh, pts, ROPE_R, sides=6)
    for y in GRID_Y:
        pts = catenary((-GRID_X, y, GRID_Z), (GRID_X, y, GRID_Z), 1.2, 12)
        tube(mesh, pts, ROPE_R, sides=6)

    for (node, anchor, depth) in anchors:
        if node in live:
            continue
        pts = catenary((node[0], node[1], GRID_Z), (anchor[0], anchor[1], depth), 6.0, 16)
        tube(mesh, pts, ROPE_R, sides=6)
    del nodes


def build_feed_hose(mesh):
    """The floating feed hose from the barge to the hero pen, routed clear of the south pen."""
    pts = [(0.0, BARGE_XY[1] + 8.0, -0.10), (14.0, -128.0, -0.10), (26.0, -105.0, -0.10),
           (26.0, -60.0, -0.10), (24.0, -30.0, -0.10), (14.0, -10.0, -0.10),
           (0.0, -1.0, -0.35)]
    tube(mesh, pts, 0.045, sides=8, cap=True, hint=(0.0, 0.0, 1.0))


# ------------------------------------------------------------------------ the pen assembly
#
# THE PEN IS FOUR OBJECTS, NOT SIXTEEN PANELS. There is no mesh cache in OpenGLContent -
# BuildObject creates a new VAO and VBO every time - and DrawObjects walks the whole queue
# with no frustum culling, once for the main pass, once for the above-water redraw, once per
# shadow cascade and once per shadow-casting light. Sixteen instances of one panel would cost
# sixteen meshes, sixteen collision trees and sixteen draw calls per pass, and save nothing.
# A merged ring is also SMALLER on disk, because it shares vertices across the sector seams.
#
# What is split, and why:
#     fouled band   0 - 5 m     its own look, and thicker twine
#     clean band    5 - 15 m    all but one sector
#     torn sector   5 - 15 m    one sector, whose PHYSICS shell carries the same hole
#     cone         15 - 20 m    coarser: it is dark down there and rarely inspected
#
# The vertical strands of the clean band and of the torn sector are spaced independently, so
# the two meet at a slightly irregular pitch. That is what a real net does at a panel seam.
WALL_PITCH = 0.30
CONE_PITCH = 0.45
LOD_PITCH = 1.00
WALL_AZ_SEGMENTS = 64
TEAR_SECTOR_HALF = 0.19635      # rad, 11.25 degrees - one sixteenth of the circle


def build_net_assets(out_dir, defl):
    """Writes the six net meshes and returns a list of (name, tris, bytes) for the report."""
    report = []

    def emit(stem, gra, shell, what):
        b1 = gra.write(os.path.join(out_dir, stem + "_gra.obj"), what + " - twine")
        b2 = shell.write(os.path.join(out_dir, stem + "_phy.obj"), what + " - collision shell")
        report.append((stem, gra.n_tris(), shell.n_tris(), b1 + b2))

    # fouled band, whole circle
    gra, shell = ObjMesh(), ObjMesh()
    build_net_band(gra, shell, 0.0, FOUL_DEPTH, SN_FOULED, WALL_PITCH, 3,
                   WALL_AZ_SEGMENTS, defl)
    emit("net_wall_fouled", gra, shell, "fouled net band, 0-5 m")

    # clean band, whole circle less the torn sector
    az0 = TEAR_AZIMUTH + TEAR_SECTOR_HALF
    gra, shell = ObjMesh(), ObjMesh()
    build_net_band(gra, shell, FOUL_DEPTH, PEN_WALL_DEPTH, SN_CLEAN, WALL_PITCH, 6,
                   WALL_AZ_SEGMENTS, defl, az0=az0, az1=az0 + 2.0 * np.pi - 2.0 * TEAR_SECTOR_HALF)
    emit("net_wall_clean", gra, shell, "clean net band, 5-15 m")

    # the torn sector
    gra, shell = ObjMesh(), ObjMesh()
    build_net_band(gra, shell, FOUL_DEPTH, PEN_WALL_DEPTH, SN_CLEAN, WALL_PITCH, 6,
                   WALL_AZ_SEGMENTS, defl,
                   az0=TEAR_AZIMUTH - TEAR_SECTOR_HALF, az1=TEAR_AZIMUTH + TEAR_SECTOR_HALF,
                   tear=True)
    emit("net_wall_torn", gra, shell, "torn net sector, 5-15 m")

    # the cone
    gra, shell = ObjMesh(), ObjMesh()
    build_net_cone(gra, shell, SN_CLEAN, CONE_PITCH, 5, WALL_AZ_SEGMENTS, defl)
    emit("net_cone", gra, shell, "net cone, 15-20 m")

    # the neighbours, at a coarse pitch: they are 70 m away, which is far beyond the
    # visibility of this water, and they are scenery from the surface camera only
    gra, shell = ObjMesh(), ObjMesh()
    build_net_band(gra, shell, 0.0, PEN_WALL_DEPTH, SN_CLEAN, LOD_PITCH, 4, 40, defl)
    emit("net_lod_wall", gra, shell, "neighbour pen wall, coarse")

    gra, shell = ObjMesh(), ObjMesh()
    build_net_cone(gra, shell, SN_CLEAN, LOD_PITCH * 1.6, 4, 40, defl)
    emit("net_lod_cone", gra, shell, "neighbour pen cone, coarse")

    return report


def main():
    ap = argparse.ArgumentParser(description="Generate the salmon farm assets.")
    ap.add_argument("--resolution", type=float, default=1.5,
                    help="metres per bathymetry pixel (default 1.5). The scene is drawn "
                         "once per render pass with no frustum culling and the seabed is "
                         "over half the triangles in it, so this is the main performance "
                         "knob; the triangle count goes as its inverse square. The site "
                         "itself barely changes with it, but the printed <dimensions> do, "
                         "so paste them again after changing it.")
    ap.add_argument("--albedo-size", type=int, default=2048,
                    help="site albedo resolution (default 2048, i.e. 0.195 m per pixel)")
    ap.add_argument("--debug-marker", action="store_true",
                    help="stamp an L-shaped shallow notch in the world +x/+y corner, to "
                         "settle the image-row to world-y direction")
    ap.add_argument("--no-textures", action="store_true")
    ap.add_argument("--no-geometry", action="store_true")
    ap.add_argument("--out", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "farm"))
    args = ap.parse_args()

    out_dir = os.path.abspath(args.out)
    os.makedirs(out_dir, exist_ok=True)

    # ---------------------------------------------------------------------- the bathymetry
    d = build_depth_field(args.resolution, args.debug_marker)
    origin_z, height = write_bathymetry(os.path.join(out_dir, "bathymetry_16b.png"), d)
    scale = 2.0 * SITE_HALF / (d.shape[0] - 1)
    print("%-26s %4d x %-4d px at %.4f m   depth %.2f - %.2f m"
          % ("bathymetry_16b.png", d.shape[1], d.shape[0], scale, d.min(), d.max()))

    if not args.no_textures:
        write_seabed_albedo(os.path.join(out_dir, "seabed_albedo.png"), args.albedo_size)
        print("%-26s %4d x %-4d px at %.4f m   site scale, uv_scale MUST be 1.0"
              % ("seabed_albedo.png", args.albedo_size, args.albedo_size,
                 2.0 * SITE_HALF / args.albedo_size))
        write_salmon_texture(os.path.join(out_dir, "salmon.png"))
        print("%-26s  512 x 256  px   counter-shaded flank" % "salmon.png")

    # ------------------------------------------------------------ the net and the structure
    net_report = []
    defl, delta_max, retention = net_deflection()

    def depth_at(x, y):
        """The generated bathymetry, sampled at a world point. Used to sit the anchors on
        the seabed rather than guessing at where it is."""
        i = int(round(np.clip((SITE_HALF - y) / scale, 0, d.shape[0] - 1)))
        j = int(round(np.clip((x + SITE_HALF) / scale, 0, d.shape[1] - 1)))
        return float(d[i, j])

    anchors = anchor_positions(depth_at)
    live_nodes = [n for (n, _, _) in anchors if abs(n[1]) < 60.0]

    if not args.no_geometry:
        net_report = build_net_assets(out_dir, defl)

        def emit1(stem, mesh, what, phy=None):
            b = mesh.write(os.path.join(out_dir, stem + ".obj"), what)
            pt = 0
            if phy is not None:
                b += phy.write(os.path.join(out_dir, stem + "_phy.obj"), what + " - collision")
                pt = phy.n_tris()
            net_report.append((stem, mesh.n_tris(), pt, b))

        gra, phy = ObjMesh(), ObjMesh()
        build_collar(gra, phy)
        emit1("collar", gra, "pen flotation collar", phy)

        m = ObjMesh(); build_sinker_tube(m, defl); emit1("sinker_tube", m, "pen bottom ring")
        m = ObjMesh(); build_pen_ropes(m, defl); emit1("pen_ropes", m, "lifting straps and sensor string")
        m = ObjMesh(); build_mortality(m, defl); emit1("mortality", m, "dead-fish cone and lift-up")

        gra, phy = ObjMesh(), ObjMesh()
        build_barge(gra, phy)
        emit1("barge", gra, "feed barge", phy)

        m = ObjMesh(); build_moorings(m, anchors, live_nodes); emit1("moorings", m, "mooring frame and dead anchor lines")
        m = ObjMesh(); build_feed_hose(m); emit1("feed_hose", m, "floating feed hose")

        fish = ObjMesh(textured=True)
        build_salmon(fish)
        nbytes = fish.write(os.path.join(out_dir, "salmon.obj"), "Atlantic salmon, 4 kg")
        net_report.append(("salmon", fish.n_tris(), 0, nbytes))

    # -------------------------------------------------------------------------- the report
    print()
    print("Paste into fish_farm.scn:")
    print('    <dimensions scalex="%.6g" scaley="%.6g" height="%.6g"/>' % (scale, scale, height))
    print('    <world_transform xyz="0.0 0.0 %.4f" rpy="0.0 0.0 0.0"/>' % origin_z)
    print()

    print("THE NET")
    print("  mesh pitch          %.2f m, against a real bar length of %.3f m" % (WALL_PITCH, NET_BAR))
    print("  solidity  clean     %.4f  ->  twine %.1f mm at this pitch"
          % (SN_CLEAN, 1000.0 * WALL_PITCH * (1.0 - np.sqrt(1.0 - SN_CLEAN))))
    print("            fouled    %.4f  ->  twine %.1f mm"
          % (SN_FOULED, 1000.0 * WALL_PITCH * (1.0 - np.sqrt(1.0 - SN_FOULED))))
    print("  baked current       %.2f m/s along (%.2f, %.2f)"
          % (np.hypot(*CURRENT), CURRENT[0], CURRENT[1]))
    print("  sinker tube         %.0f kg submerged, %.0f N/m of circumference"
          % (SINKER_MASS, SINKER_MASS * G / (2.0 * np.pi * PEN_RADIUS)))
    print("  bottom ring offset  %.2f m downstream  (%.0f %% of the net depth)"
          % (delta_max, 100.0 * delta_max / PEN_WALL_DEPTH))
    print("  volume retention    %.1f %%   (translation only - this model does not narrow "
          "the net)" % (100.0 * retention))
    print("  the tear            azimuth %.0f deg, z = %.2f - %.2f m, %.2f x %.2f m"
          % (np.degrees(TEAR_AZIMUTH), TEAR_Z0, TEAR_Z1,
             2.0 * TEAR_HALF_AZ * PEN_RADIUS, TEAR_Z1 - TEAR_Z0))
    print()

    if net_report:
        print("%-20s %10s %10s %12s" % ("mesh", "gra tris", "phy tris", "bytes"))
        tot_g = tot_p = tot_b = 0
        for name, g, p, b in net_report:
            print("%-20s %10d %10d %12d" % (name, g, p, b))
            tot_g += g
            tot_p += p
            tot_b += b
        print("%-20s %10d %10d %12d" % ("TOTAL", tot_g, tot_p, tot_b))
        print()

    print("seabed triangles:   %d" % (2 * (d.shape[0] - 1) * (d.shape[1] - 1)))
    print()
    for name, (cx, cy) in zip(("south", "hero", "north"), PEN_CENTRES):
        i = int(round((SITE_HALF - cy) / scale))
        j = int(round((cx + SITE_HALF) / scale))
        print("  water depth under the %-5s pen centre (%+.0f, %+.0f):  %.2f m"
              % (name, cx, cy, d[i, j]))
    i0 = int(round(SITE_HALF / scale))
    print("  net bottom is at %.1f m, so clearance under the hero pen is %.2f m"
          % (PEN_CONE_DEPTH, d[i0, i0] - PEN_CONE_DEPTH))

    print()
    print("THE MOORING - paste these into fish_farm.scn, they follow this run's bathymetry")
    for k, ((nx, ny), (ax, ay), dep) in enumerate(anchors):
        print('	<static name="Anchor%d" type="box">' % k)
        print('		<dimensions xyz="2.6 2.6 1.2"/>')
        print('		<material name="Steel"/><look name="steel_paint" uv_mode="2"/>')
        print('		<world_transform xyz="%.1f %.1f %.2f" rpy="0.0 0.0 0.0"/>' % (ax, ay, dep - 0.6))
        print('	</static>')
    print()
    for k, ((nx, ny), (ax, ay), dep) in enumerate(anchors):
        if (nx, ny) not in live_nodes:
            continue
        print('	<cable name="AnchorLine%d" physics="submerged" buoyant="true">' % k)
        print('		<geometry diameter="0.048" number_of_segments="40"/>')
        print('		<material name="MooringRope" stretch_factor="0.02"/>')
        print('		<look name="rope" uv_scale="20.0"/>')
        print('		<first_end position="%.1f %.1f %.2f" anchor="world"/>' % (ax, ay, dep - 0.4))
        print('		<second_end position="%.1f %.1f %.2f" anchor="world"/>' % (nx, ny, GRID_Z))
        print('	</cable>')
    print()
    for k, (nx, ny) in enumerate(grid_nodes()):
        print('	<dynamic name="Buoy%d" type="sphere" physics="floating" buoyant="true">' % k)
        print('		<dimensions radius="0.70"/>')
        print('		<origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>')
        print('		<material name="BuoyFoam"/><look name="hdpe_orange"/>')
        print('		<world_transform xyz="%.1f %.1f -0.35" rpy="0.0 0.0 0.0"/>' % (nx, ny))
        print('	</dynamic>')
        print('	<cable name="Pennant%d" physics="submerged" buoyant="false">' % k)
        print('		<geometry diameter="0.040" number_of_segments="8"/>')
        print('		<material name="MooringRope" stretch_factor="0.02"/>')
        print('		<look name="rope" uv_scale="6.0"/>')
        print('		<first_end position="%.1f %.1f %.2f" anchor="world"/>' % (nx, ny, GRID_Z))
        print('		<second_end position="%.1f %.1f -0.35" anchor="dynamic">' % (nx, ny))
        print('			<body name="Buoy%d"/>' % k)
        print('		</second_end>')
        print('	</cable>')


if __name__ == "__main__":
    main()
