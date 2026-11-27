"""Parametric 3D-printable enclosure STLs for the Sourccey UMI Portable Pack (Mode B).

Generates 100% watertight meshes using shapely + trimesh + manifold3d:
  - cad/stl/portable_pack_case.stl : Belt/wrist enclosure for Pi Zero 2W + 2S LiPo + 5V UBEC + Bus Adapter
  - cad/stl/portable_pack_lid.stl  : Matching vented lid with 4x M3 corner screw holes

Note: The 20x20 mm wrist camera mounts directly onto the Sourccey gripper base's built-in
+y wing (Ø25 mm opening + 4 corner holes), so no separate camera adapter STL is needed.

Usage:
  python cad/accessories_cad.py
"""
from pathlib import Path
import numpy as np
import shapely.geometry as sg
import trimesh

OUT = Path(__file__).resolve().parent / "stl"


def _rounded_rect(w: float, h: float, r: float) -> sg.Polygon:
    return sg.box(-w / 2 + r, -h / 2 + r, w / 2 - r, h / 2 - r).buffer(r, resolution=16)


def _circle(x: float, y: float, d: float) -> sg.Polygon:
    return sg.Point(x, y).buffer(d / 2.0, resolution=24)


def _extrude_stacked(layers: list[tuple[sg.base.BaseGeometry, float, float]]) -> trimesh.Trimesh:
    """Extrude watertight stacked layers (poly/multipoly, z0, z1) and union with manifold."""
    parts = []
    for idx, (geom, z0, z1) in enumerate(layers):
        polys = list(geom.geoms) if isinstance(geom, sg.MultiPolygon) else [geom]
        # Tiny 0.05 mm overlap with layer below ensures manifold boolean union fuses cleanly
        z_start = z0 - 0.05 if idx > 0 else z0
        for p in polys:
            m = trimesh.creation.extrude_polygon(p, height=z1 - z_start)
            m.apply_translation([0.0, 0.0, z_start])
            parts.append(m)
    if len(parts) == 1:
        out = parts[0]
    else:
        out = trimesh.boolean.union(parts, engine="manifold")
    out.merge_vertices()
    trimesh.repair.fix_normals(out)
    return out


def make_portable_pack_case() -> trimesh.Trimesh:
    """Compact belt/wrist enclosure (144 x 104 x 30 mm) for Pi Zero 2W + 2S LiPo + 5V UBEC + Bus Adapter."""
    W, H, WALL, FLOOR, TOTAL_Z = 144.0, 104.0, 2.6, 2.6, 30.0
    outer = _rounded_rect(W, H, 6.0)
    inner = _rounded_rect(W - 2 * WALL, H - 2 * WALL, 4.0)

    # Floor with 2x velcro belt-strap slots (26 x 4 mm) so it straps right onto a belt or forearm
    floor = outer
    for x_slot in (-44.0, 44.0):
        floor = floor.difference(sg.box(x_slot - 2.2, -14.0, x_slot + 2.2, 14.0))

    # Perimeter walls + 4 corner M3 screw bosses (Ø7.5 mm boss with Ø3.2 mm hole)
    wall_ring = outer.difference(inner)
    boss_xy = [(-W / 2 + 6.5, -H / 2 + 6.5), (W / 2 - 6.5, -H / 2 + 6.5),
               (-W / 2 + 6.5, H / 2 - 6.5), (W / 2 - 6.5, H / 2 - 6.5)]
    for bx, by in boss_xy:
        wall_ring = wall_ring.union(_circle(bx, by, 7.6))
    for bx, by in boss_xy:
        wall_ring = wall_ring.difference(_circle(bx, by, 3.2))

    # Internal divider wall separating the 2S LiPo bay (left 64 mm) from electronics bay (right)
    divider = sg.box(-8.0, -H / 2 + WALL, -5.8, H / 2 - WALL - 16.0)
    lower_walls = wall_ring.union(divider)

    # Upper walls (z = 16..30 mm) with clean U-cutouts for harness exit (left/front) and power switch (bottom-left)
    upper_walls = wall_ring
    upper_walls = upper_walls.difference(sg.box(-W / 2 - 2, -10.0, -W / 2 + WALL + 2, 10.0))  # cable harness exit
    upper_walls = upper_walls.difference(sg.box(-52.0, -H / 2 - 2, -36.0, -H / 2 + WALL + 2)) # power switch cutout

    return _extrude_stacked([
        (floor, 0.0, FLOOR),
        (lower_walls, FLOOR, 16.0),
        (upper_walls, 16.0, TOTAL_Z),
    ])


def make_portable_pack_lid() -> trimesh.Trimesh:
    """Matching vented lid (144 x 104 x 2.6 mm) with 4x M3.4 corner clearance holes + Pi Zero 2W airflow slots."""
    W, H = 144.0, 104.0
    lid = _rounded_rect(W, H, 6.0)
    # 4 corner M3.4 screw holes
    for bx, by in [(-W / 2 + 6.5, -H / 2 + 6.5), (W / 2 - 6.5, -H / 2 + 6.5),
                   (-W / 2 + 6.5, H / 2 - 6.5), (W / 2 - 6.5, H / 2 - 6.5)]:
        lid = lid.difference(_circle(bx, by, 3.4))
    # Cooling vent slots over the Raspberry Pi Zero 2W & UBEC side (right half)
    for vy in (-24.0, -12.0, 0.0, 12.0, 24.0):
        lid = lid.difference(sg.box(12.0, vy - 2.0, 52.0, vy + 2.0))
    return _extrude_stacked([(lid, 0.0, 2.6)])


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    builders = [
        ("portable_pack_case.stl", make_portable_pack_case),
        ("portable_pack_lid.stl", make_portable_pack_lid),
    ]
    for name, fn in builders:
        m = fn()
        p = OUT / name
        m.export(p)
        print(f"{name:26s} | faces={len(m.faces):5d} | watertight={m.is_watertight} | extents={np.round(m.extents, 1).tolist()} mm")


if __name__ == "__main__":
    main()
