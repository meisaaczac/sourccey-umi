"""Parametric CAD for the Sourccey UMI (CadQuery): the trigger lever and the pistol grip.
Frame: origin = axis of the Sourccey finger pivot, x = jaw tips, y = pivot axis, z = up. Units: mm.
  python cad/umi_cad.py      # writes cad/step/*.step and cad/stl/*.stl (LEFT and RIGHT)
"""
import cadquery as cq, math, os
from pathlib import Path
HERE=Path(__file__).resolve().parent
def box(x0,x1,y0,y1,z0,z1): return cq.Workplane("XY").box(x1-x0,y1-y0,z1-z0,centered=False).translate((x0,y0,z0))
def cz(x,y,z0,z1,d): return cq.Workplane("XY").workplane(offset=z0).center(x,y).circle(d/2).extrude(z1-z0)
def cx(y,z,x0,x1,d): return cq.Workplane("YZ").workplane(offset=x0).center(y,z).circle(d/2).extrude(x1-x0)
def cy(x,z,y0,y1,d): return cq.Workplane("XZ").workplane(offset=-y1).center(x,z).circle(d/2).extrude(y1-y0)
def hexz(x,y,z0,z1,af): return cq.Workplane("XY").workplane(offset=z0).center(x,y).polygon(6,af/math.cos(math.pi/6)).extrude(z1-z0)
def arm(p,q,w,y0,y1):
    L=math.hypot(q[0]-p[0],q[1]-p[1]); ang=math.degrees(math.atan2(q[1]-p[1],q[0]-p[0])); mid=((p[0]+q[0])/2,(p[1]+q[1])/2)
    return cq.Workplane("XZ").workplane(offset=-y1).center(*mid).slot2D(L+w,w,ang).extrude(y1-y0)

Y0,Y1=-38.3,-32.35                    # lever plate, bolted on the outer face of the -y finger lug (outer face at y=-32.3)
XF=-30.4; AX=(-0.4,-15.0)
HOLES=[(-0.4,-15.0),(-0.4,-7.9),(-0.4,-22.1),(6.7,-15.0),(-7.4,-15.0)]
PAD_LOW=(-8.0,-41.0); TAIL=(-16.0,24.0)
STOP_X=-39.0                          # tuned below

def make_lever():
    l=cy(0,0,Y0,Y1,32)
    l=l.union(arm((0,0),PAD_LOW,14,Y0,Y1)).union(arm((0,0),TAIL,13,Y0,Y1))
    pad=box(-19,-2,-46,-26,-53,-30)                      # trigger pad block (index finger)
    pad=pad.cut(cy(10,-41.5,-47,-25,30))                 # concave front where the fingertip sits
    l=l.union(pad)
    l=l.cut(box(-21,0,-43.5,-40.5,-50.5,-34.5)).cut(box(-21,0,-31.5,-28.5,-50.5,-34.5))
    for dx,dz in ((7.1,0),(-7.1,0),(0,7.1),(0,-7.1)): l=l.cut(cy(dx,dz,Y0-1,Y1+1,3.4))   # same cross pattern as the finger horn
    l=l.union(box(TAIL[0]-7,TAIL[0]+7,-44.5,Y0+0.3,TAIL[1]-9.5,TAIL[1]+7))   # open-limit tab (lands on top of the grip beam)
    l=l.cut(cy(TAIL[0],TAIL[1],Y0-0.2,Y1+1,3.2))        # spring hook hole (only through the plate, not the tab)
    return l

def make_grip(stop_x=STOP_X):
    g=box(XF-6.0,XF,-12,11.2,-27,-3)                     # interface plate on the base rear face
    g=g.union(box(-66,XF-0.5,-52,12,-34,-8))             # beam to the offset grip
    grip=box(-64,-28,-52,-22,-135,-30)
    try: grip=grip.edges("|Z").fillet(9).faces("<Z").edges().fillet(7)
    except Exception: pass
    g=g.union(grip)
    for z in (-70,-90,-110):                              # finger grooves on the grip front
        g=g.cut(cy(-31,z,-48,-26,14))
    for (y,z) in HOLES: g=g.cut(cx(y,z,XF-6.0,XF+0.3,4.2))    # M3 heat-set inserts
    g=g.cut(box(-47,-33,-38.1,-32.5,-30,-7.5))           # channel where the tension spring runs
    g=g.cut(cy(-42,-26,-53,13,3.2))                      # anchor pin (3 mm rod/screw) for the spring end
    ped=box(-60,-42,-20,6,-8,34).cut(cz(-51,-7,13.6,35,6.9)).cut(hexz(-51,-7,6,13.6,11.4)).cut(box(-61,-51,-12.8,-1.2,6,13.6))
    g=g.union(ped)
    g=g.cut(cx(-37,-82,-64,-24,3.4))                      # lanyard / cord hole through the grip
    return g


def build():
    import trimesh, sys
    sys.path.insert(0, str(HERE.parent / "tools"))
    from common import orient, mirror_y
    (HERE/"stl").mkdir(exist_ok=True); (HERE/"step").mkdir(exist_ok=True)
    L, G = make_lever(), make_grip()
    cq.exporters.export(L, str(HERE/"step"/"trigger_lever.step")); cq.exporters.export(G, str(HERE/"step"/"pistol_grip.step"))
    for n, w in (("trigger_lever", L), ("pistol_grip", G)):
        cq.exporters.export(w, f"/tmp/_{n}.stl", tolerance=0.02, angularTolerance=0.08); m = trimesh.load(f"/tmp/_{n}.stl")
        assert m.is_watertight, n
        for side, mm in (("LEFT", m), ("RIGHT", mirror_y(m, AX[0]))):
            o, over, bed = orient(mm); o.export(HERE/"stl"/f"{n}_{side}.stl"); print(f"{n}_{side}.stl  {(o.bounds[1]-o.bounds[0]).round(0).tolist()} mm")

if __name__ == "__main__":
    build()
