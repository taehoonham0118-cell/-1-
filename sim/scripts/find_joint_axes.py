# week04 | STEP 에서 관절 축 · 관절점 · 구동기 장착 부품 찾기 (함태훈, B1) — 링크 분할표 · joint origin 검증표의 근거
# 실행: python sim/scripts/find_joint_axes.py --step <양팔로봇 최종버전.stp> --out sim/robot/joint_axes_cad.csv
# 방법
#   1) 최상위 인스턴스 「RI85 제작건:n」 마다 output-v1 의 가장 큰 원통면(R 52 mm) 축 = 관절 회전축 (방향 · 축 위 한 점, 조립 좌표 mm)
#   2) 출력부(output-v1) · 고정부(housing2/3) 무게중심을 축에 투영 → 출력 쪽 방향
#   3) 출력부 · 고정부와 맞닿은(거리 ≤ 0.5 mm) 최상위 부품 = 자식 · 부모 링크 후보 (바운딩 박스로 먼저 거른 뒤 정확 거리)
import argparse, collections, csv, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from step_to_links import read_step, solids
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.BRep import BRep_Builder
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
TO_FACE = getattr(TopoDS, "Face_s", None) or TopoDS.Face


def comp(shapes):
    b = BRep_Builder(); c = TopoDS_Compound(); b.MakeCompound(c)
    for s in shapes: b.Add(c, s)
    return c


def box(sh):
    b = Bnd_Box(); BRepBndLib.Add_s(sh, b); return np.array(b.Get())


def near_box(b1, b2, tol):
    return all(b1[i] - tol <= b2[i + 3] and b2[i] - tol <= b1[i + 3] for i in range(3))


def big_cylinder(shape):
    best = None; ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        s = BRepAdaptor_Surface(TO_FACE(ex.Current()))
        if s.GetType() == GeomAbs_Cylinder:
            c = s.Cylinder(); a = c.Axis()
            if best is None or c.Radius() > best[0]:
                best = (c.Radius(), np.array([a.Location().X(), a.Location().Y(), a.Location().Z()]),
                        np.array([a.Direction().X(), a.Direction().Y(), a.Direction().Z()]))
        ex.Next()
    return best


def com(shapes):
    g = GProp_GProps()
    for sh in shapes:
        for s in solids(sh):
            gg = GProp_GProps(); BRepGProp.VolumeProperties_s(s, gg); g.Add(gg)
    c = g.CentreOfMass(); return np.array([c.X(), c.Y(), c.Z()])


ap = argparse.ArgumentParser(); ap.add_argument("--step", required=True); ap.add_argument("--out", default="sim/robot/joint_axes_cad.csv")
ap.add_argument("--touch_mm", type=float, default=0.5)
a = ap.parse_args()
parts = read_step(a.step)
top = collections.defaultdict(list)
for p in parts:
    top[p["path"].split("/")[1]].append(p["shape"])
others = {k: comp(v) for k, v in top.items() if not k.startswith("RI85")}
obox = {k: box(v) for k, v in others.items()}
rows = sorted((k for k in top if k.startswith("RI85")), key=lambda s: int(s.split(":")[1]))
out = []
for k in rows:
    sub = collections.defaultdict(list)
    for p in parts:
        seg = p["path"].split("/")
        if seg[1] == k: sub[seg[2].split(":")[0]].append(p["shape"])
    r, pt, d = big_cylinder(comp(sub["output-v1"]))
    oc, hc = com(sub["output-v1"]), com(sub["housing2-v1"] + sub["housing3-v1"])
    if np.dot(oc - hc, d) < 0: d = -d                     # 축 방향 = 고정부 → 출력부 쪽
    touch = {}
    for nm, grp in (("output", sub["output-v1"]), ("housing", sub["housing2-v1"] + sub["housing3-v1"])):
        g = comp(grp); gb = box(g); hits = []
        for ok, osh in others.items():
            if not near_box(gb, obox[ok], a.touch_mm): continue
            e = BRepExtrema_DistShapeShape(g, osh)
            if e.IsDone() and e.Value() <= a.touch_mm: hits.append(ok)
        touch[nm] = ";".join(sorted(hits))
    out.append(dict(actuator=k, axis_x=round(d[0], 4), axis_y=round(d[1], 4), axis_z=round(d[2], 4),
                    pt_x_mm=round(pt[0], 2), pt_y_mm=round(pt[1], 2), pt_z_mm=round(pt[2], 2), radius_mm=round(r, 1),
                    tilt_deg=round(float(np.degrees(np.arccos(min(1, np.max(np.abs(d)))))), 2),
                    output_touch=touch["output"], housing_touch=touch["housing"]))
    print(out[-1])
os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
with open(a.out, "w", newline="", encoding="utf-8-sig") as fp:
    w = csv.DictWriter(fp, fieldnames=list(out[0])); w.writeheader(); w.writerows(out)
print("저장:", a.out)
