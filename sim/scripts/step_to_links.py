# week04 | STEP → 링크별 visual/collision STL + 질량 · COM · 관성 → URDF <inertial> (함태훈, B1)
#
# 실행 (저장소 루트에서):
#   1) 부품 목록만 뽑기 (월: 링크 분할표 작성용)
#      python sim/scripts/step_to_links.py --step <STEP 경로> --inventory sim/out/parts.csv
#   2) 링크 분할표(JSON)대로 변환
#      python sim/scripts/step_to_links.py --step <STEP 경로> --linkmap sim/linkmap.json --out sim/out
#
# 필요: pip install cadquery-ocp trimesh numpy scipy   (OpenCascade = FreeCAD 가 내부에서 쓰는 CAD 엔진)
#       cadquery-ocp 7.8.1.1(클라우드) · 8.0.1.1(노트북 Python 3.14) 둘 다 확인
#
# 단위 규칙
#   STEP(우리 로봇) = mm.  출력 STL · URDF = m (× 0.001).  밀도 = kg/m³ (알루미늄 2700).
#   관성은 「COM 기준, 링크 프레임 축」 으로 낸다 = URDF <inertial> 이 요구하는 기준
#   (<origin xyz> = 링크 원점 → COM, <inertia> = COM 에서 잰 텐서, rpy 0 이면 링크 축 방향).
#
# 링크 분할표(JSON) 형식 — 링크 = 서로 상대 운동이 없는 부품 묶음
#   {"density_default": 2700,
#    "links": [{"name": "left_upper_arm_link",
#               "parts": ["정규식1", "정규식2"],        # 부품 경로(조립품/부품 이름)에 맞출 정규식
#               "origin_mm": [x, y, z],                 # 링크 프레임 원점 = 부모 관절 축 위 점 (조립 좌표, mm)
#               "rpy": [r, p, y],                       # 링크 프레임 자세 (조립 좌표 기준, rad). 보통 0
#               "density": 2700,                        # 선택. 부품별로 다르면 "density_by_part": {"정규식": 밀도}
#               "mesh_exclude": ["정규식"],              # 선택. 질량에는 넣고 visual 메시에서는 뺄 부품(구동기 내부 기어 · 베어링 등)
#               "bom_mass_kg": null}]}                  # 선택. BOM 질량 (3자 비교용)
import argparse, csv, json, math, os, re, sys
import numpy as np

from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
from OCP.TDF import TDF_Label
try:
    from OCP.TDF import TDF_LabelSequence                                   # cadquery-ocp 7.x
except ImportError:
    from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence     # cadquery-ocp 8.x (노트북 Python 3.14 용 wheel)
from OCP.TDataStd import TDataStd_Name
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopLoc import TopLoc_Location
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCP.TopoDS import TopoDS
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.gp import gp_Trsf, gp_Vec, gp_Quaternion, gp_EulerSequence
import trimesh

MM = 1e-3
TO_SOLID = getattr(TopoDS, "Solid_s", None) or TopoDS.Solid   # OCP 7.x: Solid_s, 8.x: Solid
TO_FACE = getattr(TopoDS, "Face_s", None) or TopoDS.Face


# ---------------------------------------------------------------- STEP 읽기 (조립 트리 + 이름 + 위치)
def read_step(path):
    doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
    r = STEPCAFControl_Reader()
    r.SetNameMode(True)
    if r.ReadFile(path) != IFSelect_RetDone:
        sys.exit(f"STEP 읽기 실패: {path}")
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
    parts = []

    def name_of(lab):
        n = TDataStd_Name()
        return n.Get().ToExtString() if lab.FindAttribute(TDataStd_Name.GetID_s(), n) else "?"

    def walk(lab, loc, path):
        if XCAFDoc_ShapeTool.IsReference_s(lab):                     # 인스턴스 → 원본 부품
            ref = TDF_Label(); XCAFDoc_ShapeTool.GetReferredShape_s(lab, ref)
            loc = loc.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(lab))
            path = path + [name_of(lab)]
            lab = ref
        if XCAFDoc_ShapeTool.IsAssembly_s(lab):
            comps = TDF_LabelSequence(); XCAFDoc_ShapeTool.GetComponents_s(lab, comps, False)
            for i in range(1, comps.Length() + 1):
                walk(comps.Value(i), loc, path + ([name_of(lab)] if not path else []))
        else:
            shp = XCAFDoc_ShapeTool.GetShape_s(lab).Located(loc)
            parts.append({"path": "/".join(path + [name_of(lab)]) if not path or path[-1] != name_of(lab) else "/".join(path),
                          "product": name_of(lab), "shape": shp})

    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    return parts


def solids(shape):
    ex = TopExp_Explorer(shape, TopAbs_SOLID); out = []
    while ex.More():
        out.append(TO_SOLID(ex.Current())); ex.Next()
    return out


def bbox_mm(shape):
    b = Bnd_Box(); BRepBndLib.Add_s(shape, b)
    x0, y0, z0, x1, y1, z1 = b.Get()
    return [x1 - x0, y1 - y0, z1 - z0]


# ---------------------------------------------------------------- 좌표 변환 (조립 → 링크 프레임)
def link_trsf(origin_mm, rpy):
    """조립 좌표 → 링크 좌표 변환. 링크 프레임 = origin_mm 에 rpy(고정축 X→Y→Z, URDF 규약) 자세."""
    t = gp_Trsf()
    q = gp_Quaternion(); q.SetEulerAngles(gp_EulerSequence.gp_Extrinsic_XYZ, *rpy)
    t.SetTransformation(q, gp_Vec(*origin_mm))   # 링크 → 조립
    return t.Inverted()                           # 조립 → 링크


def moved(shape, trsf):
    return BRepBuilderAPI_Transform(shape, trsf, True).Shape()


# ---------------------------------------------------------------- 질량 특성 (OpenCascade, B-rep 정확 적분)
def mass_props(shapes_with_density):
    """[(shape, 밀도 kg/m³)] → 질량 kg, COM m, COM 기준 관성 kg·m² (입력 shape 좌표축)."""
    total = GProp_GProps()
    vol_mm3 = 0.0
    for shp, rho in shapes_with_density:
        for s in solids(shp):
            g = GProp_GProps(); BRepGProp.VolumeProperties_s(s, g)
            vol_mm3 += g.Mass()
            total.Add(g, rho * 1e-9)          # 밀도 kg/mm³ 를 가중치로 → Mass() 가 kg
    m = total.Mass()
    c = total.CentreOfMass()
    M = total.MatrixOfInertia()               # COM 기준 [kg·mm²]
    I = np.array([[M.Value(i, j) for j in (1, 2, 3)] for i in (1, 2, 3)]) * MM * MM
    return m, np.array([c.X(), c.Y(), c.Z()]) * MM, I, vol_mm3


# ---------------------------------------------------------------- 메시 (visual · collision)
def tessellate(shapes, lin_defl_mm=0.05, ang_defl=0.1):
    V, F, off = [], [], 0
    for shp in shapes:
        BRepMesh_IncrementalMesh(shp, lin_defl_mm, False, ang_defl, True)
        ex = TopExp_Explorer(shp, TopAbs_FACE)
        while ex.More():
            face = TO_FACE(ex.Current()); loc = TopLoc_Location()
            tri = BRep_Tool.Triangulation_s(face, loc)
            if tri is not None:
                tr = loc.Transformation()
                for i in range(1, tri.NbNodes() + 1):
                    p = tri.Node(i).Transformed(tr); V.append((p.X(), p.Y(), p.Z()))
                rev = face.Orientation() == 1   # TopAbs_REVERSED
                for i in range(1, tri.NbTriangles() + 1):
                    a, b, c = tri.Triangle(i).Get()
                    F.append((off + a - 1, off + c - 1, off + b - 1) if rev else (off + a - 1, off + b - 1, off + c - 1))
                off += tri.NbNodes()
            ex.Next()
    mesh = trimesh.Trimesh(np.array(V) * MM, np.array(F), process=True)   # mm → m
    mesh.merge_vertices()
    return mesh


def collision_mesh(visual, max_faces=200):
    hull = visual.convex_hull
    if len(hull.faces) > max_faces:
        try:
            hull = hull.simplify_quadric_decimation(face_count=max_faces).convex_hull
        except Exception:
            pass   # fast_simplification 미설치 시 hull 그대로 (MuJoCo 는 어차피 convex hull 로 충돌)
    return hull


def mesh_props(mesh, rho):
    m = mesh.copy(); m.density = rho
    return m.mass, m.center_mass, m.moment_inertia   # moment_inertia = COM 기준, 메시 좌표축


# ---------------------------------------------------------------- URDF <inertial>
def inertial_xml(name, m, c, I, src):
    f = lambda v: "0" if abs(v) < 1e-12 else f"{v:.6g}"   # 1e-12 아래 수치 잡음은 0
    return (f'  <!-- {name}: {src} -->\n'
            f'  <inertial>\n'
            f'    <origin xyz="{f(c[0])} {f(c[1])} {f(c[2])}" rpy="0 0 0"/>\n'
            f'    <mass value="{f(m)}"/>\n'
            f'    <inertia ixx="{f(I[0,0])}" iyy="{f(I[1,1])}" izz="{f(I[2,2])}" '
            f'ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" iyz="{f(I[1,2])}"/>\n'
            f'  </inertial>\n')


def triangle_ok(I):
    w = np.linalg.eigvalsh(I)
    return bool(w[0] + w[1] >= w[2] * (1 - 1e-9) and w.min() > 0)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--step", required=True)
    ap.add_argument("--inventory", help="부품 목록 CSV 경로 (월: 링크 분할표용)")
    ap.add_argument("--linkmap", help="링크 분할표 JSON")
    ap.add_argument("--out", default="sim/out")
    ap.add_argument("--lin_defl_mm", type=float, default=0.05, help="visual 메시 현 높이 허용 오차 [mm]")
    ap.add_argument("--ang_defl", type=float, default=0.1, help="visual 메시 각도 허용 오차 [rad]")
    ap.add_argument("--hull_faces", type=int, default=200)
    a = ap.parse_args()

    parts = read_step(a.step)
    print(f"STEP 부품 인스턴스 {len(parts)}개")

    if a.inventory:
        os.makedirs(os.path.dirname(a.inventory) or ".", exist_ok=True)
        with open(a.inventory, "w", newline="", encoding="utf-8-sig") as fp:
            w = csv.writer(fp); w.writerow(["path", "product", "n_solids", "volume_cm3", "mass_al_kg", "bbox_mm"])
            for p in parts:
                m, c, I, v = mass_props([(p["shape"], 2700)])
                w.writerow([p["path"], p["product"], len(solids(p["shape"])), round(v / 1000, 3), round(m, 4),
                            "x".join(f"{d:.1f}" for d in bbox_mm(p["shape"]))])
        print("저장:", a.inventory)
    if not a.linkmap:
        return

    lm = json.load(open(a.linkmap, encoding="utf-8"))
    rho0 = lm.get("density_default", 2700)
    os.makedirs(os.path.join(a.out, "meshes"), exist_ok=True)
    used = {}
    rows, xml = [], []
    for L in lm["links"]:
        pats = [re.compile(p) for p in L["parts"]]
        mine = [p for p in parts if any(r.search(p["path"]) for r in pats)]
        for p in mine:
            used.setdefault(p["path"], []).append(L["name"])
        if not mine:
            print(f"[경고] {L['name']}: 맞는 부품 없음"); continue
        T = link_trsf(L.get("origin_mm", [0, 0, 0]), L.get("rpy", [0, 0, 0]))
        dens = []
        for p in mine:
            rho = L.get("density", rho0)
            for pat, r in L.get("density_by_part", {}).items():
                if re.search(pat, p["path"]): rho = r
            dens.append((moved(p["shape"], T), rho))
        m, c, I, vol = mass_props(dens)
        excl = [re.compile(x) for x in L.get("mesh_exclude", [])]
        keep = [d for d, p in zip(dens, mine) if not any(r.search(p["path"]) for r in excl)]
        vis = tessellate([s for s, _ in keep], a.lin_defl_mm, a.ang_defl)
        col = collision_mesh(vis, a.hull_faces)
        vis.export(os.path.join(a.out, "meshes", f"{L['name']}_visual.stl"))
        col.export(os.path.join(a.out, "meshes", f"{L['name']}_collision.stl"))
        # trimesh 검산: 메시에 들어간 부품만으로 OpenCascade 값을 다시 계산해 같은 부품끼리 비교
        mk, ck, Ik, volk = mass_props(keep)
        rho_eff = mk / (volk * 1e-9)
        tm_m, tm_c, tm_I = mesh_props(vis, rho_eff)
        rhos = sorted({r for _, r in dens})
        rows.append(dict(link=L["name"], n_parts=len(mine), n_mesh_parts=len(keep), mass_occ=m, vol_cm3=vol / 1000,
                         mass_al_only=vol * 1e-9 * 2700, mass_bom=L.get("bom_mass_kg"),
                         com_occ=c.round(6).tolist(), I_occ=I.round(9).tolist(),
                         mesh_mass_occ=mk, mesh_mass_trimesh=tm_m, com_diff_mm=float(np.linalg.norm(ck - tm_c) / MM),
                         I_rel_err=float(np.linalg.norm(Ik - tm_I) / np.linalg.norm(Ik)),
                         watertight=vis.is_watertight, vis_faces=len(vis.faces), col_faces=len(col.faces),
                         hull_vol_ratio=float(col.volume / (volk * 1e-9)),   # hull 부피 / 메시 부품 실제(B-rep) 부피
                         triangle_ineq=triangle_ok(I)))
        xml.append(inertial_xml(L["name"], m, c, I, f"OpenCascade 질량 특성, 밀도 {'/'.join(str(r) for r in rhos)} kg/m³, 부품 {len(mine)}개"))
        tm_m, m_show = tm_m, mk
        print(f"{L['name']:22s} m={m:.4f} kg  COM={np.round(c,4)} m  watertight={vis.is_watertight}  "
              f"faces vis/col={len(vis.faces)}/{len(col.faces)}  trimesh 차이(메시 부품 기준): m {abs(tm_m-m_show)/m_show*100:.3f} %, I {rows[-1]['I_rel_err']*100:.3f} %")

    dup = {k: v for k, v in used.items() if len(v) > 1}
    unused = [p["path"] for p in parts if p["path"] not in used]
    if dup: print("[경고] 두 링크에 중복 배정:", dup)
    if unused: print(f"[참고] 어느 링크에도 안 들어간 부품 {len(unused)}개:", unused[:10])
    with open(os.path.join(a.out, "link_mass_props.csv"), "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    with open(os.path.join(a.out, "inertial_blocks.xml"), "w", encoding="utf-8") as fp:
        fp.write("<!-- step_to_links.py 생성. URDF 각 <link> 안에 붙여 넣는다 -->\n" + "".join(xml))
    print("저장:", os.path.join(a.out, "link_mass_props.csv"), os.path.join(a.out, "inertial_blocks.xml"))


if __name__ == "__main__":
    main()
