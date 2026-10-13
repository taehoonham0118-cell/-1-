# week04 | G1 팔 링크로 「메시 → 질량 · 관성」 검산 + convex hull 한계 측정 (함태훈, B1)
# 우리 로봇 STEP 이 오기 전, 실제 로봇 메시(Unitree G1, mujoco_menagerie)로 4주차 절차를 먼저 시험한다.
#   - G1 의 공식 inertial(g1.xml: 질량 · COM · 주관성 · quat) = 우리 로봇의 「BOM · 실측」 자리
#   - 메시를 균일 밀도로 적분한 값                              = 우리 로봇의 「CAD(FreeCAD) · trimesh」 자리
# 실행 (저장소 루트에서):
#   python sim/scripts/g1_mesh_vs_official.py --g1 mujoco_menagerie/unitree_g1
# 출력: sim/scripts/g1_mesh_vs_official.csv, sim/urdf/g1_left_arm_inertial_blocks.xml
import argparse, csv, os, xml.etree.ElementTree as ET
import numpy as np
import trimesh

LINKS = ["left_shoulder_pitch_link", "left_shoulder_roll_link", "left_shoulder_yaw_link", "left_elbow_link",
         "left_wrist_roll_link", "left_wrist_pitch_link", "left_wrist_yaw_link", "waist_yaw_link", "waist_roll_link"]
AL = 2700.0   # 알루미늄 밀도 kg/m³


def quat2mat(q):                     # MuJoCo quat = (w, x, y, z)
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def official(root, link):
    b = root.find(f".//body[@name='{link}']"); i = b.find("inertial")
    m = float(i.get("mass")); c = np.array(i.get("pos").split(), float)
    R = quat2mat(i.get("quat", "1 0 0 0").split()); D = np.diag(np.array(i.get("diaginertia").split(), float))
    meshes = []                                                 # 이 body 의 visual 메시 전부 (손처럼 pos 로 붙은 것 포함)
    for g in b.findall("geom"):
        if g.get("mesh") and g.get("class") != "collision":
            T = np.eye(4); T[:3, :3] = quat2mat(g.get("quat", "1 0 0 0").split())
            T[:3, 3] = np.array(g.get("pos", "0 0 0").split(), float)
            meshes.append((g.get("mesh"), T))
    return m, c, R @ D @ R.T, meshes                            # URDF 식: COM 기준, 링크 축 방향 전체 텐서


def mesh_file(root, name):
    for e in root.find("asset").findall("mesh"):
        f = e.get("file")
        if e.get("name", os.path.splitext(f)[0]) == name:
            return f
    return name + ".STL"


def hull_gap(mesh, hull, n=20000, seed=0):
    """hull 표면에서 점을 뽑아 실제 메시 표면까지 거리 → convex hull 이 메운 '빈 공간' 깊이 [m]"""
    pts, _ = trimesh.sample.sample_surface(hull, n, seed=seed)
    _, dist, _ = trimesh.proximity.closest_point(mesh, pts)
    return float(dist.max()), float(np.percentile(dist, 95))


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--g1", default="mujoco_menagerie/unitree_g1")
    a = ap.parse_args()
    root = ET.parse(os.path.join(a.g1, "g1.xml")).getroot()
    here = os.path.dirname(os.path.abspath(__file__))
    rows, xml = [], []
    for L in LINKS:
        m, c, I, parts = official(root, L)
        ms = []
        for mname, T in parts:
            t = trimesh.load(os.path.join(a.g1, "assets", mesh_file(root, mname))); t.apply_transform(T); ms.append(t)
        mesh = trimesh.util.concatenate(ms)
        V = mesh.volume
        rho = m / V                                        # 공식 질량에 맞춘 균일 밀도
        mm = mesh.copy(); mm.density = rho
        cm, Im = mm.center_mass, mm.moment_inertia
        hull = mesh.convex_hull
        gmax, g95 = hull_gap(mesh, hull)
        ev_o, ev_m = np.linalg.eigvalsh(I), np.linalg.eigvalsh(Im)
        rows.append(dict(link=L, n_mesh=len(ms), faces=len(mesh.faces), watertight=all(t.is_watertight for t in ms),
                         volume_cm3=round(V * 1e6, 2), mass_official=m, mass_if_aluminum=round(V * AL, 4),
                         rho_eff=round(rho), com_off_mm=round(float(np.linalg.norm(cm - c)) * 1000, 2),
                         I_rel_err_pct=round(float(np.linalg.norm(Im - I) / np.linalg.norm(I)) * 100, 1),
                         Imax_ratio_mesh_over_official=round(float(ev_m[-1] / ev_o[-1]), 3),
                         hull_faces=len(hull.faces), hull_vol_ratio=round(float(hull.volume / V), 3),
                         hull_gap_max_mm=round(gmax * 1000, 1), hull_gap_p95_mm=round(g95 * 1000, 1),
                         triangle_ok=bool(ev_o[0] + ev_o[1] >= ev_o[2])))
        f = lambda v: f"{v:.6g}"
        xml.append(f'  <!-- {L}: G1 공식 값(g1.xml quat+diaginertia → 링크 축 전체 텐서로 변환) -->\n'
                   f'  <inertial>\n    <origin xyz="{f(c[0])} {f(c[1])} {f(c[2])}" rpy="0 0 0"/>\n'
                   f'    <mass value="{f(m)}"/>\n'
                   f'    <inertia ixx="{f(I[0,0])}" iyy="{f(I[1,1])}" izz="{f(I[2,2])}" ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" iyz="{f(I[1,2])}"/>\n'
                   f'  </inertial>\n')
        r = rows[-1]
        print(f"{L:26s} 면 {r['faces']:6d} 닫힘 {str(r['watertight']):5s} 부피 {r['volume_cm3']:7.2f} cm³ | 질량 공식 {m:.3f} / Al {r['mass_if_aluminum']:.3f} kg"
              f" (ρ_eff {r['rho_eff']}) | COM 차 {r['com_off_mm']:5.1f} mm | 관성 오차 {r['I_rel_err_pct']:5.1f} % | hull 부피 ×{r['hull_vol_ratio']:.2f}, 틈 최대 {r['hull_gap_max_mm']} mm")
    out_csv = os.path.join(here, "g1_mesh_vs_official.csv")
    with open(out_csv, "w", newline="", encoding="utf-8-sig") as fp:
        w = csv.DictWriter(fp, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    out_xml = os.path.join(os.path.dirname(here), "urdf", "g1_left_arm_inertial_blocks.xml")
    with open(out_xml, "w", encoding="utf-8") as fp:
        fp.write("<!-- g1_mesh_vs_official.py 생성 — G1 공식 inertial 을 URDF 형식으로 옮긴 것 -->\n" + "".join(xml))
    print("저장:", out_csv, out_xml)


if __name__ == "__main__":
    main()
