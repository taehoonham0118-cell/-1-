# week05 | v1 링크 원점 = B1 골격 관절점(RI85 출력 디스크 중심)으로 함태훈 CAD 메시 · inertial 을 다시 표현 (함태훈, B1)
#  4주차 문제: 함태훈 링크 원점(CAD 축 위 임의 점)과 B1 골격 관절점이 축 방향으로 17.8~22.15 mm 달라
#             박종진이 merge_taehoon.py 로 평행이동해 썼음 → v1 부터는 함태훈 출력물이 처음부터 관절점 원점이 되게 한다.
#  방법: 두 모델 모두 cad_frame(CAD 원점 · 축, 고정 변환 1개)을 갖는다. q = 0 에서 각 링크 프레임을 cad_frame 기준으로 구해
#        회전 차 = 0 을 확인(assert)하고, 원점 차 dp(B1 링크 프레임 기준)만큼 COM · 메시를 옮긴다. COM 기준 관성은 불변.
#        dp 를 「축 방향 성분」과 「축선 수직 성분」으로 나눠 기록 (4주차 착오: 두 성분 혼동 금지)
#  입력: sim/robot/our_robot_v0.urdf (+ meshes/, link_mass_props.csv), B1 v1 URDF (박종진 b1/models/v1/urdf/b1_v1.urdf)
#  출력: sim/robot_v1/link_inertials_v1.json (build_v1.py 의 INER 입력 형식), sim/robot_v1/meshes/*.stl, rebase_v1_report.csv
#  실행: python sim/scripts/rebase_v1_links.py <B1 b1_v1.urdf>
import ast, csv, json, os, sys
import numpy as np, mujoco, trimesh

HERE = os.path.dirname(os.path.abspath(__file__)); SIM = os.path.dirname(HERE)
SRC = os.path.join(SIM, "robot"); OUT = os.path.join(SIM, "robot_v1"); os.makedirs(os.path.join(OUT, "meshes"), exist_ok=True)
B1 = sys.argv[1]
ARM = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"]
AX = {"shoulder_pitch": (0, 1, 0), "shoulder_roll": (1, 0, 0), "shoulder_yaw": (0, 0, 1), "elbow": (1, 0, 0)}   # 왼팔 링크 프레임 축


def fk(path):
    m = mujoco.MjModel.from_xml_path(path); d = mujoco.MjData(m); mujoco.mj_forward(m, d); return m, d


def in_cad(m, d, body):
    c = m.body("cad_frame").id; i = m.body(body).id; Rc = d.xmat[c].reshape(3, 3)
    return Rc.T @ (d.xpos[i] - d.xpos[c]), Rc.T @ d.xmat[i].reshape(3, 3)


mh, dh = fk(os.path.join(SRC, "our_robot_v0.urdf")); mb, db = fk(B1)
props = {r["link"]: r for r in csv.DictReader(open(os.path.join(SRC, "link_mass_props.csv"), encoding="utf-8-sig"))}
out, rows = {}, []
links = ["torso_link"] + [f"left_{a}_link" for a in ARM]          # 오른팔은 build_v1.py 가 왼팔을 거울로 만든다 (안 A, 팀장 10/16)
for k in links:
    ph, Rh = in_cad(mh, dh, k); pb, Rb = in_cad(mb, db, k)
    dR = Rb.T @ Rh; ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    assert ang < 1e-4, (k, ang)                                   # 링크 축 방향 동일 → 평행이동만
    dp = Rb.T @ (ph - pb)                                         # 함태훈 원점을 B1 링크 프레임으로 표현 [m]
    r = props[k]; m = float(r["mass_occ"]); com = np.array(ast.literal_eval(r["com_occ"])); Iq = np.array(ast.literal_eval(r["I_occ"]))
    com_new = com + dp
    out[k] = dict(mass=m, com=[round(float(x), 9) for x in com_new],
                  inertia=[float(Iq[0, 0]), float(Iq[1, 1]), float(Iq[2, 2]), float(Iq[0, 1]), float(Iq[0, 2]), float(Iq[1, 2])],
                  origin="B1 골격 관절점 (RI85 출력 디스크 중심)" if k != "torso_link" else "어깨 J1 축 중심 (torso_link 원점)",
                  source="함태훈 CAD (양팔로봇 최종버전.stp, 밀도 Al 2700/강 7850/PCB 1850) — rebase_v1_links.py",
                  dp_mm=[round(float(x) * 1000, 3) for x in dp])
    for kind in ("visual", "collision"):
        t = trimesh.load(os.path.join(SRC, "meshes", f"{k}_{kind}.stl")); t.apply_translation(dp)
        t.export(os.path.join(OUT, "meshes", f"{k}_{kind}.stl"))
    if k == "torso_link":
        ax_c = perp = float("nan")
    else:
        a = np.array(AX[k.replace("left_", "").replace("_link", "")], float)
        ax_c = float(dp @ a) * 1000; perp = float(np.linalg.norm(dp - (dp @ a) * a)) * 1000
    rows.append(dict(link=k, mass_kg=round(m, 6), dp_x_mm=round(dp[0] * 1000, 3), dp_y_mm=round(dp[1] * 1000, 3), dp_z_mm=round(dp[2] * 1000, 3),
                     axial_mm=round(ax_c, 3), perp_mm=round(perp, 3), rot_diff_deg=round(ang, 6),
                     com_old=list(np.round(com, 6)), com_new=list(np.round(com_new, 6))))
json.dump(out, open(os.path.join(OUT, "link_inertials_v1.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)
with open(os.path.join(OUT, "rebase_v1_report.csv"), "w", newline="", encoding="utf-8-sig") as fp:
    w = csv.DictWriter(fp, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
for r in rows:
    print(f'{r["link"]:28s} m {r["mass_kg"]:.4f}  dp(mm) ({r["dp_x_mm"]:7.2f},{r["dp_y_mm"]:7.2f},{r["dp_z_mm"]:7.2f})  축방향 {r["axial_mm"]:7.2f}  축선수직 {r["perp_mm"]:5.2f}  회전차 {r["rot_diff_deg"]}°')
