# week04 | 팀 연계: 박종진 골격 URDF(b1_humanoid_skeleton.urdf, 커밋 e292b37) joint origin · axis vs 함태훈 CAD 관절 축 검증표 (함태훈, B1)
# 비교 방법: 두 모델을 q = 0 으로 MuJoCo 에 올려 각 관절의 앵커(xanchor) · 축(xaxis)을 「몸통(torso_link) 프레임」으로 옮긴 뒤,
#   골격 앵커가 CAD 축 직선에서 얼마나 떨어졌는지(수직거리), 축 방향이 몇 도 다른지 계산. 축 위 위치 차이는 원점 선택 차이일 뿐이라 따로 표시.
# 실행: ① 10/13 CAD 그대로 v0 vs 골격 e292b37 → joint_check_skeleton.csv (편차 기록, 결과 보존)
#       ② 10/14 공칭 v0 vs 골격 0129b3c 이후:  python sim/scripts/check_skeleton_vs_cad.py --skeleton <박종진>/b1/models/v0/urdf/b1_humanoid_skeleton.urdf --out joint_check_nominal_vs_skeleton.csv
import argparse, csv, os, re, sys
import numpy as np
import mujoco
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mj_load import load_model

ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robot")
ap = argparse.ArgumentParser(); ap.add_argument("--skeleton", required=True)
ap.add_argument("--mine", default=None, help="비교할 함태훈 URDF (기본: sim/robot/our_robot_v0.urdf)")
ap.add_argument("--yaw_cad", action="store_true", help="양쪽 모두 shoulder_yaw = +π/2 (CAD 자세)로 비교")
ap.add_argument("--out", default="joint_check_skeleton.csv"); a = ap.parse_args()


def joints_in_torso(model):
    d = mujoco.MjData(model)
    if a.yaw_cad:
        for side in ("left", "right"):
            d.qpos[model.joint(f"{side}_shoulder_yaw_joint").qposadr[0]] = np.pi / 2
    mujoco.mj_forward(model, d)
    t = model.body("torso_link").id; Rt, pt = d.xmat[t].reshape(3, 3), d.xpos[t]
    out = {}
    for j in range(model.njnt):
        n = model.joint(j).name
        if n.endswith("_joint") and ("shoulder" in n or "elbow" in n):
            out[n] = (Rt.T @ (d.xanchor[j] - pt), Rt.T @ d.xaxis[j])
    return out


xml = open(a.skeleton, encoding="utf-8").read()
xml = re.sub(r"<(visual|collision)>.*?</\1>", "", xml, flags=re.S)        # 골격 비교에는 형상 불필요
sk = mujoco.MjModel.from_xml_string(xml)
me = load_model(a.mine or os.path.join(ROOT, "our_robot_v0.urdf"))
S, M = joints_in_torso(sk), joints_in_torso(me)
rows = []
for n in sorted(M):
    q, d = M[n]; p, s = S[n]
    v = p - q; along = v @ d; perp = np.linalg.norm(v - along * d)
    ang = np.degrees(np.arccos(min(1.0, abs(s @ d))))
    rows.append(dict(joint=n, skeleton_anchor_m=np.round(p, 5).tolist(), cad_anchor_m=np.round(q, 5).tolist(),
                     perp_dist_mm=round(perp * 1000, 2), along_axis_mm=round(along * 1000, 2),
                     skeleton_axis=np.round(s, 4).tolist(), cad_axis=np.round(d, 4).tolist(), axis_angle_deg=round(ang, 2),
                     axis_sign=("같음" if s @ d > 0 else "반대")))
    print(f"{n:28s} 수직거리 {rows[-1]['perp_dist_mm']:6.2f} mm  축 위 {rows[-1]['along_axis_mm']:7.2f} mm  축 각도 차 {ang:5.2f}°  부호 {rows[-1]['axis_sign']}")
with open(os.path.join(ROOT, a.out), "w", newline="", encoding="utf-8-sig") as fp:
    w = csv.DictWriter(fp, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
print("저장: sim/robot/" + a.out)
