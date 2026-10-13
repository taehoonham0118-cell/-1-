# week04 | 팀 연계: 박종진 관절 표(b1/models/link-split-v0.md §3, 커밋 0dfa6e8, 10/13) vs 함태훈 CAD 축(joint_axes_cad.csv) 검증표 (함태훈, B1)
# 같은 관절이면: 박종진 점이 함태훈 축 「직선 위」에 있어야 하고(수직 거리 ≈ 0), 축 방향이 같아야 함(각도 ≈ 0).
# 축 위 위치(along)는 원점을 어디로 잡았는지의 차이일 뿐 — URDF 에서는 축 위 어느 점이든 같은 관절.
# 실행: python sim/scripts/check_joint_table_vs_pjj.py  → sim/robot/joint_check_vs_pjj.csv
import csv, os
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robot")
# 박종진 link-split-v0.md §3 / results/joint_axes.txt (출력 디스크 중심, 축 = 고정측 → 출력측, CAD mm)
PJJ = {1: ((51.6, 0, 0), (1, 0, 0)), 2: ((159.5, -20.0, 41.2), (0, 0, 1)), 3: ((159.5, -202.3, -2.2), (0.001, -1, -0.024)),
       4: ((159.6, -296.1, 38.3), (0, -0.024, 1)), 5: ((-395.7, -20.0, 41.2), (0, 0, 1)), 6: ((-287.9, 0, 0), (-1, 0, 0)),
       7: ((-388.6, -201.8, -2.2), (0.001, -1, -0.024)), 8: ((-388.5, -295.6, 38.3), (0, -0.024, 1))}
JOINT = {1: "left_shoulder_pitch", 2: "left_shoulder_roll", 3: "left_shoulder_yaw", 4: "left_elbow",
         6: "right_shoulder_pitch", 5: "right_shoulder_roll", 7: "right_shoulder_yaw", 8: "right_elbow"}
rows = []
for r in csv.DictReader(open(os.path.join(ROOT, "joint_axes_cad.csv"), encoding="utf-8-sig")):
    i = int(r["actuator"].split(":")[1])
    p = np.array([float(r[k]) for k in ("pt_x_mm", "pt_y_mm", "pt_z_mm")])
    d = np.array([float(r[k]) for k in ("axis_x", "axis_y", "axis_z")]); d /= np.linalg.norm(d)
    q, a = np.array(PJJ[i][0], float), np.array(PJJ[i][1], float); a /= np.linalg.norm(a)
    v = q - p
    rows.append(dict(joint=JOINT[i], actuator=f"RI85:{i}", ham_pt_mm=p.tolist(), pjj_pt_mm=q.tolist(),
                     perp_dist_mm=round(float(np.linalg.norm(v - (v @ d) * d)), 3), along_axis_mm=round(float(v @ d), 2),
                     axis_angle_deg=round(float(np.degrees(np.arccos(min(1.0, abs(a @ d))))), 3)))
    print(f"{rows[-1]['joint']:22s} 수직거리 {rows[-1]['perp_dist_mm']:6.3f} mm  축 위 거리 {rows[-1]['along_axis_mm']:6.2f} mm  축 각도 차 {rows[-1]['axis_angle_deg']:.3f}°")
with open(os.path.join(ROOT, "joint_check_vs_pjj.csv"), "w", newline="", encoding="utf-8-sig") as fp:
    w = csv.DictWriter(fp, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
print("최대 수직거리", max(r["perp_dist_mm"] for r in rows), "mm, 최대 축 각도 차", max(r["axis_angle_deg"] for r in rows), "°")
