# week05 | 팀 연계: 박종진 미러 규칙표(docs/mirror_rules.md)로 「거울로 생성된 팔」의 액추에이터 부호 · 범위 점검 (함태훈, B1)
#  커리큘럼 문구는 「좌측 팔」이지만 B1 v1 은 기준 팔 = left, 생성 팔 = right → 생성된 right 를 점검한다
#  점검 1 (정적): 관절 쌍마다 class · kp · kv · forcerange · ctrlrange · gear · 관절 범위 · damping · armature 동일, 축 = 유사벡터 규칙 (−x, y, −z)
#  점검 2 (중력): 같은 관절각 300 자세에서 좌우 qfrc_bias(중력 토크) 가 같은 부호 · 같은 크기 (안 A 거울이면 같아야 함)
#  점검 3 (동작): 영점에서 각 액추에이터에 +0.3 rad 명령 → ee 변위 (Δx, Δy, Δz) 가 좌우 (같음, 반대, 같음) 인지 + 「+ 방향의 의미」(roll + = 바깥, elbow + = 앞)
#  점검 4 (B2 부호표 교차): 함태훈 CAD 축(sim/robot/joint_axes_cad.csv, 고정측 → 출력측)을 cad_frame 으로 옮겨
#          CAD 자세(shoulder_yaw = +π/2)의 URDF 관절축과 비교 → 박종진 joint-origin-check-v0.md §2 「같음/반대」와 일치하는지
#  출력: results/mirror_actuator_check.md · .csv
#  실행: python sim/v1/v1_mirror_actuator_check.py
import csv, os
import numpy as np, mujoco
import v1_common as c

m = c.load(); d = mujoco.MjData(m); A = c.assumptions()
mirror_axis = lambda a: np.array([-a[0], a[1], -a[2]])
rows = []; allok = True
# ---- 점검 1
for a in c.ARM:
    jl, jr = m.joint(f"left_{a}_joint").id, m.joint(f"right_{a}_joint").id
    al, ar = m.actuator(f"left_{a}").id, m.actuator(f"right_{a}").id
    dl, dr = m.jnt_dofadr[jl], m.jnt_dofadr[jr]
    same = dict(kp=(m.actuator_gainprm[al][0], m.actuator_gainprm[ar][0]), kv=(m.actuator_biasprm[al][2], m.actuator_biasprm[ar][2]),
                forcerange=(tuple(m.actuator_forcerange[al]), tuple(m.actuator_forcerange[ar])), ctrlrange=(tuple(m.actuator_ctrlrange[al]), tuple(m.actuator_ctrlrange[ar])),
                gear=(m.actuator_gear[al][0], m.actuator_gear[ar][0]), jrange=(tuple(m.jnt_range[jl]), tuple(m.jnt_range[jr])),
                damping=(m.dof_damping[dl], m.dof_damping[dr]), armature=(m.dof_armature[dl], m.dof_armature[dr]), actfrc=(tuple(m.jnt_actfrcrange[jl]), tuple(m.jnt_actfrcrange[jr])))
    diff = {k: v for k, v in same.items() if not np.allclose(np.array(v[0], float), np.array(v[1], float), atol=1e-6)}
    ax_ok = np.allclose(m.jnt_axis[jr], mirror_axis(m.jnt_axis[jl]), atol=1e-9)
    ctrl_eq_range = np.allclose(m.actuator_ctrlrange[ar], m.jnt_range[jr]) and np.allclose(m.actuator_ctrlrange[ar], A["limits"][a])
    ok = (not diff) and ax_ok and ctrl_eq_range and m.actuator_gear[ar][0] == 1.0; allok &= ok
    rows.append(dict(check="1 정적", joint=a, left_axis=m.jnt_axis[jl].round(4).tolist(), right_axis=m.jnt_axis[jr].round(4).tolist(),
                     axis_rule=("반전" if not np.allclose(m.jnt_axis[jl], m.jnt_axis[jr]) else "유지"), axis_ok=ax_ok,
                     ctrlrange=[round(float(x), 4) for x in m.actuator_ctrlrange[ar]], ctrl_eq_range=ctrl_eq_range, diffs=list(diff) or "없음", ok=ok))
# ---- 점검 2
rng = np.random.default_rng(3); worst = 0.0
for _ in range(300):
    q = [rng.uniform(*A["limits"][a]) for a in c.ARM]; d.qpos[:] = 0; d.qvel[:] = 0
    for s in c.SIDES: c.set_arm(m, d, q, s)
    mujoco.mj_forward(m, d)
    tl = np.array([d.qfrc_bias[c.da(m, f"left_{a}_joint")] for a in c.ARM]); tr = np.array([d.qfrc_bias[c.da(m, f"right_{a}_joint")] for a in c.ARM])
    worst = max(worst, np.abs(tl - tr).max())
g_ok = worst < 1e-9; allok &= g_ok
rows.append(dict(check="2 중력", joint="전체", detail=f"300 자세 max|τ_L − τ_R| = {worst:.1e} N·m", ok=g_ok))
# ---- 점검 3
sem = {"shoulder_pitch": ("x", -1, "pitch + = 팔 뒤로 (ee x 감소) — y축 오른손 법칙"), "shoulder_roll": ("|y|", +1, "roll + = 바깥으로 벌림 (ee |y| 증가)"),
       "shoulder_yaw": ("—", 0, "영점에서 팔이 수직이라 ee 거의 안 움직임 (축 = 팔 길이 방향)"), "elbow": ("x", +1, "elbow + = 앞으로 굽힘 (ee x 증가)")}
for a in c.ARM:
    dd = {}
    for s in c.SIDES:
        mujoco.mj_resetData(m, d); mujoco.mj_forward(m, d); e0 = d.site_xpos[m.site(f"{s}_ee").id].copy()
        q0 = {n: d.qpos[c.qa(m, f"{n}_joint")] for n in [f"{x}_{y}" for x in c.SIDES for y in c.ARM]}
        for k in range(1500):
            for n, v in q0.items():
                d.ctrl[m.actuator(n).id] = v + (0.3 if n == f"{s}_{a}" else 0.0) + d.qfrc_bias[c.da(m, f"{n}_joint")] / 300.0
            mujoco.mj_step(m, d)
        dd[s] = d.site_xpos[m.site(f"{s}_ee").id] - e0
    L_, R_ = dd["left"], dd["right"]
    mir = abs(L_[0] - R_[0]) < 1e-6 and abs(L_[1] + R_[1]) < 1e-6 and abs(L_[2] - R_[2]) < 1e-6
    key, sg, txt = sem[a]
    if key == "x": semok = np.sign(L_[0]) == sg and np.sign(R_[0]) == sg
    elif key == "|y|": semok = L_[1] > 0 and R_[1] < 0
    else: semok = np.linalg.norm(L_) < 0.01
    allok &= bool(mir and semok)
    rows.append(dict(check="3 동작", joint=a, dL_mm=(L_ * 1000).round(2).tolist(), dR_mm=(R_ * 1000).round(2).tolist(), mirror=mir, meaning=txt, meaning_ok=bool(semok), ok=bool(mir and semok)))
# ---- 점검 4
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
cad = {int(r["actuator"].split(":")[1]): np.array([float(r["axis_x"]), float(r["axis_y"]), float(r["axis_z"])])
       for r in csv.DictReader(open(os.path.join(ROOT, "robot", "joint_axes_cad.csv"), encoding="utf-8-sig"))}
ACT = {"left": [1, 2, 3, 4], "right": [6, 5, 7, 8]}
PJJ = {("left", "shoulder_pitch"): "같음", ("left", "shoulder_roll"): "같음", ("left", "shoulder_yaw"): "반대", ("left", "elbow"): "같음",
       ("right", "shoulder_pitch"): "반대", ("right", "shoulder_roll"): "반대", ("right", "shoulder_yaw"): "같음", ("right", "elbow"): "반대"}   # joint-origin-check-v0.md §2
mujoco.mj_resetData(m, d)
for s in c.SIDES: d.qpos[c.qa(m, f"{s}_shoulder_yaw_joint")] = np.pi / 2           # CAD 자세
mujoco.mj_forward(m, d); Rc = d.xmat[m.body("cad_frame").id].reshape(3, 3)
for s in c.SIDES:
    for a, n in zip(c.ARM, ACT[s]):
        ax_world = Rc @ cad[n]; ax_urdf = d.xaxis[m.joint(f"{s}_{a}_joint").id]
        dot = float(ax_world @ ax_urdf); mine = "같음" if dot > 0 else "반대"; ok = mine == PJJ[(s, a)]; allok &= ok
        rows.append(dict(check="4 B2 부호", joint=f"{s}_{a}", actuator=f"RI85:{n}", dot=round(dot, 4), ham=mine, pjj=PJJ[(s, a)], angle_deg=round(float(np.degrees(np.arccos(min(1, abs(dot))))), 2), ok=ok))

with open(os.path.join(c.OUT, "mirror_actuator_check.csv"), "w", newline="", encoding="utf-8-sig") as fp:
    keys = []; [keys.append(k) for r in rows for k in r if k not in keys]
    w = csv.DictWriter(fp, fieldnames=keys); w.writeheader(); w.writerows(rows)
L = ["# 팀 연계 — 미러 규칙표로 생성 팔(right) 액추에이터 부호 · 범위 점검 (자동 생성 — sim/v1/v1_mirror_actuator_check.py)", "",
     f"모델 v1 `scene.xml`, 미러 규칙표 = 박종진 `b1/models/v1/docs/mirror_rules.md`, MuJoCo {mujoco.__version__}. **종합: {'전부 OK' if allok else 'NG 있음'}** ({sum(r['ok'] for r in rows)}/{len(rows)})", "",
     "## 1. 정적 (값 · 축 규칙)", "", "| joint | left axis | right axis | 규칙 | 축 OK | right ctrlrange = 관절 범위 = 공용값 | 좌우 다른 값 | 판정 |", "|---|---|---|---|---|---|---|---|"]
L += [f'| {r["joint"]} | {r["left_axis"]} | {r["right_axis"]} | {r["axis_rule"]} | {r["axis_ok"]} | {r["ctrlrange"]} {r["ctrl_eq_range"]} | {r["diffs"]} | {"OK" if r["ok"] else "NG"} |' for r in rows if r["check"] == "1 정적"]
L += ["", "- 비교한 값: class kp · kv(컴파일) · forcerange · ctrlrange · gear · 관절 range · damping · armature · joint actuatorfrcrange", "",
      "## 2. 중력 토크 부호", "", f'- {[r for r in rows if r["check"] == "2 중력"][0]["detail"]} → 같은 관절각에서 좌우 토크가 부호까지 같음 = 제어기 · 토크 예산표를 한 팔 기준으로 써도 됨', "",
      "## 3. 동작 (영점에서 +0.3 rad, 1.5 s 후 ee 변위 mm)", "", "| joint | left Δ(x, y, z) | right Δ(x, y, z) | 거울(x 같음 · y 반대 · z 같음) | + 방향 의미 | 판정 |", "|---|---|---|---|---|---|"]
L += [f'| {r["joint"]} | {r["dL_mm"]} | {r["dR_mm"]} | {r["mirror"]} | {r["meaning"]} ({r["meaning_ok"]}) | {"OK" if r["ok"] else "NG"} |' for r in rows if r["check"] == "3 동작"]
L += ["", "## 4. 구동기 +방향(고정측 → 출력측) vs URDF 축 — B2 부호표 교차 확인", "",
      "| joint | STEP 구동기 | 함태훈 CAD 축 · URDF 축 내적 | 축 각도차 ° | 함태훈 | 박종진 §2 | 일치 |", "|---|---|---|---|---|---|---|"]
L += [f'| {r["joint"]} | {r["actuator"]} | {r["dot"]} | {r["angle_deg"]} | {r["ham"]} | {r["pjj"]} | {"OK" if r["ok"] else "NG"} |' for r in rows if r["check"] == "4 B2 부호"]
L += ["", "- 「반대」 4관절(left yaw, right pitch · roll · elbow)은 B2 드라이버 계층 한 곳에서만 q · τ 부호 반전 (팀장 10/16). MJCF 액추에이터 gear 는 전부 +1 로 두는 것이 맞음 — 여기서 반전하면 두 번 뒤집힘",
      "- 축 각도차 1.37° = A1 편차(yaw · elbow, 회신 대기). 공칭 축은 직교 유지"]
open(os.path.join(c.OUT, "mirror_actuator_check.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
print("\n".join(L))
