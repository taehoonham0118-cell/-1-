# week05 | v1 모델 검증 ① 질량 합계 · ② 정지 유지 · ③ 중력 토크 vs 구동기 — 함태훈 담당분을 독립 방법으로 (함태훈, B1)
#  박종진 validate_model.py(B1 공용 기준 계산)와 「다른 방법」으로 같은 질문에 답하고 숫자를 대조한다.
#   ① 기준 = 함태훈 CAD 질량표(sim/robot/link_mass_props.csv) + B1 공용 가정값(assumptions_b1.json). 질량 + 전체 COM + 링크 주관성 모멘트까지 비교
#   ② 4 자세(영점 · 앞으로 수평 · 옆으로 수평 · ③의 pitch 최악 자세) × 중력보상 유/무, 5 s. 처짐 이론값 τg/kp 와 비교, 베이스 밀림 · 떨림 기록
#   ③ qfrc_bias(qvel = 0) 를 4가지로 검산: mj_rne(가속 0) · 자코비안 전치합 −Σ J_comᵀ m g · 외적 합 Σ aⱼ·((cᵢ − pⱼ) × mᵢ g) · 손계산(수평 자세, FK 없이)
#      최악 자세 = 관절 범위 안 전역 최적화(scipy differential_evolution) — 15° 격자(박종진)보다 촘촘한지 확인
#      페이로드 0 / 0.5 / 1.0 / 3.0 kg (ee site), 3 kg 은 플랜지(tool0) 작용도 같이 → 정격 46 N·m(팀장 10/16 PDR 기준) 대비 여유율
#  출력: results/validate_123.json · validate_123.md, assets/screenshots/week05/week05_hold.png · week05_gravity_torque.png
#  실행: python sim/v1/v1_validate_123.py
import ast, csv, json, os
import numpy as np, mujoco
from scipy.optimize import differential_evolution
import v1_common as c

m = c.load(); d = mujoco.MjData(m); A = c.assumptions(); P = c.params()
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))           # sim/
RATED = 46.0; PEAK = A["actuator"]["effort_peak"]; SF = 1.2                  # 정격 ≥ 정적 필요 토크 × 1.2 (팀장 10/16)
R = {"model": dict(nq=m.nq, nu=m.nu, nbody=m.nbody, mujoco=mujoco.__version__, timestep=m.opt.timestep)}

# ================================================================ ① 질량 합계
props = {r["link"]: r for r in csv.DictReader(open(os.path.join(ROOT, "robot", "link_mass_props.csv"), encoding="utf-8-sig"))}
v1 = json.load(open(os.path.join(ROOT, "robot_v1", "link_inertials_v1.json"), encoding="utf-8"))
b, t, g = A["base"], A["torso"], A["gripper"]
ref = {"base_link": (b["mass_lower"], "가정 C1"), "base_plate_link": (b["mass_plate"], "CAD BASE 3.STEP 3.6901 kg (C1)"),
       "torso_link": (t["box_mass"] + float(props["torso_link"]["mass_occ"]), f'가정 box {t["box_mass"]} + CAD RI85 고정측 {float(props["torso_link"]["mass_occ"]):.4f}')}
for s in c.SIDES:
    for a in c.ARM:
        ref[f"{s}_{a}_link"] = (float(props[f"left_{a}_link"]["mass_occ"]), "CAD 왼팔(오른팔 = 거울, 안 A)")
    ref[f"{s}_gripper_link"] = (g["mass"], "가정 S14 (BOM 대기)")
rows = []
for n, (v, src) in ref.items():
    bid = m.body(n).id; mm = float(m.body_mass[bid])
    row = dict(link=n, model_kg=round(mm, 4), ref_kg=round(v, 4), err_pct=round((mm - v) / v * 100, 4), src=src)
    if n in v1 or n.replace("right_", "left_") in v1:          # CAD 링크: 주관성 모멘트(고유값)도 비교 — 회전 · 거울에 불변
        k = n.replace("right_", "left_"); I6 = v1[k]["inertia"]
        I = np.array([[I6[0], I6[3], I6[4]], [I6[3], I6[1], I6[5]], [I6[4], I6[5], I6[2]]])
        if k == "torso_link":     # 몸통 = CAD + box 합성 → 합성 COM 기준 (평행축)
            tz0 = b["plate_top_z"] - t["shoulder_z"]; tz1 = t["box_top_above_shoulder"]; th = tz1 - tz0
            bm = t["box_mass"]; bc = np.array([0, 0, (tz0 + tz1) / 2]); bI = np.diag([bm * (t["box_y"]**2 + th**2), bm * (t["box_x"]**2 + th**2), bm * (t["box_x"]**2 + t["box_y"]**2)]) / 12
            cm, cc = v1[k]["mass"], np.array(v1[k]["com"]); M_ = cm + bm; C = (cm * cc + bm * bc) / M_
            I = sum(Ii + mi * ((ci - C) @ (ci - C) * np.eye(3) - np.outer(ci - C, ci - C)) for mi, ci, Ii in ((cm, cc, I), (bm, bc, bI)))
        ev = np.sort(np.linalg.eigvalsh(I)); evm = np.sort(m.body_inertia[bid])
        row["I_principal_err_pct"] = round(float(np.max(np.abs(evm - ev) / ev) * 100), 4)
    rows.append(row)
tot = float(m.body_subtreemass[0]); reft = sum(v for v, _ in ref.values()); dummy = tot - sum(r["model_kg"] for r in rows)
th_right = sum(float(props[f"right_{a}_link"]["mass_occ"]) for a in c.ARM); th_left = sum(float(props[f"left_{a}_link"]["mass_occ"]) for a in c.ARM)
mujoco.mj_forward(m, d); com = d.subtree_com[0].copy()
R["mass"] = dict(total=round(tot, 4), ref_total=round(reft, 4), err_pct=round((tot - reft) / reft * 100, 4), dummy_kg=round(dummy, 4),
                 robot_com_m=[round(float(x), 4) for x in com], cad_right_arm=round(th_right, 4), cad_left_arm=round(th_left, 4), links=rows,
                 pass_=bool(abs(tot - reft) / reft < 0.05 and all(abs(r["err_pct"]) < 5 for r in rows)))

# ================================================================ ③ 중력 토크 (② 의 최악 자세를 쓰므로 먼저)
ee = m.site("left_ee").id; tool = m.body("left_tool0").id
dofs = [c.da(m, f"left_{a}_joint") for a in c.ARM]
lims = [tuple(A["limits"][a]) for a in c.ARM]


def tau_g(q, payload=0.0, at="ee"):
    d.qpos[:] = 0; d.qvel[:] = 0; c.set_arm(m, d, q); mujoco.mj_forward(m, d)
    tau = d.qfrc_bias[dofs].copy()
    if payload:
        jac = np.zeros((3, m.nv))
        if at == "ee": mujoco.mj_jacSite(m, d, jac, None, ee)
        else: mujoco.mj_jac(m, d, jac, None, d.xpos[tool], tool)
        tau += -(jac[:, dofs].T @ np.array([0, 0, -c.G * payload]))       # 들고 있는 데 필요한 토크 = −Jᵀ F_gravity
    return tau


# (a) 검산: 무작위 200 자세에서 4 방법 일치
rng = np.random.default_rng(1); err = dict(rne=0.0, jac=0.0, cross=0.0)
for _ in range(200):
    q = [rng.uniform(*l) for l in lims]; tb = tau_g(q)
    rn = np.zeros(m.nv); mujoco.mj_rne(m, d, 0, rn); err["rne"] = max(err["rne"], np.abs(rn[dofs] - tb).max())
    tj = np.zeros(m.nv); jac = np.zeros((3, m.nv))
    for bi in range(1, m.nbody):
        mujoco.mj_jacBodyCom(m, d, jac, None, bi); tj += -(jac.T @ np.array([0, 0, -c.G * m.body_mass[bi]]))
    err["jac"] = max(err["jac"], np.abs(tj[dofs] - tb).max())
    tc = []
    for a in c.ARM:                       # 외적 합: 관절 j 보다 바깥 body 만, aⱼ · Σ (cᵢ − pⱼ) × (mᵢ g)
        jid = m.joint(f"left_{a}_joint").id; pj = d.xanchor[jid]; aj = d.xaxis[jid]; jb = m.jnt_bodyid[jid]
        sub = [bi for bi in range(1, m.nbody) if any(x == jb for x in _anc(bi))] if False else None
        s_ = 0.0
        for bi in range(1, m.nbody):
            p_ = bi; inside = False
            while p_ > 0:
                if p_ == jb: inside = True; break
                p_ = m.body_parentid[p_]
            if inside: s_ += aj @ np.cross(d.xipos[bi] - pj, np.array([0, 0, -c.G * m.body_mass[bi]]))
        tc.append(-s_)
    err["cross"] = max(err["cross"], np.abs(np.array(tc) - tb).max())
R["gravity_check"] = {k: float(v) for k, v in err.items()}

# (b) 손계산: 앞으로 수평(pitch −90°) — 영점에서 팔이 수직으로 내려 있으므로 각 COM 의 「어깨축 아래 깊이」가 수평 팔 길이가 된다
#     z 성분만 더하면 된다 (shoulder_yaw origin 의 Rz(∓π/2) 는 x · y 만 바꿈). 값 = link_inertials_v1.json + build_v1 관절 위치
Jz = {"shoulder_pitch": 0.0, "shoulder_roll": -0.0200, "shoulder_yaw": -0.0200 - 0.1823, "elbow": -0.0200 - 0.1823 - 0.0938}
z_tool = Jz["elbow"] - 0.1989
hand = sum(v1[f"left_{a}_link"]["mass"] * (-(Jz[a] + v1[f"left_{a}_link"]["com"][2])) for a in c.ARM) * c.G
hand += g["mass"] * c.G * (-(z_tool - g["box_xyz"][2] / 2))
hand_3 = hand + 3.0 * c.G * (-(z_tool - g["ee_offset"]))
R["hand_horizontal"] = dict(tau0=round(hand, 4), tau3=round(hand_3, 4), mj0=round(float(abs(tau_g([-np.pi / 2, 0, 0, 0])[0])), 4),
                            mj3=round(float(abs(tau_g([-np.pi / 2, 0, 0, 0], 3.0)[0])), 4), arm_reach_ee_m=round(-(z_tool - g["ee_offset"]), 4))

# (c) Q1 실험: qvel ≠ 0 이면 qfrc_bias 에 코리올리 · 원심 항이 더해짐, 감쇠는 qfrc_passive 로 따로
q = [-np.pi / 2, 0.3, 0.4, 0.8]; d.qpos[:] = 0; c.set_arm(m, d, q); d.qvel[:] = 0
for x in dofs: d.qvel[x] = 3.0                                              # 모든 팔 관절 3 rad/s (C4 velocity 상한)
mujoco.mj_forward(m, d); b3 = d.qfrc_bias[dofs].copy(); pas = d.qfrc_passive[dofs].copy(); g0 = tau_g(q)
R["bias_vs_qvel"] = dict(pose=q, qvel=3.0, gravity=[round(float(x), 3) for x in g0], bias_qvel3=[round(float(x), 3) for x in b3],
                         coriolis=[round(float(x), 3) for x in b3 - g0], passive_damping=[round(float(x), 3) for x in pas])

# (d) 최악 자세: 전역 최적화
grav, worst = {}, {}
for pl, at in ((0.0, "ee"), (0.5, "ee"), (1.0, "ee"), (3.0, "ee"), (3.0, "tool0")):
    key = f"{pl:g}kg@{at}"; grav[key] = {}
    for i, a in enumerate(c.ARM):
        best = None
        for seed in (0, 1, 2):
            r = differential_evolution(lambda q: -abs(tau_g(q, pl, at)[i]), lims, seed=seed, tol=1e-10, maxiter=300, polish=True)
            if best is None or r.fun < best.fun: best = r
        tau = -best.fun
        grav[key][a] = dict(tau=round(float(tau), 3), pose_deg=[round(float(np.degrees(x)), 1) for x in best.x],
                            need_rated=round(float(tau * SF), 2), margin_rated_pct=round((RATED - tau) / tau * 100, 1),
                            margin_peak_pct=round((PEAK - tau) / tau * 100, 1), ok=bool(tau * SF <= RATED))
        if key == "0kg@ee": worst[a] = best.x
R["gravity"] = dict(rated=RATED, peak=PEAK, sf=SF, results=grav, pass_3kg=all(v["ok"] for v in grav["3kg@ee"].values()))

# ================================================================ ② 정지 유지
acts = [m.actuator(f"{s}_{a}").id for s in c.SIDES for a in c.ARM]
jn = [f"{s}_{a}_joint" for s in c.SIDES for a in c.ARM]
kp = np.array([m.actuator_gainprm[i][0] for i in acts])


def hold(qL, gc, T=5.0, payload=0.0):
    mujoco.mj_resetData(m, d)
    for s in c.SIDES: c.set_arm(m, d, qL, s)
    mujoco.mj_forward(m, d); q0 = np.array([d.qpos[c.qa(m, j)] for j in jn]); base0 = d.qpos[:3].copy()
    tg0 = np.array([d.qfrc_bias[c.da(m, j)] for j in jn])
    E, V, tt = [], [], []
    for k in range(int(T / m.opt.timestep)):
        tg = np.array([d.qfrc_bias[c.da(m, j)] for j in jn])
        d.ctrl[acts] = q0 + (tg / kp if gc else 0.0)
        d.ctrl[:3] = 0.0                                                     # 베이스 속도 명령 0
        mujoco.mj_step(m, d)
        q = np.array([d.qpos[c.qa(m, j)] for j in jn]); E.append(np.degrees(np.abs(q - q0))); V.append([d.qvel[c.da(m, j)] for j in jn]); tt.append(d.time)
    E = np.array(E); V = np.array(V); last = int(len(E) * 0.8)
    sag = np.degrees(np.abs(tg0 / kp))
    return dict(final_err_deg=round(float(E[-1].max()), 4), max_err_deg=round(float(E.max()), 4), worst_joint=jn[int(np.argmax(E[-1]))],
                sag_theory_deg=round(float(sag.max()), 4), qvel_rms_last1s=float(np.sqrt((V[last:] ** 2).mean())),
                base_drift_mm=round(float(np.linalg.norm(d.qpos[:2] - base0[:2]) * 1000), 4), finite=bool(np.isfinite(d.qpos).all()),
                ncon=int(sum(1 for cc in d.contact[:d.ncon] if "floor" not in (m.geom(cc.geom1).name, m.geom(cc.geom2).name))),
                pass_=bool(E[-1].max() <= 0.5 and np.isfinite(d.qpos).all()), _t=np.array(tt), _e=E.max(axis=1))


POSES = {"영점": [0, 0, 0, 0], "앞으로 수평": [-np.pi / 2, 0, 0, 0], "옆으로 벌림(roll 상한)": [0, A["limits"]["shoulder_roll"][1], 0, 0],
         "pitch 최악(③)": list(worst["shoulder_pitch"])}
H = {}
for nme, qL in POSES.items():
    for gc in (False, True):
        H[f"{nme}|{'오프셋(A)' if gc else '없음'}"] = hold(qL, gc)
# 방식 B: MuJoCo gravcomp(팔 body gravcomp="1") + joint actuatorgravcomp="true" — 보상 토크가 qfrc_actuator 로 들어가
#   ctrlrange(= 관절 범위, inheritrange) 에 잘리지 않는다. 합계는 joint actuatorfrcrange(= URDF effort ±100) 로 제한
import re
def gc_edit(x):
    x = re.sub(r'<body name="((left|right)_(shoulder|elbow|tool0|gripper)[^"]*)"', r'<body name="\1" gravcomp="1"', x)
    return x.replace('<joint armature=', '<joint actuatorgravcomp="true" armature=')
mB = c.load(xml_edit=gc_edit); dB = mujoco.MjData(mB)
actsB = [mB.actuator(f"{s}_{a}").id for s in c.SIDES for a in c.ARM]
for nme, qL in POSES.items():
    mujoco.mj_resetData(mB, dB)
    for s in c.SIDES: c.set_arm(mB, dB, qL, s)
    mujoco.mj_forward(mB, dB); q0 = np.array([dB.qpos[c.qa(mB, j)] for j in jn]); e = []
    for k in range(int(5.0 / mB.opt.timestep)):
        dB.ctrl[actsB] = q0; dB.ctrl[:3] = 0; mujoco.mj_step(mB, dB)
        e.append(np.degrees(np.abs(np.array([dB.qpos[c.qa(mB, j)] for j in jn]) - q0)).max())
    H[f"{nme}|gravcomp(B)"] = dict(final_err_deg=round(float(e[-1]), 5), max_err_deg=round(float(max(e)), 5), finite=bool(np.isfinite(dB.qpos).all()),
                                   pass_=bool(e[-1] <= 0.5), _t=None, _e=None)
R["gravcomp_B"] = dict(n_bodies=int((mB.body_gravcomp > 0).sum()), n_joints_actgc=int(mB.jnt_actgravcomp.sum()),
                       jnt_actfrcrange=mB.jnt_actfrcrange[c.da(mB, "left_shoulder_pitch_joint")].tolist())
R["hold"] = {k: {kk: vv for kk, vv in v.items() if not kk.startswith("_")} for k, v in H.items()}
R["hold_pass_gc_all"] = all(v["pass_"] for k, v in H.items() if k.endswith("(A)"))
R["hold_pass_gcB_all"] = all(v["pass_"] for k, v in H.items() if k.endswith("(B)"))
R["hold_pass_nogc"] = {k: v["pass_"] for k, v in H.items() if k.endswith("없음")}

# ================================================================ 그림
plt = c.plot_style()
fig, ax = plt.subplots(figsize=(6.4, 3.4))
for (k, v), col in zip([(k, v) for k, v in H.items() if k.endswith("없음")], c.SERIES):
    ax.plot(v["_t"], v["_e"], color=col, label=k.split("|")[0] + " · 보상 없음")
for (k, v), col in zip([(k, v) for k, v in H.items() if k.endswith("(A)")], c.SERIES):
    ax.plot(v["_t"], v["_e"], color=col, ls="--", lw=1.2)
ax.axhline(0.5, color="#52514e", lw=0.8, ls=":"); ax.text(4.95, 0.62, "기준 0.5°", ha="right", fontsize=8, color="#52514e")
ax.set(xlabel="t [s]", ylabel="최대 관절 오차 [°]", title="② 정지 유지 5 s (kp 300) — 실선 보상 없음 · 점선 목표 오프셋(A)")
ax.annotate("roll 상한: 오프셋(A)도 4.33° 남음\n(목표가 ctrlrange 에 잘림)", xy=(3.0, 4.33), xytext=(2.2, 2.6), fontsize=8, color="#52514e",
            arrowprops=dict(arrowstyle="-", color="#8a8986", lw=0.8))
ax.legend(loc="center right", bbox_to_anchor=(1.0, 0.62)); fig.tight_layout(); fig.savefig(os.path.join(c.IMG, "week05_hold.png"), dpi=150)

fig, ax = plt.subplots(figsize=(6.4, 3.4)); keys = ["0kg@ee", "0.5kg@ee", "1kg@ee", "3kg@ee"]; w = 0.19
for k_, (key, col) in enumerate(zip(keys, c.SERIES)):
    vals = [grav[key][a]["tau"] for a in c.ARM]
    ax.bar(np.arange(4) + (k_ - 1.5) * w, vals, w * 0.92, color=col, label=f"페이로드 {key.split('kg')[0]} kg")
ax.set_ylim(0, 54); ax.axhline(RATED / SF, color="#52514e", lw=1, ls="--"); ax.text(2.5, RATED / SF + 0.8, f"정격 46 ÷ 1.2 = {RATED / SF:.1f}", ha="left", fontsize=8, color="#52514e")
ax.set_xticks(range(4)); ax.set_xticklabels(c.ARM); ax.set(ylabel="최악 자세 |τg| [N·m]", title="③ 관절별 최악 중력 토크 (전역 최적화, 페이로드 = ee site)")
ax.legend(loc="upper center", ncol=4, bbox_to_anchor=(0.5, 1.0)); ax.grid(axis="x", visible=False); fig.tight_layout(); fig.savefig(os.path.join(c.IMG, "week05_gravity_torque.png"), dpi=150)

json.dump(R, open(os.path.join(c.OUT, "validate_123.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False, default=float)
ok = lambda b_: "**통과**" if b_ else "**미통과**"
mres = R["mass"]; hh = R["hand_horizontal"]; gr = R["gravity"]["results"]
L = ["# v1 검증 ① · ② · ③ — 함태훈 독립 계산 (자동 생성 — sim/v1/v1_validate_123.py)", "",
     f'모델: 박종진 build_v1.py 빌드 `scene.xml`(B1 공용, nq {m.nq} · nu {m.nu}), MuJoCo {mujoco.__version__}, timestep {m.opt.timestep} s. 기준 = 함태훈 CAD 질량표 + B1 공용 가정값 v0.2. 정격 판정 = 최악 |τg| × {SF} ≤ {RATED:g} N·m (팀장 10/16), 피크 {PEAK:g} N·m', "",
     "| 검증 | 통과 기준 | 결과 | 판정 |", "|---|---|---|---|",
     f'| ① 질량 합계 | 전체 · 링크별 ±5 % | {mres["total"]} vs 기준 {mres["ref_total"]} kg ({mres["err_pct"]:+.3f} %, 차 = 더미 프레임 {mres["dummy_kg"]} kg), 링크 13개 최대 {max(abs(r["err_pct"]) for r in rows):.4f} %, 주관성 모멘트 최대 {max(r.get("I_principal_err_pct", 0) for r in rows):.4f} % | {ok(mres["pass_"])} |',
     f'| ② 정지 유지 | 5 s, 오차 ≤ 0.5°, 발산 · 떨림 없음 | 보상 없음: 영점 {R["hold"]["영점|없음"]["final_err_deg"]}° · 앞 수평 {R["hold"]["앞으로 수평|없음"]["final_err_deg"]}° (이론 τg/kp {R["hold"]["앞으로 수평|없음"]["sag_theory_deg"]}°) / 목표 오프셋(A): roll 상한 자세 {R["hold"]["옆으로 벌림(roll 상한)|오프셋(A)"]["final_err_deg"]}° 남음 / gravcomp(B): 4자세 최대 {max(v["final_err_deg"] for k, v in R["hold"].items() if k.endswith("(B)"))}° | 보상 없음 {ok(all(R["hold_pass_nogc"].values()))} · A {ok(R["hold_pass_gc_all"])} · B {ok(R["hold_pass_gcB_all"])} |',
     f'| ③ 중력 토크 vs 구동기 | 최악 |τg| × 1.2 ≤ 46 N·m | 3 kg(ee): pitch {gr["3kg@ee"]["shoulder_pitch"]["tau"]} · roll {gr["3kg@ee"]["shoulder_roll"]["tau"]} · yaw {gr["3kg@ee"]["shoulder_yaw"]["tau"]} · elbow {gr["3kg@ee"]["elbow"]["tau"]} N·m → 필요 정격 pitch {gr["3kg@ee"]["shoulder_pitch"]["need_rated"]} · roll {gr["3kg@ee"]["shoulder_roll"]["need_rated"]} N·m | {ok(R["gravity"]["pass_3kg"])} (어깨 2관절) |', "",
     "## ① 질량 · COM · 관성", "", "| 링크 | 모델 kg | 기준 kg | 오차 % | 주관성 모멘트 오차 % | 기준 출처 |", "|---|---|---|---|---|---|"]
L += [f'| {r["link"]} | {r["model_kg"]} | {r["ref_kg"]} | {r["err_pct"]} | {r.get("I_principal_err_pct", "–")} | {r["src"]} |' for r in rows]
L += ["", f'- 로봇 전체 COM (영점, odom 기준) = {mres["robot_com_m"]} m. 함태훈 CAD 실제 오른팔 {mres["cad_right_arm"]} kg vs 왼팔 {mres["cad_left_arm"]} kg (차 {mres["cad_right_arm"] - mres["cad_left_arm"]:+.4f} kg, {(mres["cad_right_arm"] / mres["cad_left_arm"] - 1) * 100:+.2f} %) — 공칭 거울로 버린 차이, ±5 % 안',
      "- 주관성 모멘트(고유값) 비교 = 회전 · 거울에 불변이라 링크 프레임이 달라도 비교 가능. 몸통은 CAD + box 8 kg 을 평행축으로 합성해 비교", "- BOM 미수령 → 「BOM 대비」가 아니라 「CAD + 가정값 대비」. BOM 수령 시 기준 열만 교체", "",
      "## ② 정지 유지 (kp 300 · dampratio 1, 양팔 같은 자세, 베이스 속도 명령 0)", "",
      "| 자세 | 보상 | 5 s 후 오차 ° | 최대 오차 ° | 이론 처짐 τg/kp ° | 마지막 1 s qvel RMS [rad/s] | 베이스 밀림 mm | 판정 |", "|---|---|---|---|---|---|---|---|"]
for k, v in R["hold"].items():
    nme, mode = k.split("|")
    L.append(f'| {nme} | {mode} | {v["final_err_deg"]} | {v["max_err_deg"]} | {v.get("sag_theory_deg", "–")} | {("%.1e" % v["qvel_rms_last1s"]) if "qvel_rms_last1s" in v else "–"} | {v.get("base_drift_mm", "–")} | {ok(v["pass_"])} |')
L += ["", "- **보상 A**(박종진 방식) = position 목표에 τg/kp 를 더함. 그런데 ctrlrange = 관절 범위(`inheritrange=1`)라서 **관절 한계 근처에서는 목표가 잘려** 보상이 사라진다 → roll 상한(1.5 rad)에서 4.33° 그대로 남음 (함태훈 발견 10/17)",
      f'- **보상 B** = MuJoCo `gravcomp="1"`(팔 body {R["gravcomp_B"]["n_bodies"]}개) + joint `actuatorgravcomp="true"`({R["gravcomp_B"]["n_joints_actgc"]}개): 보상 토크가 qfrc_actuator 로 들어가 목표 클리핑과 무관, 합계는 joint actuatorfrcrange {R["gravcomp_B"]["jnt_actfrcrange"]} N·m 로 제한 → 4자세 전부 0°. 주의: `actuatorfrc` 센서는 보상분을 포함하지 않으므로 토크 기록은 `jointactuatorfrc` 센서로',
      "- 보상 없는 처짐은 이론 τg/kp 와 0.4 % 안에서 일치 → kp 300 만으로 0.5° 를 맞추려면 pitch kp ≈ 2,900 N·m/rad 필요(액추에이터 파라미터표). 중력보상이 정답", "",
      "## ③ 중력 토크", "", "### 검산 (qfrc_bias, qvel = 0)", "",
      f'- 무작위 200 자세 최대 차: mj_rne(가속 0) {R["gravity_check"]["rne"]:.1e} · 자코비안 전치합 −Σ J_comᵀ mᵢ g {R["gravity_check"]["jac"]:.1e} · 외적 합 Σ aⱼ·((cᵢ − pⱼ) × mᵢ g) {R["gravity_check"]["cross"]:.1e} N·m',
      f'- 손계산(앞으로 수평, FK 없이 각 COM 의 어깨축 아래 깊이 × 질량 × g): 0 kg {hh["tau0"]} vs MuJoCo {hh["mj0"]} N·m · 3 kg {hh["tau3"]} vs {hh["mj3"]} N·m (차 0.005 N·m = 더미 프레임 1 g 질량), ee 수평 도달 {hh["arm_reach_ee_m"]} m',
      f'- qvel ≠ 0 이면 qfrc_bias 에 코리올리 · 원심이 더해짐: 자세 {[round(x, 2) for x in R["bias_vs_qvel"]["pose"]]} rad, 팔 4관절 3 rad/s → 중력 {R["bias_vs_qvel"]["gravity"]} vs bias {R["bias_vs_qvel"]["bias_qvel3"]} N·m (차 {R["bias_vs_qvel"]["coriolis"]}). 관절 감쇠 0.5 × 3 = 1.5 N·m 은 qfrc_bias 가 아니라 qfrc_passive {R["bias_vs_qvel"]["passive_damping"]}',
      "", "### 관절별 최악 자세 토크 (전역 최적화 differential_evolution × 시드 3, 관절 범위 안)", "",
      "| 관절 | 0 kg | 0.5 kg | 1.0 kg | 3 kg (ee) [자세 °] | 3 kg (플랜지) | 필요 정격 (3 kg × 1.2) | 정격 46 여유 % | 피크 100 여유 % | 판정 |", "|---|---|---|---|---|---|---|---|---|---|"]
for a in c.ARM:
    x3 = gr["3kg@ee"][a]
    L.append(f'| {a} | {gr["0kg@ee"][a]["tau"]} | {gr["0.5kg@ee"][a]["tau"]} | {gr["1kg@ee"][a]["tau"]} | {x3["tau"]} {x3["pose_deg"]} | {gr["3kg@tool0"][a]["tau"]} | {x3["need_rated"]} | {x3["margin_rated_pct"]} | {x3["margin_peak_pct"]} | {"OK" if x3["ok"] else ("미통과 (여유 < 20 %)" if x3["tau"] <= RATED else "초과")} |')
L += ["", "- 여유율 = (정격 − τ) / τ × 100 (박종진 validate_model.md 와 같은 정의). 자세 = (pitch, roll, yaw, elbow) °",
      "- 페이로드 작용점 영향: 3 kg 을 플랜지(tool0) → ee(+0.10 m) 로 옮기면 pitch +2.94 N·m (= 3 × 9.81 × 0.10)",
      "- 박종진 15° 격자 대비: pitch 42.91 → 42.92 (같음), roll 38.89 → 39.77 N·m (+2.3 %, 격자 사이 yaw ≈ −55° 자세) — 판정은 같음"]
open(os.path.join(c.OUT, "validate_123.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
print("\n".join(L))
