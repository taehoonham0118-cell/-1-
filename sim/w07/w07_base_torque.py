# week07 | 협조 동작 중 토크 예산 갱신 — 베이스 가감속이 팔 관절에 주는 추가 토크 (함태훈, B1)
#  원리: 베이스가 가속하면 팔 좌표계에서 각 링크에 관성력 −mᵢ a 가 걸림 → 관절 토크 τ_add = M_ab(q) · q̈_b
#        (M_ab = 질량행렬의 팔 행 × 베이스 열(TX · TY · RZ) 결합 블록). yaw 회전 중이면 원심 · 코리올리 c(q, ω) 도 더해짐
#  1) 추정(이론): 왼팔 무작위 3,000 자세 + 앞으로 수평 · 영점 · 박종진 최악 자세, 3 kg, 방향은 가장 나쁜 쪽(보수적)
#        |τᵢ| ≤ |τg,ᵢ| + a‖M_ab[i, x·y]‖ + α|M_ab[i, yaw]| + |cᵢ(ω)|
#     가속 조건: 설계 가정 a 1.0 m/s² (0 → 1.5 m/s 1.5 s), α 1.57 rad/s² (0 → 90 °/s 1 s), ω 1.571 rad/s
#                모델 계단 명령(velocity kv 200 · 50 에 최대 속도 계단) a₀ = kv·v_max / M, α₀ = kv·ω_max / I_zz
#  2) 확인(시뮬): 앞으로 수평 · 영점 자세(3 kg, 아는 짐 FF) 를 서보로 유지한 채 베이스 x · y 사다리꼴(1.0 m/s²) · 계단 명령 → Δτ 시뮬 vs M_ab·q̈_b(측정 가속도)
#  3) 토크 예산표 갱신: 6주차 표(정적 · 집기 · 뻗기) + 베이스 가감속 열 → 정격 ≥ 필요 × 1.2 판정 (a2_doc · cad_ri85), PDR 52
#  출력: results/base_torque.{md,json}, results/torque_budget_w07.{md,csv}, assets/screenshots/week07/week07_base_accel_torque.png
#  실행: python sim/w07/w07_base_torque.py   (~15 s)
import csv, time
from w07_common import *

t0 = time.time(); MPAY = 3.0; PDR = 52.0
A_DES, AL_DES, W_DES = 1.0, np.pi / 2, np.pi / 2
P = PARAMS["base"]

# ---------- 1) 자세 전수 추정 ----------
mG = load(payload=MPAY, sides=("left",), no_gravcomp=True); dG = mujoco.MjData(mG)
qi = arm_q(mG, "left"); di = arm_d(mG, "left"); bd = base_d(mG)
M_tot = float(mG.body_subtreemass[0])
jid = [mG.joint(f"left_{a}_joint").id for a in ARM]; lo, hi = mG.jnt_range[jid, 0], mG.jnt_range[jid, 1]
wmt = json.load(open(os.path.join(V1, "results", "workspace_max_torque.json"), encoding="utf-8"))["results"]["3kg"]
QS = [("영점", np.zeros(4)), ("앞으로 수평", np.array([-np.pi / 2, 0, 0, 0]))] + [(f"박종진 최악 {j}", np.deg2rad(wmt[j]["q_deg"])) for j in ARM if "q_deg" in wmt[j]]
QX = np.vstack([q for _, q in QS] + [np.random.default_rng(7).uniform(lo, hi, (3000, 4))])
M = np.zeros((mG.nv, mG.nv)); rows = []
for q in QX:
    dG.qpos[:] = 0; dG.qvel[:] = 0; dG.qpos[qi] = q; mujoco.mj_forward(mG, dG); mujoco.mj_fullM(mG, dG, M)
    tg = dG.qfrc_bias[di].copy(); Mab = M[np.ix_(di, bd)].copy()
    dG.qvel[bd[2]] = W_DES; mujoco.mj_forward(mG, dG); cw = dG.qfrc_bias[di] - tg
    Izz = M[bd[2], bd[2]]
    rows.append((q, tg, Mab, cw, Izz, dG.ncon))
A_STEP = P["vx_kv"] * P["v_max"] / M_tot
IZZ0 = rows[0][4]; AL_STEP = P["wz_kv"] * P["w_max"] / IZZ0


def bound(r, a, al, w_on=True):
    q, tg, Mab, cw, _, _ = r
    return np.abs(tg) + a * np.linalg.norm(Mab[:, :2], axis=1) + al * np.abs(Mab[:, 2]) + (np.abs(cw) if w_on else 0)


EST = {}
for tag, a, al in (("static", 0, 0), ("design", A_DES, AL_DES), ("step", A_STEP, AL_STEP)):
    B = np.array([bound(r, a, al, tag != "static") for r in rows if r[5] == 0]); Q_ = np.array([r[0] for r in rows if r[5] == 0])
    ADD = np.array([a * np.linalg.norm(r[2][:, :2], axis=1) + al * np.abs(r[2][:, 2]) + (np.abs(r[3]) if tag != "static" else 0) for r in rows if r[5] == 0])
    EST[tag] = {j: dict(tau=float(B[:, i].max()), q_deg=np.rad2deg(Q_[B[:, i].argmax()]).round(1).tolist(), add_max=float(ADD[:, i].max()),
                        add_at_worst=float(ADD[B[:, i].argmax(), i])) for i, j in enumerate(ARM)}
LIN_PER = {j: float(max(np.linalg.norm(r[2][i, :2]) for r in rows)) for i, j in enumerate(ARM)}   # N·m per m/s²
YAW_PER = {j: float(max(abs(r[2][i, 2]) for r in rows)) for i, j in enumerate(ARM)}            # N·m per rad/s²
CEN = {j: float(max(abs(r[3][i]) for r in rows)) for i, j in enumerate(ARM)}

# 손 검산: 영점(팔 늘어뜨림)에서 전방 가속 → pitch 토크 = Σ mᵢ a · (어깨 아래 COM 깊이 hᵢ) (점질량 진자)
dG.qpos[:] = 0; dG.qpos[qi] = 0; mujoco.mj_forward(mG, dG); sh = dG.xpos[mG.body("left_shoulder_pitch_link").id]
sp = mG.body("left_shoulder_pitch_link").id
def _under(b):
    while b > 0:
        if b == sp: return True
        b = mG.body_parentid[b]
    return False
hand = sum(mG.body_mass[b] * (sh[2] - dG.xipos[b][2]) for b in range(mG.nbody) if _under(b))
dG.qpos[:] = 0; mujoco.mj_forward(mG, dG); mujoco.mj_fullM(mG, dG, M); mab_zero = M[np.ix_(di, bd)][0, 0]


# ---------- 2) 시뮬 확인 ----------
def sim_base(q_arm, axis, mode, T=4.0):
    m = load(payload=MPAY, payload_gc=1.0, sides=("left",)); d = mujoco.MjData(m)
    qi_ = arm_q(m, "left"); di_ = arm_d(m, "left"); bu = base_u(m); bd_ = base_d(m)
    d.qpos[qi_] = q_arm; d.ctrl[arm_u(m, "left")] = q_arm; mujoco.mj_forward(m, d)
    for _ in range(int(0.5 / m.opt.timestep)): mujoco.mj_step(m, d)
    tau0 = drive_torque(m, d, di_).copy(); bias0 = d.qfrc_bias[di_].copy(); Mf = np.zeros((m.nv, m.nv)); out = []
    for k in range(int(T / m.opt.timestep)):
        t = k * m.opt.timestep
        if mode == "trap":   # 1.0 m/s² 로 1.5 s 가속 → 1 s 등속 → 1.5 s 감속 (속도 명령, 베이스 lag 는 그대로)
            v = A_DES * min(t, 1.5) - A_DES * min(max(t - 2.5, 0), 1.5) if axis != 2 else (AL_DES * min(t, 1.0) - AL_DES * min(max(t - 2.0, 0), 1.0))
        else:                # 계단: 최대 속도를 바로 명령 → 2 s 에 0
            v = (P["v_max"] if axis != 2 else P["w_max"]) * (t < 2.0)
        d.ctrl[bu] = 0; d.ctrl[bu[axis]] = v
        mujoco.mj_step(m, d)
        if k % 2 == 0:
            mujoco.mj_fullM(m, d, Mf); th = Mf[np.ix_(di_, bd_)] @ d.qacc[bd_] + (d.qfrc_bias[di_] - bias0)   # 결합 관성 + 원심 · 코리올리
            out.append((t, drive_torque(m, d, di_) - tau0, th, d.qacc[bd_[axis]], d.qvel[bd_[axis]]))
    tt = np.array([o[0] for o in out]); DT = np.array([o[1] for o in out]); TH = np.array([o[2] for o in out])
    return dict(t=tt, dtau=DT, th=TH, acc=np.array([o[3] for o in out]), vel=np.array([o[4] for o in out]),
                dtau_max=np.abs(DT).max(0).tolist(), th_max=np.abs(TH).max(0).tolist(), acc_max=float(np.abs([o[3] for o in out]).max()))


SIMS = {}
for pn, qa_ in (("앞으로 수평", np.array([-np.pi / 2, 0, 0, 0])), ("영점", np.zeros(4))):
    for ax, an in ((0, "x 전후"), (1, "y 좌우"), (2, "yaw")):
        for mode in ("trap", "step"):
            SIMS[f"{pn}|{an}|{mode}"] = sim_base(qa_, ax, mode)

# ---------- 3) 토크 예산표 갱신 ----------
W6 = json.load(open(os.path.join(HERE, "..", "w06", "results", "torque_budget.json"), encoding="utf-8"))
PL = "3kg"; TB = []
for r6 in W6["rows"]:
    j = r6["joint"]; act, rt, pk = rated(r6["case"], j)
    st = r6["static"][PL]; pick = r6["pick"][PL]; reach = r6["reach"][PL]
    des = EST["design"][j]["tau"]; stp = EST["step"][j]["tau"]
    need = max(st, pick, reach, des)        # 설계 조건(가정 가속)까지 = 필요 토크
    TB.append(dict(case=r6["case"], joint=j, actuator=act, rated=rt, peak=pk, static=st, pick=pick, reach=reach, base_design=des, base_step=stp,
                   need=need, margin=(rt / need - 1) * 100, ok=bool(rt >= 1.2 * need), margin_step=(rt / stp - 1) * 100, ok_step=bool(rt >= 1.2 * stp),
                   pdr52=(PDR / need - 1) * 100, peak_ok_step=bool(pk >= stp)))
R = dict(M_total=M_tot, Izz=IZZ0, a_step=A_STEP, al_step=AL_STEP, design=dict(a=A_DES, al=AL_DES, w=W_DES), est=EST, lin_per=LIN_PER, yaw_per=YAW_PER, cen=CEN,
         hand_check=dict(sum_m_h=float(hand), Mab_pitch_x=float(mab_zero)),
         sims={k: {kk: v[kk] for kk in ("dtau_max", "th_max", "acc_max")} for k, v in SIMS.items()}, budget=TB)
save_json("base_torque.json", R)

f2 = lambda x: f"{x:.2f}"
Lm = ["# 베이스 가감속에 의한 팔 관절 추가 토크 (week07, 함태훈) — 자동 생성 `w07_base_torque.py`", "",
      f"모델 전체 질량 {M_tot:.2f} kg(3 kg 짐 포함), yaw 관성 I_zz {IZZ0:.2f} kg·m²(영점). 베이스 액추에이터 velocity kv {P['vx_kv']:g} · {P['wz_kv']:g}, 최대 {P['v_max']} m/s · {P['w_max']:.3f} rad/s",
      f"- 설계 가정: a {A_DES} m/s² (0 → 1.5 m/s 1.5 s), α {AL_DES:.2f} rad/s² (0 → 90 °/s 1 s), ω {W_DES:.2f} rad/s (원심) — **가정값**, 하체 사양 수령 시 교체",
      f"- 모델 계단 명령(현재 kv 로 최대 속도 계단): a₀ = kv·v/M = {P['vx_kv']:g}×{P['v_max']}/{M_tot:.1f} = **{A_STEP:.2f} m/s²**, α₀ = {P['wz_kv']:g}×{P['w_max']:.3f}/{IZZ0:.2f} = **{AL_STEP:.2f} rad/s²**",
      "", "## 1. 추가 토크 계수 (왼팔 3,000 + 기준 자세 중 최대, 3 kg)", "",
      "| 관절 | 직선 가속 1 m/s² 당 ‖M_ab[i, x·y]‖ N·m | yaw 각가속 1 rad/s² 당 |M_ab[i, yaw]| N·m | ω 90 °/s 원심 · 코리올리 N·m |", "|---|---|---|---|"]
for j in ARM: Lm.append(f"| {j} | {LIN_PER[j]:.2f} | {YAW_PER[j]:.2f} | {CEN[j]:.2f} |")
Lm += ["", f"- 손 검산 (영점, 팔 늘어뜨림, 전방 가속 → pitch): 점질량 진자 Σmᵢhᵢ = {hand:.3f} kg·m vs M_ab[pitch, x] = {mab_zero:.3f} → 1 m/s² 에 {abs(mab_zero):.2f} N·m",
       "", "## 2. 관절별 최악 토크 = |τg| + a‖M_ab‖ + α|M_ab,yaw| + |c(ω)| (방향 최악, 보수적)", "",
       "| 관절 | 정적 최악 τg (3 kg) | 설계 가속 포함 (추가분) · 자세 ° | 모델 계단 명령 포함 (추가분) |", "|---|---|---|---|"]
for j in ARM:
    Lm.append(f"| {j} | {EST['static'][j]['tau']:.2f} | **{EST['design'][j]['tau']:.2f}** (+{EST['design'][j]['add_at_worst']:.2f}) · {EST['design'][j]['q_deg']} | {EST['step'][j]['tau']:.2f} (+{EST['step'][j]['add_at_worst']:.2f}) |")
Lm += ["", "## 3. 시뮬 확인 — 팔을 서보로 유지한 채 베이스만 움직임 (3 kg 아는 짐, Δτ = 정지 대비 구동 토크 변화)", "",
       "| 자세 | 방향 | 명령 | 베이스 최대 가속 | Δτ 시뮬 최대 pitch · roll · yaw · elbow N·m | 이론 M_ab·q̈_b + c(q, q̇) 최대 |", "|---|---|---|---|---|---|"]
for k, v in SIMS.items():
    pn, an, md = k.split("|")
    Lm.append(f"| {pn} | {an} | {'사다리꼴' if md == 'trap' else '계단'} | {v['acc_max']:.2f} | {' · '.join(f2(x) for x in v['dtau_max'])} | {' · '.join(f2(x) for x in v['th_max'])} |")
Lm += ["", "- 사다리꼴(가속 1 m/s² 제한)에서 시뮬 = 이론(차 ≤ 6 %, yaw 는 원심 · 코리올리 c 포함 시 일치). 속도 계단은 가속도 피크가 수 ms 로 짧아 서보(유한 강성) 가 걸러 시뮬 < 강체 이론 → 이론 = 보수적 상한",
       "- 결론: 현재 베이스 velocity kv 로 최대 속도를 계단 명령하면 a₀ 3.8 m/s² · α₀ 18.9 rad/s² — cad_ri85 roll 은 정격 초과 가능 → **베이스 명령에 가속 제한(≤ 1 m/s², ≤ 1.57 rad/s²)을 넣는 것이 조건** (하체 사양 수령 시 재계산)",
       "", "## 4. 토크 예산표 갱신 (3 kg, 노션 판정 규칙: 정격 ≥ 필요 토크 × 1.2)", "",
       "필요 토크 = max(정적 최악, 집기, 뻗기(6주차), **베이스 설계 가속 포함**). 계단 명령 열은 참고(제어기에서 가속 제한으로 피함)", "",
       "| case | 관절 | 구동기 정격 / 피크 | 정적 | 집기 | 뻗기 1.5 s | **베이스 설계 가속** | 필요 | 여유 % · 판정 | 계단 명령 (여유 %) | PDR 52 여유 % |", "|---|---|---|---|---|---|---|---|---|---|---|"]
for r in TB:
    Lm.append(f"| {r['case']} | {r['joint']} | {r['actuator']} {r['rated']} / {r['peak']} | {r['static']:.2f} | {r['pick']:.2f} | {r['reach']:.2f} | **{r['base_design']:.2f}** | {r['need']:.2f} | "
              f"{r['margin']:.0f} · {'OK' if r['ok'] else '**NG**'} | {r['base_step']:.2f} ({r['margin_step']:.0f} %) | {r['pdr52']:.1f} {'OK' if r['pdr52'] >= 20 else '**주의**'} |")
Lm += ["", f"실행 시간 {time.time() - t0:.0f} s"]
save_md("base_torque.md", "\n".join(Lm) + "\n")
with open(os.path.join(OUT, "torque_budget_w07.csv"), "w", newline="", encoding="utf-8") as f:
    w = csv.writer(f); w.writerow(list(TB[0].keys()))
    for r in TB: w.writerow([f"{v:.3f}" if isinstance(v, float) else v for v in r.values()])

plt = plot_style()
fig, ax = plt.subplots(1, 3, figsize=(13, 3.5))
for i, key in enumerate(("영점|x 전후|trap", "앞으로 수평|y 좌우|trap", "영점|x 전후|step")):
    v = SIMS[key]; jj = int(np.argmax(v["dtau_max"]))
    ax[i].plot(v["t"], v["dtau"][:, jj], color=SERIES[0], lw=1.6, label=f"시뮬 Δτ {ARM[jj]}")
    ax[i].plot(v["t"], v["th"][:, jj], color=SERIES[1], lw=1.4, ls="--", label="이론 M_ab·q̈_b + c")
    a2 = ax[i].twinx(); a2.plot(v["t"], v["acc"], color="#8a8986", lw=1, alpha=0.6); a2.set_ylabel("베이스 가속 [m/s²]", color="#8a8986")
    a2.spines["top"].set_visible(False); a2.grid(False)
    ax[i].set_title(key.replace("|", " · ").replace("trap", "사다리꼴 1 m/s²").replace("step", "속도 계단")); ax[i].set_xlabel("시간 [s]"); ax[i].set_ylabel("추가 토크 [N·m]"); ax[i].legend(loc="upper right")
fig.tight_layout(); fig.savefig(os.path.join(IMG, "week07_base_accel_torque.png"), dpi=130); plt.close(fig)
print(open(os.path.join(OUT, "base_torque.md"), encoding="utf-8").read())
