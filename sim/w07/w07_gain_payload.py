# week07 | 게인 통일 확인 + 검증 ② 처짐 재기록 + 짐 0 · 3 kg 계단응답 + 짐 질량 중력보상 비교 (함태훈, B1)
#  모델: w07_common (게인 통일 빌드: pitch 400·31 / roll 350·27.5 / yaw 300·15 / elbow 300·11, a2_doc, 중력보상 B on)
#  1) 검증 ② 정지 유지 재기록: 영점 · 앞으로 수평 · roll 상한 × (중력보상 끔 / 켬) × (week05 kp 300 / 통일 게인), 이론 τg / kp 와 대조
#  2) 짐 0 · 3 kg 계단응답(0.2 rad, 영점 + 앞으로 수평 + 무작위 300 자세): 모르는 짐 / 아는 짐(FF) — OS · 2 % 정착 · ζ.
#     3 kg 대책 = 짐 질량을 알 때 관성 비례 게인 스케줄(kp · kv × Mᵢᵢ(3 kg) / Mᵢᵢ(0 kg)) 비교
#  3) 짐 질량 중력보상 5 방식: 앞으로 수평 자세에서 0.5 s 에 3 kg 잡기 → 3.5 s 에 놓기
#     U 모르는 짐 / K 아는 짐 FF / W2 · W4 질량 틀림(2 · 4 kg 로 가정) / I 적분항(PI) / E 질량 추정(정지 구간 PD 토크 최소자승) + 놓은 뒤 처리
#  출력: results/gain_payload.{md,json}, assets/screenshots/week07/week07_payload_comp.png · week07_step_0_3kg.png
#  실행: python sim/w07/w07_gain_payload.py   (~40 s)
import time
from w07_common import *

t0 = time.time(); STEP = 0.2; TSIM = 1.0; OS_MAX = 5.0; TS_MAX = 0.30; MPAY = 3.0
POSE = {"zero": np.zeros(4), "forward": np.array([-np.pi / 2, 0, 0, 0]), "roll_upper": None}
R = {"gains": {j: dict(kp=kp, kv=kv) for j, (kp, kv) in GAINS.items()}}


# ---------- 1) 검증 ② 재기록 ----------
def hold(m, q0, T=5.0):
    d = mujoco.MjData(m); qi = arm_q(m, "left"); u = arm_u(m, "left"); qr = arm_q(m, "right"); ur = arm_u(m, "right")
    d.qpos[qi] = q0; d.qpos[qr] = q0; d.ctrl[u] = q0; d.ctrl[ur] = q0; mujoco.mj_forward(m, d)
    for _ in range(int(T / m.opt.timestep)): mujoco.mj_step(m, d)
    return np.rad2deg(np.abs(d.qpos[qi] - q0))


def tau_g(m, q0):   # 왼팔 관절 중력 토크 (보상 없는 모델에서 qfrc_bias, q̇ = 0)
    d = mujoco.MjData(m); d.qpos[arm_q(m, "left")] = q0; d.qpos[arm_q(m, "right")] = q0; mujoco.mj_forward(m, d)
    return d.qfrc_bias[arm_d(m, "left")].copy()


mU = load(no_gravcomp=True)
lo_hi = mU.jnt_range[[mU.joint(f"left_{a}_joint").id for a in ARM]]
POSE["roll_upper"] = np.array([0, lo_hi[1, 1], 0, 0])
OLD = {j: (300.0, 20.0) for j in ARM}          # week05 (kp 300, 처짐엔 kv 무관)
KP = np.array([GAINS[j][0] for j in ARM])
hold_rows = []
for gname, gains in (("week05 kp 300", OLD), ("통일 게인", None)):
    for gc in (False, True):
        m = load(no_gravcomp=not gc, gains=gains)
        kp = np.array([m.actuator_gainprm[m.actuator(f"left_{a}").id, 0] for a in ARM])
        for pn, q0 in POSE.items():
            tg = tau_g(mU, q0); th = np.rad2deg(np.abs(np.linalg.solve(np.diag(kp), tg))) if not gc else np.zeros(4)
            sim = hold(m, q0)
            hold_rows.append(dict(gains=gname, gravcomp=gc, pose=pn, tau_g=tg.tolist(), theory_deg=th.tolist(), sim_deg=sim.tolist(),
                                  theory_max=float(th.max()), sim_max=float(sim.max()), joint=ARM[int(np.argmax(sim))] if sim.max() > 1e-6 else "-"))
R["hold"] = hold_rows


# ---------- 2) 짐 0 · 3 kg 계단응답 ----------
def step_resp(m, q0, j, amp):
    d = mujoco.MjData(m); qi = arm_q(m, "left"); u = arm_u(m, "left")
    d.qpos[qi] = q0; d.ctrl[u] = q0; mujoco.mj_forward(m, d)
    # 짐이 모르는 짐이면 처진 위치에서 출발 (정상상태 → 계단): 1.5 s 먼저 유지
    for _ in range(int(1.5 / m.opt.timestep)): mujoco.mj_step(m, d)
    qs = d.qpos[qi[j]]; d.ctrl[u[j]] = q0[j] + amp
    n = int(TSIM / m.opt.timestep); y = np.zeros(n); nc = 0
    for k in range(n):
        mujoco.mj_step(m, d); y[k] = d.qpos[qi[j]] - qs; nc = max(nc, d.ncon)
    y = y / amp; tt = (np.arange(n) + 1) * m.opt.timestep
    # 모르는 짐이면 최종값이 1 이 아니라 처짐만큼 다름 → 최종값 기준으로 OS · 정착 (처짐은 따로 표)
    yf = y[-50:].mean(); os_ = max(0.0, (y.max() - yf) / yf * 100); out = np.where(np.abs(y - yf) > 0.02 * abs(yf))[0]
    ts = tt[out[-1]] if len(out) and out[-1] < n - 1 else (0.0 if not len(out) else np.inf)
    return os_, ts, nc, tt, y


def mii(m, q0, i):
    d = mujoco.MjData(m); d.qpos[arm_q(m, "left")] = q0; mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, M); return M[arm_d(m, "left")[i], arm_d(m, "left")[i]]


def set_gain(m, i, kp, kv):
    u = arm_u(m, "left")[i]; m.actuator_gainprm[u, 0] = kp; m.actuator_biasprm[u, 1] = -kp; m.actuator_biasprm[u, 2] = -kv


M0 = load(); M3u = load(payload=MPAY, sides=("left",)); M3k = load(payload=MPAY, payload_gc=1.0, sides=("left",))
QX = np.vstack([POSE["zero"], POSE["forward"], np.random.default_rng(1).uniform(lo_hi[:, 0], lo_hi[:, 1], (300, 4))])
step_rows = []; curves = {}
for i, j in enumerate(ARM):
    kp, kv = GAINS[j]
    for k, q0 in enumerate(QX):
        amp = STEP if q0[i] + STEP <= lo_hi[i, 1] else -STEP
        m0 = mii(M0, q0, i); m3 = mii(M3u, q0, i)
        rec = dict(joint=j, k=k, q0=q0.tolist(), M0=m0, M3=m3, zeta0=kv / (2 * np.sqrt(kp * m0)), zeta3=kv / (2 * np.sqrt(kp * m3)))
        for tag, mm in (("0kg", M0), ("3kg_unknown", M3u), ("3kg_known", M3k), ("3kg_sched", M3k)):
            if tag == "3kg_sched": set_gain(mm, i, kp * m3 / m0, kv * m3 / m0)
            os_, ts, nc, tt, y = step_resp(mm, q0.copy(), i, amp)
            if tag == "3kg_sched": set_gain(mm, i, kp, kv)
            rec[tag] = dict(os=os_, ts=ts, nc=nc)
            if k == 1: curves[(j, tag)] = (tt, y)
        step_rows.append(rec)


def worst(tag, j):
    r = [x for x in step_rows if x["joint"] == j and max(x[t]["nc"] for t in ("0kg", "3kg_unknown", "3kg_known", "3kg_sched")) == 0]
    ko = max(r, key=lambda x: x[tag]["os"]); kt = max(r, key=lambda x: x[tag]["ts"])
    return dict(os=ko[tag]["os"], ts=kt[tag]["ts"], n=len(r), zeta_min=min(x["zeta3" if tag != "0kg" else "zeta0"] for x in r))


STEPW = {j: {t: worst(t, j) for t in ("0kg", "3kg_unknown", "3kg_known", "3kg_sched")} for j in ARM}
FWD = {j: next(x for x in step_rows if x["joint"] == j and x["k"] == 1) for j in ARM}
R["step"] = dict(worst=STEPW, forward=FWD, n_poses=len(QX))


# ---------- 3) 짐 질량 중력보상 비교 (잡기 0.5 s → 놓기 3.5 s, 앞으로 수평 왼팔) ----------
T_GRASP, T_REL, T_END = 0.5, 3.5, 5.5; KI_TI = 0.5     # 적분 시간 Ti = 0.5 s (Ki = kp / Ti)
EST_WIN = (0.9, 1.1)                                  # 추정 창: 잡고 0.4 ~ 0.6 s (정착 후)
q_hold = POSE["forward"]


def run_comp(method):
    gc = {"K": 1.0, "W2": 2 / 3, "W4": 4 / 3}.get(method, 0.0)
    m = load(payload=MPAY, payload_gc=gc, sides=("left",)); set_payload(m, "left", 0)
    d = mujoco.MjData(m); qi = arm_q(m, "left"); di = arm_d(m, "left"); u = arm_u(m, "left"); bp = m.body("left_payload").id
    d.qpos[qi] = q_hold; d.ctrl[u] = q_hold; mujoco.mj_forward(m, d)
    n = int(T_END / m.opt.timestep); dt = m.opt.timestep; integ = np.zeros(4); m_hat = None; buf = []
    log = dict(t=[], err=[], ee=[], tau=[]); ee0 = d.site_xpos[m.site("left_ee").id].copy(); jac = np.zeros((3, m.nv))
    for k in range(n):
        t = k * dt
        if abs(t - T_GRASP) < dt / 2: set_payload(m, "left", MPAY)
        if abs(t - T_REL) < dt / 2: set_payload(m, "left", 0)
        e = q_hold - d.qpos[qi]
        if method == "I":
            integ += e * dt; d.ctrl[u] = q_hold + integ / KI_TI      # ctrl 오프셋 = (Ki / kp) ∫e, Ki = kp / Ti
        if method == "E":
            mujoco.mj_jacBodyCom(m, d, jac, None, bp); Jz = jac[2, di]
            if EST_WIN[0] <= t < EST_WIN[1]: buf.append((d.actuator_force[u].copy(), Jz.copy()))
            if t >= EST_WIN[1] and m_hat is None:   # τ_pd = m g J_zᵀ (정지) → 최소자승 m̂
                tp = np.array([b[0] for b in buf]).ravel(); A_ = np.array([b[1] for b in buf]).ravel() * G
                m_hat = float(A_ @ tp / (A_ @ A_))
            d.qfrc_applied[di] = (m_hat or 0.0) * G * Jz          # 추정 질량 FF (놓은 뒤에도 그대로 = 놓기 감지 없음)
        mujoco.mj_step(m, d)
        if k % 5 == 0:
            log["t"].append(t); log["err"].append(np.rad2deg(e).tolist())
            log["ee"].append(((d.site_xpos[m.site("left_ee").id] - ee0) * 1e3).tolist()); log["tau"].append(drive_torque(m, d, di).tolist())
    L = {k: np.array(v) for k, v in log.items()}; tt = L["t"]
    hold_ = (tt > T_REL - 0.5) & (tt < T_REL); after = tt >= T_REL; grasp = (tt >= T_GRASP) & (tt < T_REL)
    dz = L["ee"][:, 2]
    rec = dict(method=method, ss_drop_mm=float(-dz[hold_].mean()), ss_err_deg=float(np.abs(L["err"][hold_]).max()),
               grasp_peak_drop_mm=float(-dz[grasp].min()), release_peak_mm=float(np.abs(dz[after]).max()),
               final_mm=float(dz[-1]), m_hat=m_hat)
    # 놓은 뒤 0.5 mm 안으로 돌아오는 시간
    a_idx = np.where(after)[0]; outside = a_idx[np.abs(dz[a_idx]) > 0.5]
    rec["recover_s"] = float(tt[outside[-1]] - T_REL) if len(outside) and outside[-1] < len(tt) - 1 else (0.0 if not len(outside) else None)
    return rec, L


COMP = {}; LOGS = {}
for meth in ("U", "K", "W2", "W4", "I", "E"):
    COMP[meth], LOGS[meth] = run_comp(meth)
# 이론: 모르는 짐 정상상태 Δq = K⁻¹ Jᵀ (m g),  끝점 처짐 = J_p Δq
m_ = load(payload=MPAY, sides=("left",)); d_ = mujoco.MjData(m_); d_.qpos[arm_q(m_, "left")] = q_hold; mujoco.mj_forward(m_, d_)
jac = np.zeros((3, m_.nv)); mujoco.mj_jacBodyCom(m_, d_, jac, None, m_.body("left_payload").id); Jp = jac[:, arm_d(m_, "left")]
dq_th = np.linalg.solve(np.diag(KP), Jp.T @ np.array([0, 0, MPAY * G]))
TH = dict(dq_deg=np.rad2deg(dq_th).tolist(), drop_mm=float(-(Jp @ -dq_th)[2] * 1e3) if False else float(abs((Jp @ dq_th)[2]) * 1e3),
          tau_payload=(Jp.T @ np.array([0, 0, MPAY * G])).tolist())
# 질량 틀림 이론 = (1 − m̂/m) × 모르는 짐 처짐 · 적분 놓기 오버슈트 이론 = 같은 크기 반대 방향(적분이 m g Jᵀ 를 기억)
TH["W2_mm"] = TH["drop_mm"] * (1 - 2 / 3); TH["W4_mm"] = TH["drop_mm"] * (1 - 4 / 3)
R["comp"] = dict(results=COMP, theory=TH, Ti=KI_TI, est_window=EST_WIN, pose_deg=np.rad2deg(q_hold).tolist(), payload=MPAY)

# ---------- 출력 ----------
f2 = lambda x: f"{x:.2f}"; f1 = lambda x: f"{x:.1f}"
Lm = ["# 게인 통일 · 짐 질량 중력보상 (week07, 함태훈) — 자동 생성 `w07_gain_payload.py`", "",
      f"모델: `sim/w07/b1_snapshot/v1` (박종진 d6d6304 + 게인 통일 패치 빌드), a2_doc, mujoco {mujoco.__version__}, dt 0.002 s",
      "", "## 0. 통일 게인 (`actuator_params.json` `joint_gains`, 고정 kp · kv = Isaac stiffness · damping)", "",
      "| 관절 | kp [N·m/rad] | kv [N·m·s/rad] |", "|---|---|---|"] + [f"| {j} | {GAINS[j][0]:g} | {GAINS[j][1]:g} |" for j in ARM] + [
      "", "## 1. 검증 ② 정지 유지 재기록 (5 s, 왼팔 관절 최대 오차 °, 이론 = |K⁻¹ τg|)", "",
      "| 게인 | 중력보상 | 영점 이론 / 시뮬 | 앞으로 수평 이론 / 시뮬 | roll 상한 이론 / 시뮬 |", "|---|---|---|---|---|"]
for gname in ("week05 kp 300", "통일 게인"):
    for gc in (False, True):
        cells = []
        for pn in POSE:
            r = next(x for x in hold_rows if x["gains"] == gname and x["gravcomp"] == gc and x["pose"] == pn)
            cells.append(f"{r['theory_max']:.2f} / **{r['sim_max']:.2f}**")
        Lm.append(f"| {gname} | {'켬 (B)' if gc else '끔'} | " + " | ".join(cells) + " |")
Lm += ["", "- 보상 끔 앞으로 수평: τg(pitch) / kp → 300 일 때 4.85°, 400 일 때 3.64° (= 4.85 × 300/400). 시뮬은 다른 관절 결합으로 약간 작음. 보상 B 켜면 0.00°",
       "- 박종진 `validate_model.py` (패치 빌드, mujoco 3.17) ② 결과와 같은 값: 앞으로 수평 보상 없음 3.63° · B 0.0°, roll 상한 3.72° → B 0.0° (`b1_snapshot/v1/results/validate_model.md`)",
       "", f"## 2. 짐 0 · 3 kg 계단응답 (0.2 rad, 영점 + 앞으로 수평 + 무작위 300 자세, 계단 중 접촉 자세 제외 · 중력보상 B on)", "",
       "OS · 정착은 최종값 기준(모르는 짐은 처진 위치에서 출발해 처진 위치로 정착). 스케줄 = 짐 질량을 알 때 kp · kv × Mᵢᵢ(3 kg)/Mᵢᵢ(0 kg) (ω_n · ζ 를 0 kg 와 같게)", "",
       "| 관절 | ζ 최소 0 → 3 kg | 0 kg OS % / 정착 s | 3 kg 모르는 짐 | 3 kg 아는 짐(FF) | 3 kg FF + 게인 스케줄 | R3 (≤ 5 % · ≤ 0.30 s) 3 kg FF |", "|---|---|---|---|---|---|---|"]
for j in ARM:
    w = STEPW[j]; ok = w["3kg_known"]["os"] <= OS_MAX and w["3kg_known"]["ts"] <= TS_MAX + 1e-9
    Lm.append(f"| {j} | {w['0kg']['zeta_min']:.2f} → {w['3kg_known']['zeta_min']:.2f} | {f2(w['0kg']['os'])} / {w['0kg']['ts']:.3f} | "
              f"{f2(w['3kg_unknown']['os'])} / {w['3kg_unknown']['ts']:.3f} | {f2(w['3kg_known']['os'])} / {w['3kg_known']['ts']:.3f} | "
              f"{f2(w['3kg_sched']['os'])} / {w['3kg_sched']['ts']:.3f} | {'통과' if ok else '**미통과**'} |")
Lm += ["", "앞으로 수평 자세(pitch −90°) 한 자세만:", "", "| 관절 | Mᵢᵢ 0 → 3 kg [kg·m²] | ζ 0 → 3 kg | 0 kg OS / 정착 | 3 kg FF OS / 정착 | 3 kg 스케줄 OS / 정착 |", "|---|---|---|---|---|---|"]
for j in ARM:
    x = FWD[j]
    Lm.append(f"| {j} | {x['M0']:.3f} → {x['M3']:.3f} | {x['zeta0']:.2f} → {x['zeta3']:.2f} | {f2(x['0kg']['os'])} / {x['0kg']['ts']:.3f} | {f2(x['3kg_known']['os'])} / {x['3kg_known']['ts']:.3f} | {f2(x['3kg_sched']['os'])} / {x['3kg_sched']['ts']:.3f} |")
names = dict(U="U 모르는 짐 (FF 없음)", K="K 아는 짐 FF (gravcomp 1)", W2="W2 질량 틀림 (2 kg 로 가정)", W4="W4 질량 틀림 (4 kg 로 가정)",
             I=f"I 적분항 (Ti {KI_TI} s, anti-windup 없음)", E="E 질량 추정 (잡고 0.4 ~ 0.6 s PD 토크 최소자승) → FF")
Lm += ["", f"## 3. 짐 질량 중력보상 비교 — 왼팔 앞으로 수평, {T_GRASP} s 에 {MPAY:g} kg 잡기 · {T_REL} s 에 놓기", "",
       "| 방식 | 잡은 직후 최대 처짐 mm | 유지 정상상태 처짐 mm | 관절 오차 최대 ° | 놓은 뒤 최대 이탈 mm | 0.5 mm 복귀 s | 추정 m̂ kg |", "|---|---|---|---|---|---|---|"]
for k, r in COMP.items():
    Lm.append(f"| {names[k]} | {r['grasp_peak_drop_mm']:.1f} | **{r['ss_drop_mm']:.1f}** | {r['ss_err_deg']:.2f} | {r['release_peak_mm']:.1f} | "
              f"{'-' if r['recover_s'] is None else f'{r['recover_s']:.2f}'} | {'-' if r['m_hat'] is None else f'{r['m_hat']:.3f}'} |")
Lm += ["", f"- 이론 (모르는 짐 정상상태): Δq = K⁻¹ Jᵀ m g = {np.round(TH['dq_deg'], 2).tolist()}° → 끝점 처짐 {TH['drop_mm']:.1f} mm "
       f"(시뮬 {COMP['U']['ss_drop_mm']:.1f} mm, 차이 = 선형화 · 처진 자세에서 J 변화). 짐 토크 Jᵀ m g = {np.round(TH['tau_payload'], 2).tolist()} N·m",
       f"- 질량 틀림 이론 = (1 − m̂/m) × 처짐: 2 kg 가정 {TH['W2_mm']:.1f} mm 처짐, 4 kg 가정 {abs(TH['W4_mm']):.1f} mm 들림 (시뮬 {COMP['W2']['ss_drop_mm']:.1f} · {COMP['W4']['ss_drop_mm']:.1f})",
       "- 적분항: 오차를 0 으로 만들지만 놓는 순간 적분값이 m g Jᵀ 만큼 남아 팔이 같은 크기로 튀어 오르고 Ti 정도 걸려 돌아옴. 포화 · 접촉(물체가 테이블에 걸림)에서는 적분이 계속 커지는 와인드업 → 위험",
       "- 추정: 정지 구간에서만 정확(움직이는 중엔 관성 · 속도 항이 섞임). 놓기 감지가 없으면 FF 가 남아 놓은 뒤 팔이 들림 → 그리퍼 열림 신호로 m̂ = 0 리셋 필요",
       f"", f"실행 시간 {time.time() - t0:.0f} s"]
save_md("gain_payload.md", "\n".join(Lm) + "\n"); save_json("gain_payload.json", R)

# ---------- 그림 ----------
plt = plot_style()
fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
col = dict(U=SERIES[1], K=SERIES[0], W2=SERIES[3], W4=SERIES[4], I=SERIES[2], E=SERIES[5])
for k, Lg in LOGS.items():
    ax[0].plot(Lg["t"], Lg["ee"][:, 2], color=col[k], lw=1.6, label=names[k].split(" (")[0])
    ax[1].plot(Lg["t"], Lg["tau"][:, 0], color=col[k], lw=1.6)
for a in ax:
    a.axvline(T_GRASP, color="#8a8986", ls=":", lw=1); a.axvline(T_REL, color="#8a8986", ls=":", lw=1); a.set_xlabel("시간 [s]")
ax[0].set_ylabel("끝점 z 변화 [mm]"); ax[0].set_title(f"왼팔 앞으로 수평, {MPAY:g} kg 잡기 {T_GRASP} s · 놓기 {T_REL} s"); ax[0].legend(ncol=2)
ax[1].set_ylabel("shoulder_pitch 구동 토크 [N·m]"); ax[1].set_title("pitch 구동 토크 (PD + 보상 + FF)")
fig.tight_layout(); fig.savefig(os.path.join(IMG, "week07_payload_comp.png"), dpi=130); plt.close(fig)

fig, ax = plt.subplots(1, 4, figsize=(13, 3.2), sharey=True)
for i, j in enumerate(ARM):
    for tag, c, lab in (("0kg", SERIES[0], "0 kg"), ("3kg_known", SERIES[1], "3 kg (FF)"), ("3kg_sched", SERIES[2], "3 kg FF + 스케줄")):
        tt, y = curves[(j, tag)]; ax[i].plot(tt, y, color=c, lw=1.6, label=lab)
    ax[i].axhline(1.05, color="#8a8986", ls=":", lw=1); ax[i].set_title(f"{j} (앞으로 수평)"); ax[i].set_xlabel("시간 [s]"); ax[i].set_xlim(0, 0.6)
ax[0].set_ylabel("정규화 응답"); ax[0].legend()
fig.tight_layout(); fig.savefig(os.path.join(IMG, "week07_step_0_3kg.png"), dpi=130); plt.close(fig)
print(open(os.path.join(OUT, "gain_payload.md"), encoding="utf-8").read())
