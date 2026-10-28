# week06 | 관절별 kp · kv 튜닝 (함태훈, B1) — A2 armature 반영 모델(a2_doc 기본, cad_ri85 병기), 중력보상 B on
#  규칙 (함태훈 제안 — 팀 공용 기준 아님, 6주차 보고서에 명시):
#   R1 kp: 모르는 짐 1.0 kg 이 붙어도 정적 처짐 ≤ 1° (Δτ_1kg / kp ≤ 1°, Δτ = 박종진 workspace_max_torque 0 → 1 kg 차),
#          하한 = 팀 공용 C5 300 N·m/rad, 50 단위 올림
#   R2 kv: 고정 kv (Isaac ImplicitActuatorCfg damping 과 같은 방식) = 2 · ζ · √(kp · Mᵢᵢ,max), ζ = 1 (가장 무거운 자세에서 임계감쇠)
#          → 가벼운 자세는 과감쇠(오버슈트 없음, 대신 느려짐) → R3 로 확인
#   R3 확인: 0.2 rad 계단, Mᵢᵢ 최소 · 중앙 · 최대 자세, 두 case 에서 오버슈트 ≤ 5 %, 2 % 정착 ≤ 0.30 s
#  비교: week05 기본(kp 300 · dampratio 1 = qpos0 컴파일 kv)
#  출력: results/kpkv_tune.{md,json}, assets/screenshots/week06/week06_kpkv_step.png
import json, os, time
import numpy as np, mujoco
from w06_common import *

STEP = 0.2; TSIM = 1.0; ZETA = 1.0; OS_MAX = 5.0; TS_MAX = 0.30; ERR1KG = np.deg2rad(1.0); KP_FLOOR = 300.0
CASES = ["a2_doc", "cad_ri85"]
t0 = time.time()
wmt = json.load(open(os.path.join(V1, "results", "workspace_max_torque.json"), encoding="utf-8"))["results"]


def mii_samples(case, n=5000, seed=0):
    m = load(case=case); d = mujoco.MjData(m); qi = arm_q(m, "left"); di = arm_d(m, "left")
    jid = [m.joint(f"left_{a}_joint").id for a in ARM]; lo, hi = m.jnt_range[jid, 0], m.jnt_range[jid, 1]
    rng = np.random.default_rng(seed); Q = rng.uniform(lo, hi, (n, 4)); Ms = np.zeros((n, 4)); M = np.zeros((m.nv, m.nv))
    for k in range(n):
        d.qpos[:] = 0; d.qpos[qi] = Q[k]; mujoco.mj_forward(m, d); mujoco.mj_fullM(m, d, M)
        Ms[k] = np.diag(M)[di]
    return Q, Ms, lo, hi


def step_resp(m, q0, j, amp):
    """왼팔 관절 j 에 amp 계단 (나머지 관절은 서보로 유지). 반환: OS %, 2 % 정착 s, 최대 구동 토크(jointactuatorfrc), 접촉 수 최대"""
    d = mujoco.MjData(m); qi = arm_q(m, "left"); di = arm_d(m, "left"); u = arm_u(m, "left")
    d.qpos[qi] = q0; d.ctrl[u] = q0; mujoco.mj_forward(m, d)
    d.ctrl[u[j]] = q0[j] + amp; n = int(TSIM / m.opt.timestep); y = np.zeros(n); tau = np.zeros(n); nc = 0
    for k in range(n):
        mujoco.mj_step(m, d); y[k] = d.qpos[qi[j]] - q0[j]; tau[k] = d.qfrc_actuator[di[j]]; nc = max(nc, d.ncon)
    y = y / amp; tt = (np.arange(n) + 1) * m.opt.timestep
    os_ = max(0.0, (y.max() - 1) * 100); out = np.where(np.abs(y - 1) > 0.02)[0]
    ts = tt[out[-1]] if len(out) and out[-1] < n - 1 else (0.0 if not len(out) else np.inf)
    return os_, ts, float(np.abs(tau).max()), nc, tt, y


# --- 1) Mᵢᵢ 범위 (armature 포함) ---
MII = {c: mii_samples(c) for c in CASES}
# --- 2) R1 → kp 하한, R2 → kv 탐색 (R3 만족하는 가장 작은 kp, 그 kp 에서 최악 정착시간 최소 kv) ---
def poses(c, i):
    Q, Ms, lo, hi = MII[c]; order = np.argsort(Ms[:, i])
    return [(tag, Q[k].copy(), Ms[k, i], (STEP if Q[k][i] + STEP <= hi[i] else -STEP)) for tag, k in
            (("min", order[0]), ("med", order[len(order) // 2]), ("max", order[-1]))]


MS = {c: load(case=c) for c in CASES}       # 게인은 배열에서 직접 바꿔 탐색 (컴파일 반복 없이)


def set_gain(m, i, kp, kv):
    for s in SIDES:
        u = arm_u(m, s)[i]; m.actuator_gainprm[u, 0] = kp; m.actuator_biasprm[u, 1] = -kp; m.actuator_biasprm[u, 2] = -kv


def worst(i, kp, kv):
    w_os = w_ts = 0.0
    for c in CASES:
        set_gain(MS[c], i, kp, kv)
        for tag, q0, mii, amp in poses(c, i):
            os_, ts, *_ = step_resp(MS[c], q0, i, amp); w_os = max(w_os, os_); w_ts = max(w_ts, ts)
    return w_os, w_ts


gains = {}; rows = []
for i, j in enumerate(ARM):
    dtau = wmt["1kg"][j]["tau_abs_max"] - wmt["0kg"][j]["tau_abs_max"]
    kp_min = dtau / ERR1KG; kp = max(KP_FLOOR, np.ceil(kp_min / 50) * 50); kp_r1 = kp
    mmax = max(MII[c][1][:, i].max() for c in CASES); mmin = min(MII[c][1][:, i].min() for c in CASES)
    while True:
        kv_c = 2 * np.sqrt(kp * mmax)                                   # ζ=1 at Mᵢᵢ,max
        cand = []
        for f in np.arange(0.50, 1.21, 0.05):                            # ζ(Mmax) 0.5 ~ 1.2
            kv = round(kv_c * f * 2) / 2; os_, ts = worst(i, kp, kv); cand.append((kv, os_, ts))
        okc = [x for x in cand if x[1] <= OS_MAX and x[2] <= TS_MAX]
        if okc or kp >= 3000: break
        kp += 50
    kv, os_, ts = min(okc, key=lambda x: x[2]) if okc else min(cand, key=lambda x: x[2])
    gains[j] = (float(kp), float(kv))
    rows.append(dict(joint=j, dtau_1kg=dtau, kp_min=kp_min, kp_r1=kp_r1, kp=kp, Mmax=mmax, Mmin=mmin, kv=kv,
                     zeta_at_Mmax=kv / (2 * np.sqrt(kp * mmax)), zeta_at_Mmin=kv / (2 * np.sqrt(kp * mmin)),
                     err_1kg_deg=float(np.rad2deg(dtau / kp)), search=[dict(kv=a, os=b, ts=cc) for a, b, cc in cand]))

# --- 3) R3 확인: 계단 응답 (Mᵢᵢ 최소 · 중앙 · 최대 자세 × 두 case × week05 기본 / week06 튜닝) ---
res = []; curves = {}
for c in CASES:
    Q, Ms, lo, hi = MII[c]
    mb = load(case=c); mt = load(case=c, gains=gains)
    for i, j in enumerate(ARM):
        order = np.argsort(Ms[:, i])
        for tag, k in (("min", order[0]), ("med", order[len(order) // 2]), ("max", order[-1])):
            q0 = Q[k].copy(); amp = STEP if q0[i] + STEP <= hi[i] else -STEP
            for gname, mm in (("week05", mb), ("week06", mt)):
                os_, ts, tmax, nc, tt, y = step_resp(mm, q0, i, amp)
                u = arm_u(mm, "left")[i]; kp = mm.actuator_gainprm[u, 0]; kv = -mm.actuator_biasprm[u, 2]
                zeta = kv / (2 * np.sqrt(kp * Ms[k, i]))
                res.append(dict(case=c, joint=j, pose=tag, Mii=float(Ms[k, i]), gains=gname, kp=float(kp), kv=float(kv), zeta=float(zeta),
                                os_pct=float(os_), ts_s=float(ts), tau_max=tmax, contacts=int(nc), q0=[float(x) for x in q0]))
                if c == "a2_doc": curves[(j, tag, gname)] = (tt, y)

# --- 4) 표 · 판정 ---
def agg(g):
    r = [x for x in res if x["gains"] == g]
    return {j: dict(os=max(x["os_pct"] for x in r if x["joint"] == j), ts=max(x["ts_s"] for x in r if x["joint"] == j)) for j in ARM}
A5, A6 = agg("week05"), agg("week06")
ok6 = all(A6[j]["os"] <= OS_MAX and A6[j]["ts"] <= TS_MAX for j in ARM)
ok5 = all(A5[j]["os"] <= OS_MAX and A5[j]["ts"] <= TS_MAX for j in ARM)

L = ["# kp · kv 튜닝 (week06, 함태훈) — A2 armature 반영 모델, 중력보상 B on", "",
     f"- 모델: 박종진 d6d6304 빌드 사본(`sim/w06/b1_snapshot/v1`), case a2_doc(어깨 TQ armature 0.219 · 팔꿈치 RI85 0.098) 기본, cad_ri85(8개 RI85) 병기. dt {mb.opt.timestep} s, implicitfast",
     f"- 규칙(함태훈 제안): R1 모르는 짐 1 kg 정적 처짐 ≤ 1° → kp ≥ Δτ_1kg / 1°(하한 C5 300) · R2 고정 kv = 2√(kp·Mᵢᵢ,max) (가장 무거운 자세 ζ = 1) · R3 0.2 rad 계단 오버슈트 ≤ {OS_MAX:g} %, 2 % 정착 ≤ {TS_MAX:.2f} s (Mᵢᵢ 최소 · 중앙 · 최대 자세 × 2 case)",
     "", "## 1. Mᵢᵢ 범위 (armature 포함, 왼팔 관절 범위 균일 5,000 자세)", "",
     "| 관절 | a2_doc 최소 / 중앙 / 최대 (kg·m²) | 최대/최소 | cad_ri85 최소 / 최대 | 최대/최소 |", "|---|---|---|---|---|"]
for i, j in enumerate(ARM):
    a, b = MII["a2_doc"][1][:, i], MII["cad_ri85"][1][:, i]
    L.append(f"| {j} | {a.min():.4f} / {np.median(a):.4f} / {a.max():.4f} | {a.max()/a.min():.2f} | {b.min():.4f} / {b.max():.4f} | {b.max()/b.min():.2f} |")
L += ["", "→ armature 가 Mᵢᵢ 의 바닥을 올려 자세에 따른 변화가 줄어듦 (pitch 링크만 0.049 ~ 0.849 = 17배 → TQ armature 0.219 더하면 3.98배). 고정 kv 를 쓸 수 있는 근거.", "",
      "## 2. 선정 kp · kv", "", "| 관절 | Δτ 0→1 kg (N·m) | kp 최소 R1 | **kp** (R1 → R3 만족까지 50씩) | Mᵢᵢ 최소 ~ 최대 | **kv** | ζ (Mᵢᵢ 최대 ~ 최소) | 1 kg 처짐 ° | week05 kp · kv(qpos0 컴파일, a2_doc) |", "|---|---|---|---|---|---|---|---|---|"]
mb_a = load(case="a2_doc")
for i, r in enumerate(rows):
    kvb = -mb_a.actuator_biasprm[arm_u(mb_a, "left")[i], 2]
    L.append(f"| {r['joint']} | {r['dtau_1kg']:.2f} | {r['kp_min']:.0f} | **{r['kp']:.0f}** | {r['Mmin']:.3f} ~ {r['Mmax']:.3f} | **{r['kv']:.1f}** | {r['zeta_at_Mmax']:.2f} ~ {r['zeta_at_Mmin']:.2f} | {r['err_1kg_deg']:.2f} | 300 · {kvb:.2f} |")
L += ["", "## 3. 계단 응답 0.2 rad (최악값 = 3 자세 × 2 case 중 최대)", "",
      "| 관절 | week05 오버슈트 % · 정착 s | **week06 오버슈트 % · 정착 s** | week06 ζ 범위 | 최대 구동 토크 N·m (week06) |", "|---|---|---|---|---|"]
for j in ARM:
    z = [x["zeta"] for x in res if x["gains"] == "week06" and x["joint"] == j]; tm = max(x["tau_max"] for x in res if x["gains"] == "week06" and x["joint"] == j)
    L.append(f"| {j} | {A5[j]['os']:.1f} · {A5[j]['ts']:.3f} | **{A6[j]['os']:.1f} · {A6[j]['ts']:.3f}** | {min(z):.2f} ~ {max(z):.2f} | {tm:.1f} |")
L += ["", f"- 판정: week06 게인 R3 **{'통과' if ok6 else '미통과'}** (week05 기본 {'통과' if ok5 else '미통과'})",
      f"- 접촉: 계단 시험 중 최대 접촉 수 {max(x['contacts'] for x in res)}",
      "", "## 4. 자세 · case 별 전체", "", "| case | 관절 | 자세 | Mᵢᵢ | 게인 | kp | kv | ζ | 오버슈트 % | 정착 s | τmax |", "|---|---|---|---|---|---|---|---|---|---|---|"]
for x in res:
    L.append(f"| {x['case']} | {x['joint']} | {x['pose']} | {x['Mii']:.4f} | {x['gains']} | {x['kp']:.0f} | {x['kv']:.2f} | {x['zeta']:.2f} | {x['os_pct']:.2f} | {x['ts_s']:.3f} | {x['tau_max']:.1f} |")
L += ["", f"실행 {time.time()-t0:.1f} s · `python sim/w06/w06_kpkv_tune.py`"]
save_md("kpkv_tune.md", "\n".join(L) + "\n")
save_json("kpkv_tune.json", dict(rule=dict(step=STEP, zeta=ZETA, os_max=OS_MAX, ts_max=TS_MAX, err_1kg_deg=1.0, kp_floor=KP_FLOOR),
                                 gains={j: dict(kp=g[0], kv=g[1]) for j, g in gains.items()}, rows=rows, results=res,
                                 summary=dict(week05=A5, week06=A6, pass_week06=ok6, pass_week05=ok5)))

plt = plot_style()
fig, ax = plt.subplots(1, 4, figsize=(12, 3), sharey=True)
for i, j in enumerate(ARM):
    for tag, ls in (("min", ":"), ("med", "--"), ("max", "-")):
        for g, col in (("week05", "#8a8986"), ("week06", JCOL[j])):
            tt, y = curves[(j, tag, g)]; ax[i].plot(tt, y, ls, color=col, lw=1.5, label=f"{g} Mᵢᵢ {tag}")
    ax[i].axhline(1.05, color="#c9c8c4", lw=0.8); ax[i].axhline(0.98, color="#c9c8c4", lw=0.8)
    ax[i].set_title(f"{j}  kp {gains[j][0]:.0f} · kv {gains[j][1]:.1f}"); ax[i].set_xlabel("t (s)"); ax[i].set_xlim(0, 0.6)
ax[0].set_ylabel("q / 목표 (0.2 rad 계단)"); ax[3].legend(loc="lower right", fontsize=6)
fig.suptitle("계단 응답 — 회색 week05 (kp 300, dampratio 1) / 색 week06 튜닝 (a2_doc)", fontsize=10); fig.tight_layout()
fig.savefig(os.path.join(IMG, "week06_kpkv_step.png"), dpi=130)
print("\n".join(L[:40]))
