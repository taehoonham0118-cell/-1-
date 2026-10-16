# week05 | armature 를 넣었을 때 / 뺐을 때 — 계단 응답과 수치 안정성 (구술 Q2, 3주차 2링크 결과와 연결) (함태훈, B1)
#  armature = 감속기 앞(모터 쪽) 회전자 관성을 출력축으로 옮긴 값 J_rotor · N². RI85 사양(A2) 미수령 → 0 (C4). 그래서 「값이 들어오면 무엇이 바뀌나」를 미리 본다
#  실험 1 계단 응답: v1 왼팔 shoulder_pitch 0.2 rad 계단, 다른 관절 유지(목표 오프셋 중력보상), armature 0 ~ 0.5 kg·m²
#     (i) dampratio 1 (컴파일 때 kv 를 관성에 맞춰 다시 계산)  (ii) kv 고정 = armature 0 일 때 값(20.53)
#  실험 2 수치 안정성: 가장 가벼운 관절(shoulder_yaw, Mᵢᵢ 0.0153 kg·m²)에 계단 → integrator(Euler · implicitfast) × timestep 를 올리며 발산 여부
#     이론: 명시적 적분의 안정 한계 dt ≈ 2/ω_n (감쇠 무시), ω_n = √(kp/(Mᵢᵢ + armature))
#  출력: results/armature_study.csv · armature_study.md, assets/screenshots/week05/week05_armature_step.png
#  실행: python sim/v1/v1_armature_study.py
import csv, os, re
import numpy as np, mujoco
import v1_common as c

ARMS = [0.0, 0.02, 0.05, 0.1, 0.2, 0.5]
KV0 = None


def edit(arm, kv=None, cls="shoulder", all_cls=False):
    def f(x):
        for cl in (("shoulder", "elbow") if all_cls else (cls,)):
            x = re.sub(rf'(<default class="{cl}">\s*<joint armature=")[^"]*"', rf'\g<1>{arm}"', x)
        if kv is not None:
            x = re.sub(rf'(<default class="{cls}">\s*<joint[^>]*>\s*<position kp="[^"]*") dampratio="[^"]*"', rf'\1 kv="{kv}"', x)
        return x
    return f


def step(m, joint, amp=0.2, T=1.5):
    d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    acts = {m.actuator(i).name: i for i in range(3, m.nu)}
    q0 = d.qpos.copy(); tgt = {n: q0[c.qa(m, f"{n}_joint")] for n in acts}; tgt[joint] += amp
    kp = {n: m.actuator_gainprm[i][0] for n, i in acts.items()}
    t, y, tau = [], [], []
    for k in range(int(T / m.opt.timestep)):
        for n, i in acts.items(): d.ctrl[i] = tgt[n] + d.qfrc_bias[c.da(m, f"{n}_joint")] / kp[n]
        mujoco.mj_step(m, d)
        if not np.isfinite(d.qpos).all() or np.abs(d.qpos[3:]).max() > 50: return None
        t.append(d.time); y.append((d.qpos[c.qa(m, f"{joint}_joint")] - q0[c.qa(m, f"{joint}_joint")]) / amp); tau.append(d.actuator_force[acts[joint]])
    t, y = np.array(t), np.array(y); out = np.where(np.abs(y - 1) > 0.02)[0]
    return dict(t=t, y=y, tau=np.array(tau), os=max(0.0, (y.max() - 1) * 100), settle=float(t[out[-1]]) if len(out) else 0.0,
                rise=float(t[np.argmax(y >= 0.9)] - t[np.argmax(y >= 0.1)]), tau_peak=float(np.abs(tau).max()),
                late_osc=float(np.ptp(y[int(len(y) * 0.8):])))


rows, curves = [], {}
m0 = c.load(); KV0 = -m0.actuator_biasprm[m0.actuator("left_shoulder_pitch").id][2]
dof = c.da(m0, "left_shoulder_pitch_joint"); d0 = mujoco.MjData(m0); mujoco.mj_forward(m0, d0)
M = np.zeros((m0.nv, m0.nv)); mujoco.mj_fullM(m0, d0, M); Mii = M[dof, dof]
for arm in ARMS:
    for pol in ("dampratio 1", "kv 고정"):
        m = c.load(xml_edit=edit(arm, None if pol == "dampratio 1" else round(KV0, 4)))
        kv = -m.actuator_biasprm[m.actuator("left_shoulder_pitch").id][2]
        s = step(m, "left_shoulder_pitch"); curves[(arm, pol)] = s
        Ih = Mii + arm; wn = np.sqrt(300 / Ih); z = kv / (2 * np.sqrt(300 * Ih))
        rows.append(dict(exp="step", joint="shoulder_pitch", armature=arm, policy=pol, kv=round(kv, 3), I_hold=round(Ih, 4), wn=round(wn, 2), zeta_hold=round(z, 3),
                         overshoot_pct=round(s["os"], 2), rise_s=round(s["rise"], 3), settle_s=round(s["settle"], 3), tau_peak=round(s["tau_peak"], 1)))

# 실험 2: 안정성 — armature 를 팔 8관절 전부(어깨 · 팔꿈치 class)에 넣고, 가장 가벼운 shoulder_yaw 에 계단
#   1-DOF 반암시적(symplectic) Euler 의 정확한 안정 한계: v⁺ = v + dt(−k x − c v)/I, x⁺ = x + dt v⁺ 의 행렬 스펙트럼 반지름 < 1
def dt_limit(I, k, cdamp):
    for dt in np.arange(1e-4, 0.2, 1e-4):
        A_ = np.array([[1 - dt * dt * k / I, dt * (1 - dt * cdamp / I)], [-dt * k / I, 1 - dt * cdamp / I]])
        if np.abs(np.linalg.eigvals(A_)).max() > 1 + 1e-9: return round(dt - 1e-4, 4)
    return None
stab = []
dts = [0.002, 0.004, 0.005, 0.006, 0.007, 0.008, 0.01, 0.012, 0.015, 0.02, 0.025, 0.03, 0.04, 0.05]
dofy = c.da(m0, "left_shoulder_yaw_joint"); Myy = M[dofy, dofy]
for arm in (0.0, 0.01, 0.05):
    for integ, iv in (("Euler", mujoco.mjtIntegrator.mjINT_EULER), ("implicitfast", mujoco.mjtIntegrator.mjINT_IMPLICITFAST)):
        m = c.load(xml_edit=edit(arm, all_cls=True)); m.opt.integrator = iv; ok_dt = None
        kvy = -m.actuator_biasprm[m.actuator("left_shoulder_yaw").id][2]
        for dt in dts:
            m.opt.timestep = dt; s = step(m, "left_shoulder_yaw", T=2.0)
            good = s is not None and s["late_osc"] < 0.01 and abs(s["y"][-1] - 1) < 0.05
            stab.append(dict(exp="stability", joint="shoulder_yaw", armature=arm, integrator=integ, dt=dt, stable=good))
            if good: ok_dt = dt
            else: break
        Iy = Myy + arm; wn = np.sqrt(300 / Iy)
        # 팔 관절별 예측 한계 (I = Mᵢᵢ + armature, k = kp, c = kv + damping) → 최솟값이 모델 전체 한계
        lim_j = {}
        for a in c.ARM:
            dj = c.da(m, f"left_{a}_joint"); kvj = -m.actuator_biasprm[m.actuator(f"left_{a}").id][2]
            lim_j[a] = dt_limit(M[dj, dj] + arm, 300.0, kvj + m.dof_damping[dj])
        jmin = min(lim_j, key=lambda a: lim_j[a])
        rows.append(dict(exp="stability", joint="shoulder_yaw", armature=arm, policy=integ, kv=round(kvy, 3), I_hold=round(Iy, 4), wn=round(wn, 1),
                         zeta_hold=round(kvy / (2 * np.sqrt(300 * Iy)), 3), max_stable_dt=ok_dt, dt_theory_2_over_wn=round(2 / wn, 4),
                         dt_pred_euler=f"{lim_j[jmin]} ({jmin})"))

with open(os.path.join(c.OUT, "armature_study.csv"), "w", newline="", encoding="utf-8-sig") as fp:
    keys = []; [keys.append(k) for r in rows for k in r if k not in keys]
    w = csv.DictWriter(fp, fieldnames=keys); w.writeheader(); w.writerows(rows)

plt = c.plot_style()
fig, ax = plt.subplots(1, 2, figsize=(10, 3.6), sharey=True)
for k_, pol in enumerate(("dampratio 1", "kv 고정")):
    for arm, col in zip(ARMS, c.SERIES):
        s = curves[(arm, pol)]; ax[k_].plot(s["t"], s["y"], color=col, label=f"armature {arm:g} (OS {s['os']:.1f} %)")
    ax[k_].axhline(1, color="#8a8986", lw=0.8, ls="--"); ax[k_].set(xlabel="t [s]", title=f"shoulder_pitch 0.2 rad 계단 — {pol}" + (f" ({KV0:.2f})" if pol == "kv 고정" else ""))
    ax[k_].legend(loc="lower right")
ax[0].set_ylabel("q / 0.2 rad"); fig.tight_layout(); fig.savefig(os.path.join(c.IMG, "week05_armature_step.png"), dpi=150)

L = ["# armature 유무 — 계단 응답 · 수치 안정성 (자동 생성 — sim/v1/v1_armature_study.py)", "",
     f"모델: v1 `scene.xml`, MuJoCo {mujoco.__version__}. armature 는 메모리에서만 바꿈(파일 그대로). shoulder_pitch Mᵢᵢ(qpos0) = {Mii:.4f}, shoulder_yaw Mᵢᵢ = {Myy:.4f} kg·m²", "",
     "## 실험 1 — shoulder_pitch 계단 (kp 300, 다른 관절 유지, 중력보상)", "",
     "| armature [kg·m²] | kv 정책 | kv | I = Mᵢᵢ + armature | ω_n = √(kp/I) | ζ_hold | 오버슈트 % | 상승 10→90 % [s] | 정착 2 % [s] | 최대 토크 [N·m] |", "|---|---|---|---|---|---|---|---|---|---|"]
L += [f'| {r["armature"]:g} | {r["policy"]} | {r["kv"]} | {r["I_hold"]} | {r["wn"]} | {r["zeta_hold"]} | {r["overshoot_pct"]} | {r["rise_s"]} | {r["settle_s"]} | {r["tau_peak"]} |' for r in rows if r["exp"] == "step"]
L += ["", "## 실험 2 — 수치 안정성 (shoulder_yaw 0.2 rad 계단, 2 s 뒤 1 % 안에 정착하면 안정)", "",
      "| armature (팔 8관절) | integrator | yaw ω_n [rad/s] | 감쇠 무시 2/ω_n [s] | 반암시적 Euler 예측 한계 (가장 엄한 관절) | 시뮬 안정 최대 timestep [s] (시험 " + ", ".join(f"{x:g}" for x in dts) + ") |", "|---|---|---|---|---|---|"]
L += [f'| {r["armature"]:g} | {r["policy"]} | {r["wn"]} | {r["dt_theory_2_over_wn"]} | {r["dt_pred_euler"] if r["policy"] == "Euler" else "– (속도 항 암시적)"} | {r["max_stable_dt"]} |' for r in rows if r["exp"] == "stability"]
open(os.path.join(c.OUT, "armature_study.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
print("\n".join(L))
