# week05 | v1 액추에이터 파라미터표 — joint ↔ kp · kv · forcerange · armature · damping 과 그 근거 (함태훈, B1 · 커리큘럼 개별 과제)
#  값의 출처: actuator_params.json (MJCF 덧붙이기, 함태훈 담당) = B1 공용 가정값 C4 · C5 (팀장 10/16)
#  이 스크립트가 하는 일
#   (1) 컴파일된 모델에서 실제 kv · ctrlrange · forcerange 를 읽어 표로 만든다 (json 값 ↔ 모델 값 일치 확인)
#   (2) dampratio → kv 환산 규칙 확인: kv = 2·ζ·√(kp · I_eff),  I_eff = 1 / (M⁻¹)_ii  at qpos0 (다른 자유도 · 베이스가 자유일 때 반사관성)
#   (3) 실제 운용(다른 관절은 서보로 붙잡힘)에서는 관성 ≈ M_ii → 실효 감쇠비 ζ_hold = kv / (2√(kp·M_ii)). 자세에 따라 M_ii 가 변하므로 범위를 구함
#   (4) 관절별 계단 응답(0.2 rad, 다른 관절 유지, 중력보상)으로 오버슈트 · 정착시간 측정 → (3) 예측과 비교
#   (5) 포화 오차 forcerange/kp, 수평 처짐 τg/kp, 0.5° 처짐 기준 필요 kp
#  출력: results/actuator_table.csv · actuator_table.md, assets/screenshots/week05/week05_step_per_joint.png
#  실행: python sim/v1/v1_actuator_table.py
import csv, os
import numpy as np, mujoco
import v1_common as c

m = c.load(); d = mujoco.MjData(m); P = c.params()
mujoco.mj_forward(m, d)
M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, M); Mi = np.linalg.inv(M)
cls = {a: k for k, v in P["classes"].items() for a in v["joints"]}
lim = c.assumptions()["limits"]

# (3) 자세별 M_ii 범위: 관절 범위 안 무작위 2000 자세 (왼팔, 베이스 0)
rng = np.random.default_rng(0); Mii = {a: [] for a in c.ARM}
for _ in range(2000):
    d.qpos[:] = 0; c.set_arm(m, d, [rng.uniform(*lim[a]) for a in c.ARM]); mujoco.mj_forward(m, d)
    Mq = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, Mq)
    for a in c.ARM: Mii[a].append(Mq[c.da(m, f"left_{a}_joint"), c.da(m, f"left_{a}_joint")])


# (4) 계단 응답: 영점에서 관절 하나만 +0.2 rad, 나머지 관절 목표 = 현재, 중력보상(τg/kp 피드포워드)으로 중력 영향 제거
def step(joint, amp=0.2, T=1.0, model=None):
    mm = model or m; dd = mujoco.MjData(mm); mujoco.mj_forward(mm, dd)
    acts = {mm.actuator(i).name: i for i in range(mm.nu)}
    q0 = dd.qpos.copy(); tgt = {f"{s}_{a}": q0[c.qa(mm, f"{s}_{a}_joint")] for s in c.SIDES for a in c.ARM}
    tgt[joint] += amp; kp = {n: mm.actuator_gainprm[i][0] for n, i in acts.items()}
    t, y, tau = [], [], []
    for k in range(int(T / mm.opt.timestep)):
        for n, v in tgt.items():
            j = f"{n}_joint"; dd.ctrl[acts[n]] = v + dd.qfrc_bias[c.da(mm, j)] / kp[n]
        mujoco.mj_step(mm, dd)
        t.append(dd.time); y.append(dd.qpos[c.qa(mm, f"{joint}_joint")] - q0[c.qa(mm, f"{joint}_joint")]); tau.append(dd.actuator_force[acts[joint]])
    t, y = np.array(t), np.array(y) / amp
    os_ = max(0.0, (y.max() - 1) * 100)
    out = np.where(np.abs(y - 1) > 0.02)[0]; ts = t[out[-1]] if len(out) else 0.0          # 2 % 정착
    rise = t[np.argmax(y >= 0.9)] - t[np.argmax(y >= 0.1)]
    return dict(t=t, y=y, tau=np.array(tau), overshoot=os_, settle=ts, rise=rise, tau_max=float(np.abs(tau).max()))


def zeta_os(z):   # 2차계 오버슈트 [%]
    return 0.0 if z >= 1 else 100 * np.exp(-np.pi * z / np.sqrt(1 - z * z))


rows, steps = [], {}
for a in c.ARM:
    n = f"left_{a}"; i = m.actuator(n).id; dof = c.da(m, f"{n}_joint"); cp = P["classes"][cls[a]]
    kp = m.actuator_gainprm[i][0]; kv = -m.actuator_biasprm[i][2]
    Ieff = 1 / Mi[dof, dof]; M0 = M[dof, dof]
    z_hold = kv / (2 * np.sqrt(kp * M0)); zr = [kv / (2 * np.sqrt(kp * x)) for x in (max(Mii[a]), min(Mii[a]))]
    s = step(n); steps[a] = s
    # 수평 처짐(양팔 앞으로, pitch −90°)의 중력 토크
    d.qpos[:] = 0; c.set_arm(m, d, [-np.pi / 2, 0, 0, 0]); mujoco.mj_forward(m, d); tg = abs(d.qfrc_bias[dof])
    rows.append(dict(joint=f"{{side}}_{a}_joint", cls=cls[a], kp=kp, dampratio=cp["dampratio"], kv_compiled=round(kv, 3),
                     I_eff_kgm2=round(Ieff, 5), M_ii_qpos0=round(M0, 5), M_ii_min=round(min(Mii[a]), 5), M_ii_max=round(max(Mii[a]), 5),
                     wn_rad_s=round(np.sqrt(kp / M0), 1), zeta_hold=round(z_hold, 3), zeta_range=f"{zr[0]:.2f}~{zr[1]:.2f}",
                     os_pred_pct=round(zeta_os(z_hold), 1), os_sim_pct=round(s["overshoot"], 1), settle_s=round(s["settle"], 3), rise_s=round(s["rise"], 3),
                     forcerange=float(m.actuator_forcerange[i][1]), sat_err_deg=round(np.degrees(m.actuator_forcerange[i][1] / kp), 1),
                     ctrlrange=f"{m.actuator_ctrlrange[i][0]:g}~{m.actuator_ctrlrange[i][1]:g}", armature=float(m.dof_armature[dof]),
                     damping=float(m.dof_damping[dof]), frictionloss=float(m.dof_frictionloss[dof]),
                     tau_g_fwd=round(tg, 2), sag_deg=round(np.degrees(tg / kp), 2), kp_for_0p5deg=round(tg / np.radians(0.5)),
                     json_match=bool(abs(kp - cp["kp"]) < 1e-9 and abs(m.actuator_forcerange[i][1] - cp["forcerange"]) < 1e-9
                                     and abs(m.dof_armature[dof] - cp["armature"]) < 1e-12 and abs(m.dof_damping[dof] - cp["damping"]) < 1e-12)))
    # 규칙 확인 (assert): kv = 2ζ√(kp·I_eff)
    assert abs(kv - 2 * cp["dampratio"] * np.sqrt(kp * Ieff)) < 1e-6, a

with open(os.path.join(c.OUT, "actuator_table.csv"), "w", newline="", encoding="utf-8-sig") as fp:
    w = csv.DictWriter(fp, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)

# 그림: 관절별 정규화 계단 응답
plt = c.plot_style()
fig, ax = plt.subplots(1, 2, figsize=(10, 3.6))
col = c.JCOL
for a, s in steps.items():
    r = [x for x in rows if x["joint"].endswith(f"{a}_joint")][0]
    ax[0].plot(s["t"], s["y"], color=col[a], label=f'{a} (ζ_hold {r["zeta_hold"]}, OS {r["os_sim_pct"]} %)')
    ax[1].plot(s["t"], s["tau"], color=col[a])
ax[0].axhline(1, color="0.6", lw=0.8, ls="--"); ax[0].set(xlabel="t [s]", ylabel="q / 0.2 rad", title="계단 응답 (kp 300, dampratio 1, 다른 관절 유지)")
ax[0].legend(fontsize=7.5, frameon=False); ax[1].set(xlabel="t [s]", ylabel="actuator_force [N·m]", title="구동 토크 (중력보상 포함)")
fig.tight_layout(); fig.savefig(os.path.join(c.IMG, "week05_step_per_joint.png"), dpi=150)

L = ["# v1 액추에이터 파라미터표 (자동 생성 — sim/v1/v1_actuator_table.py)", "",
     f"모델: 박종진 build_v1.py 빌드 `b1.xml`(값 = actuator_params.json), MuJoCo {mujoco.__version__}, timestep {m.opt.timestep} s, integrator implicitfast. 왼팔 기준(오른팔 = 거울, 값 동일)", "",
     "| joint | class | kp [N·m/rad] | dampratio → kv 컴파일값 [N·m·s/rad] | I_eff = 1/(M⁻¹)ᵢᵢ | Mᵢᵢ(qpos0) [범위] | ω_n=√(kp/Mᵢᵢ) | ζ_hold [범위] | OS 예측/시뮬 % | 정착 2 % [s] | forcerange (포화 오차) | ctrlrange [rad] | armature · damping · frictionloss |",
     "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
for r in rows:
    L.append(f'| `{r["joint"]}` | {r["cls"]} | {r["kp"]:g} | {r["dampratio"]:g} → {r["kv_compiled"]} | {r["I_eff_kgm2"]} | {r["M_ii_qpos0"]} [{r["M_ii_min"]}~{r["M_ii_max"]}] | {r["wn_rad_s"]} | {r["zeta_hold"]} [{r["zeta_range"]}] | {r["os_pred_pct"]} / {r["os_sim_pct"]} | {r["settle_s"]} | ±{r["forcerange"]:g} N·m ({r["sat_err_deg"]}°) | {r["ctrlrange"]} | {r["armature"]:g} · {r["damping"]:g} · {r["frictionloss"]:g} |')
L += ["", "| joint | 수평 자세 τg [N·m] | 처짐 τg/kp [°] | 0.5° 기준 필요 kp | json ↔ 모델 일치 |", "|---|---|---|---|---|"]
L += [f'| `{r["joint"]}` | {r["tau_g_fwd"]} | {r["sag_deg"]} | {r["kp_for_0p5deg"]} | {"OK" if r["json_match"] else "NG"} |' for r in rows]
open(os.path.join(c.OUT, "actuator_table.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
print("\n".join(L))
