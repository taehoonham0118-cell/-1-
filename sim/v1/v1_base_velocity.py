# week05 | 공통 실습(화): 홀로노믹 베이스 x · y · yaw 속도 명령 vs 실제 qvel (함태훈, B1)
#  베이스 = slide x · slide y · hinge yaw (URDF planar → MuJoCo _TX · _TY · _RZ) + velocity 액추에이터 (base_vx · vy · wz)
#  velocity 액추에이터 힘 = kv (ctrl − q̇) → 1차 지연: 시정수 τ = M_eff / kv  (x · y: 로봇 전체 질량, yaw: 베이스 축 회전관성)
#  구간 (팔은 영점 유지 + 중력보상 B):
#   A 0~2 s  vx = 0.5 m/s           B 2~4 s  vy = 0.5 m/s          C 4~6 s  wz = 0.5 rad/s
#   D 6~9 s  「로봇 기준 앞으로 0.5 m/s」 + wz 0.5 rad/s — 명령은 월드 축이라 R(yaw) 로 변환 (팀장 10/8: 로봇 기준 속도 = R(yaw) 변환)
#  출력: results/base_velocity.md · .csv, assets/screenshots/week05/week05_base_velocity.png
#  실행: python sim/v1/v1_base_velocity.py
import csv, os
import numpy as np, mujoco
import v1_common as c

m = c.load(xml_edit=c.gravcomp_edit); d = mujoco.MjData(m)
J = {k: m.joint(f"base_planar_joint_{k}").id for k in ("TX", "TY", "RZ")}
qv = {k: m.jnt_dofadr[j] for k, j in J.items()}; qp = {k: m.jnt_qposadr[j] for k, j in J.items()}
A = {k: m.actuator(n).id for k, n in (("TX", "base_vx"), ("TY", "base_vy"), ("RZ", "base_wz"))}
kv = {k: float(m.actuator_gainprm[i][0]) for k, i in A.items()}
mujoco.mj_forward(m, d)
M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, M)
Meff = {k: M[qv[k], qv[k]] for k in qv}                                   # 팔 고정 · 영점 기준 (팔은 서보로 붙잡힘)
tau_th = {k: Meff[k] / kv[k] for k in kv}
arm = {f"{s}_{a}": m.actuator(f"{s}_{a}").id for s in c.SIDES for a in c.ARM}
q0 = {n: d.qpos[c.qa(m, f"{n}_joint")] for n in arm}


def cmd_world(t, yaw):
    if t < 2: return 0.5, 0.0, 0.0, "A vx"
    if t < 4: return 0.0, 0.5, 0.0, "B vy"
    if t < 6: return 0.0, 0.0, 0.5, "C wz"
    vb = np.array([0.5, 0.0]); R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    vw = R @ vb; return vw[0], vw[1], 0.5, "D body-forward + wz"


T = 9.0; rows = []
while d.time < T - 1e-9:
    yaw = d.qpos[qp["RZ"]]; vx, vy, wz, ph = cmd_world(d.time, yaw)
    d.ctrl[A["TX"]], d.ctrl[A["TY"]], d.ctrl[A["RZ"]] = vx, vy, wz
    for n, i in arm.items(): d.ctrl[i] = q0[n]
    mujoco.mj_step(m, d)
    yaw = d.qpos[qp["RZ"]]; Rt = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
    vb = Rt @ np.array([d.qvel[qv["TX"]], d.qvel[qv["TY"]]])            # 로봇 기준 속도
    rows.append(dict(t=round(d.time, 4), phase=ph, cmd_vx=vx, cmd_vy=vy, cmd_wz=wz, vx=d.qvel[qv["TX"]], vy=d.qvel[qv["TY"]], wz=d.qvel[qv["RZ"]],
                     vbody_x=vb[0], vbody_y=vb[1], x=d.qpos[qp["TX"]], y=d.qpos[qp["TY"]], yaw=yaw,
                     arm_err_deg=float(np.degrees(max(abs(d.qpos[c.qa(m, f"{n}_joint")] - q0[n]) for n in arm)))))
R_ = {k: np.array([r[k] for r in rows]) for k in rows[0] if k != "phase"}; ph = np.array([r["phase"] for r in rows]); t = R_["t"]


def tau63(sig, t0, target):
    sel = (t > t0) & (t < t0 + 2)
    y = sig[sel]; tt = t[sel] - t0
    i = np.argmax(y >= 0.632 * target); return float(tt[i])


meas = {"TX": tau63(R_["vx"], 0, 0.5), "TY": tau63(R_["vy"], 2, 0.5), "RZ": tau63(R_["wz"], 4, 0.5)}
ss = lambda sig, a, b, tgt: float(np.abs(sig[(t > a) & (t < b)] - tgt).max())
D = (t > 7.5) & (t < 9)
res = dict(kv={k: kv[k] for k in kv}, Meff={k: round(Meff[k], 4) for k in Meff}, tau_theory={k: round(tau_th[k], 4) for k in tau_th},
           tau_meas={k: round(meas[k], 4) for k in meas},
           ss_err=dict(vx=ss(R_["vx"], 1.5, 2, 0.5), vy=ss(R_["vy"], 3.5, 4, 0.5), wz=ss(R_["wz"], 5.5, 6, 0.5)),
           D_body_fwd=float(R_["vbody_x"][D].mean()), D_body_side_max=float(np.abs(R_["vbody_y"][D]).max()), D_wz=float(R_["wz"][D].mean()),
           cross_max=dict(vy_in_A=float(np.abs(R_["vy"][(t > 0) & (t < 2)]).max()), wz_in_A=float(np.abs(R_["wz"][(t > 0) & (t < 4)]).max()),
                          vx_B_vs_exp=float(np.abs(R_["vx"][(t > 2) & (t < 4)] - 0.5 * np.exp(-(t[(t > 2) & (t < 4)] - 2) / tau_th["TX"])).max())),
           arm_err_max_deg=float(R_["arm_err_deg"].max()), end_pose=[round(float(R_["x"][-1]), 3), round(float(R_["y"][-1]), 3), round(float(np.degrees(R_["yaw"][-1])), 1)])
with open(os.path.join(c.OUT, "base_velocity.csv"), "w", newline="", encoding="utf-8-sig") as fp:
    w = csv.DictWriter(fp, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows[::5])

plt = c.plot_style()
fig, ax = plt.subplots(1, 2, figsize=(10.5, 3.6))
for key, cmd, col, lab in (("vx", "cmd_vx", c.SERIES[0], "vx [m/s]"), ("vy", "cmd_vy", c.SERIES[1], "vy [m/s]"), ("wz", "cmd_wz", c.SERIES[2], "wz [rad/s]")):
    ax[0].plot(t, R_[cmd], color=col, lw=1, ls="--"); ax[0].plot(t, R_[key], color=col, label=lab)
for x0 in (2, 4, 6): ax[0].axvline(x0, color="#8a8986", lw=0.6)
for x0, s in ((1, "A vx"), (3, "B vy"), (5, "C wz"), (7.5, "D 로봇 기준 전진 + 회전")): ax[0].text(x0, 0.58, s, ha="center", fontsize=8, color="#52514e")
ax[0].set(xlabel="t [s]", ylabel="월드 기준 속도", title="명령(점선) vs 실제 qvel(실선)", ylim=(-0.6, 0.65)); ax[0].legend(loc="lower left")
ax[1].plot(R_["x"], R_["y"], color=c.SERIES[0]); ax[1].plot(R_["x"][0], R_["y"][0], "o", color="#52514e", ms=6); ax[1].text(R_["x"][0] - 0.05, R_["y"][0] - 0.15, "시작", fontsize=8, color="#52514e")
for i in list(range(0, int(6 / 0.002), 500)) + list(range(int(6 / 0.002), len(t), 250)):
    ax[1].annotate("", xy=(R_["x"][i] + 0.18 * np.cos(R_["yaw"][i]), R_["y"][i] + 0.18 * np.sin(R_["yaw"][i])), xytext=(R_["x"][i], R_["y"][i]),
                   arrowprops=dict(arrowstyle="-|>", color=c.SERIES[1], lw=1.4))
ax[1].set(xlabel="x [m]", ylabel="y [m]", title="베이스 궤적 (주황 화살표 = 로봇 정면 방향)"); ax[1].set_aspect("equal", adjustable="datalim")
fig.tight_layout(); fig.savefig(os.path.join(c.IMG, "week05_base_velocity.png"), dpi=150)

L = ["# 공통 실습(화) — 베이스 속도 명령 vs qvel (자동 생성 — sim/v1/v1_base_velocity.py)", "",
     f"모델 v1 `scene.xml` + 중력보상 B, MuJoCo {mujoco.__version__}, dt {m.opt.timestep} s. 팔 영점 유지. 그림: `assets/screenshots/week05/week05_base_velocity.png`", "",
     "| 축 | kv | M_eff (질량행렬 대각) | 이론 τ = M/kv [s] | 측정 τ(63 %) [s] | 구간 끝 0.5 s 최대 오차 (이론 잔여 0.5·e^(−1.5/τ)) |", "|---|---|---|---|---|---|"]
for k, lab, unit in (("TX", "x (base_vx)", "m/s"), ("TY", "y (base_vy)", "m/s"), ("RZ", "yaw (base_wz)", "rad/s")):
    e = res["ss_err"][{"TX": "vx", "TY": "vy", "RZ": "wz"}[k]]
    L.append(f"| {lab} | {kv[k]:g} | {Meff[k]:.4f} {'kg' if k != 'RZ' else 'kg·m²'} | {tau_th[k]:.4f} | {meas[k]:.4f} | {e:.2e} ({0.5*np.exp(-1.5/tau_th[k]):.2e}) {unit} |")
L += ["", f"- 축 간 섞임: 구간 A 에서 |vy| ≤ {res['cross_max']['vy_in_A']:.1e} m/s, A · B 에서 |wz| ≤ {res['cross_max']['wz_in_A']:.1e} rad/s, 구간 B 의 vx 는 순수 1차 감쇠 0.5·e^(−t/τ) 와 차 ≤ {res['cross_max']['vx_B_vs_exp']:.1e} m/s → x · y · yaw 독립 (홀로노믹). 오차는 정상상태 오차가 아니라 1차 지연의 남은 꼬리",
      f"- 구간 D (로봇 기준 전진 0.5 m/s 를 R(yaw) 로 월드 명령 변환 + 회전 0.5 rad/s): 7.5~9 s 로봇 기준 전진 {res['D_body_fwd']:.3f} m/s · 옆 방향 최대 {res['D_body_side_max']:.3f} m/s · wz {res['D_wz']:.3f} rad/s — 이론(돌아가는 명령을 1차 지연으로 따라감, ωτ = {0.5*tau_th['TX']:.4f}): 전진 v/(1+(ωτ)²) = {0.5/(1+(0.5*tau_th['TX'])**2):.3f}, 옆 v·ωτ/(1+(ωτ)²) = {0.5*0.5*tau_th['TX']/(1+(0.5*tau_th['TX'])**2):.3f} m/s → 일치. 회전하며 직진하려면 kv 를 키우거나 R(yaw)에 ωτ 만큼 앞선 각도를 써야 함",
      f"- 베이스가 움직이는 동안 팔 관절 최대 오차 {res['arm_err_max_deg']:.4f}° (중력보상 B) — 베이스 가속 순간의 관성력 때문(정지 유지 0° 와 대비), 6주차 「팔 + 베이스 동시 명령」 에서 다룰 항목. 끝 위치 x {res['end_pose'][0]} · y {res['end_pose'][1]} m · yaw {res['end_pose'][2]}°",
      "- 박종진 base_velocity_test (vx · vy 0.378 s, 이론 0.375 s) 와 같은 모델 · 같은 결론. 이 스크립트는 로봇 기준 명령 변환(구간 D)을 추가로 확인"]
open(os.path.join(c.OUT, "base_velocity.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
print("\n".join(L))
