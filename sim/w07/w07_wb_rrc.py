# week07 | 작업공간 속도 제어(resolved-rate) — 베이스 + 팔 협조 직선 추종 (함태훈, B1)
#  자코비안 · 가중치 = wb_jac.py (박종진 wb_ik 인터페이스 v2 공유 시 교체)
#  제어 (100 Hz, 시뮬 500 Hz):  ẋ_cmd = ẋ_ref + K (x_ref − x)                  (폐루프 CLIK, K = 10 1/s)
#                              q̇ = J# ẋ_cmd + (I − J# J) q̇0,  J# = W⁻¹Jᵀ(JW⁻¹Jᵀ + λ²I)⁻¹   (λ 0.03, q̇0 = 팔 한계 회피)
#                              속도 한계: 팔 3.0 rad/s · 베이스 1.5 m/s · 1.571 rad/s (넘으면 q̇ 전체를 같은 비율로 줄임 = 방향 유지)
#   - 베이스: velocity 액추에이터(kv 200) 에 world 속도 q̇[TX, TY], q̇[RZ] (MuJoCo slide 축은 world 고정)
#             실제 베이스 명령(로봇 기준) = R(yaw)ᵀ [ẋ, ẏ]  → 로그 v_body 로 기록
#   - 팔: position 목표 = 속도 적분 q_ref += q̇ Δt
#  경로: A = 테이블 높이 z 0.75 m(6주차 pick_point 높이) 전방 직선 x 0.30 → 0.80 m (팔 단독 최대 도달 x ≈ 0.44 m @ z 0.7 ~ 0.8, 박종진 workspace.md)
#        B = 측방 직선 z 0.90 m, x 0.40, y 0.40 → −0.20 m (왼팔이 정중면 넘어 0.2 m — 팔 단독은 0.095 m 까지)
#  비교: 팔만(4 DOF) / 전신 W = 10(기본, 팔 우선) / W = 1 / W = 100, 경로 A 는 3 kg(아는 짐) 도 — 토크 예산(w07_base_torque.py) 입력
#  * 노션 예시 0.6 m 는 바닥 기준이면 팔 최저 도달 0.606 m(박종진 workspace.md) 아래 — 베이스는 높이를 못 바꾸므로 전신으로도 불가 → 6주차 테이블 높이 0.75 m 사용
#  출력: results/wb_rrc.{md,json}, wb_log_<run>.csv, assets/screenshots/week07/week07_wb_line.png · week07_wb_line.gif(--gif)
#  실행: python sim/w07/w07_wb_rrc.py [--gif]   (~20 s, GIF 포함 ~60 s · 리눅스 헤드리스는 MUJOCO_GL=egl)
import sys, csv, time
from w07_common import *
from wb_jac import wb_jacobian, wb_weights, wdls, limit_avoid
sys.path.insert(0, os.path.join(V1, "scripts")); import ik

t0 = time.time(); SIDE = "left"; DT_C = 0.01; KCL = 10.0; LAM = 0.03; T_SET = 0.5; T_PATH = 4.0; T_HOLD = 1.0
VLIM = dict(arm=3.0, lin=1.5, yaw=np.pi / 2)
PATHS = {"A": (np.array([0.30, 0.2867, 0.75]), np.array([0.80, 0.2867, 0.75])),
         "B": (np.array([0.40, 0.40, 0.90]), np.array([0.40, -0.20, 0.90]))}
RUNS = [("A", "arm", None, 0.0), ("A", "wb10", 10, 0.0), ("A", "wb1", 1, 0.0), ("A", "wb100", 100, 0.0), ("A", "wb10_3kg", 10, 3.0),
        ("B", "arm", None, 0.0), ("B", "wb10", 10, 0.0), ("B", "wb1", 1, 0.0), ("B", "wb100", 100, 0.0)]


def run(path, name, w, payload, frames=None):
    m = load(payload=payload, payload_gc=1.0, sides=(SIDE,)) if payload else load()
    d = mujoco.MjData(m); qi = arm_q(m, SIDE); u = arm_u(m, SIDE); bu = base_u(m); bq = base_q(m); bd = base_d(m)
    inc = w is not None; p0, p1 = PATHS[path]; L = np.linalg.norm(p1 - p0); dirv = (p1 - p0) / L
    r = ik.solve_ik(m, d, SIDE, p0); q = r.q.copy()
    d.qpos[qi] = q; d.ctrl[u] = q; qr = arm_q(m, "right"); d.qpos[qr] = 0; mujoco.mj_forward(m, d)
    W = wb_weights(inc, w or 1, w or 1); sid = m.site(f"{SIDE}_ee").id; dofs_arm = arm_d(m, SIDE)
    nsub = int(round(DT_C / m.opt.timestep)); n_c = int((T_SET + T_PATH + T_HOLD) / DT_C)
    log = []; qdot = np.zeros(len(W)); vb_prev = np.zeros(3)
    for kc in range(n_c):
        t = kc * DT_C; s_, sd_, _ = quintic(0.0, 1.0, T_PATH, max(t - T_SET, 0.0))
        xr = p0 + dirv * L * s_; xdr = dirv * L * sd_ if t >= T_SET else np.zeros(3)
        x = d.site_xpos[sid].copy(); xc = xdr + KCL * (xr - x)
        J, _ = wb_jacobian(m, d, SIDE, inc); Jw = wdls(J, W, LAM)
        qdot = Jw @ xc + (np.eye(len(W)) - Jw @ J) @ limit_avoid(m, d, SIDE, inc, k=0.5)
        sc = max(np.abs(qdot[-4:]).max() / VLIM["arm"], (np.linalg.norm(qdot[:2]) / VLIM["lin"]) if inc else 0, (abs(qdot[2]) / VLIM["yaw"]) if inc else 0, 1.0)
        qdot = qdot / sc
        q = np.clip(q + qdot[-4:] * DT_C, m.jnt_range[[m.joint(f"{SIDE}_{a}_joint").id for a in ARM], 0], m.jnt_range[[m.joint(f"{SIDE}_{a}_joint").id for a in ARM], 1])
        d.ctrl[u] = q
        if inc: d.ctrl[bu] = qdot[:3]
        for _ in range(nsub): mujoco.mj_step(m, d)
        x = d.site_xpos[sid]; e = x - xr; e_path = e - dirv * (e @ dirv)
        yaw = d.qpos[bq[2]]; vw = d.qvel[bd[:2]]; Rz = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        vb = d.qvel[bd].copy(); ab = (vb - vb_prev) / DT_C; vb_prev = vb
        log.append(dict(t=t + DT_C, xr=xr.tolist(), x=x.tolist(), e=float(np.linalg.norm(e)), e_path=float(np.linalg.norm(e_path)),
                        base=d.qpos[bq].tolist(), v_world=vb.tolist(), v_body=(Rz @ vw).tolist() + [float(vb[2])], a_base=ab.tolist(),
                        q=d.qpos[qi].tolist(), qd=d.qvel[qi].tolist(), tau=drive_torque(m, d, dofs_arm).tolist(), scale=float(sc),
                        cmd_base=(qdot[:3].tolist() if inc else [0, 0, 0])))
        if frames is not None and kc % 5 == 0: frames.append((d.qpos.copy(), xr.copy()))
    lg = log; tt = np.array([g["t"] for g in lg]); mv = (tt > T_SET) & (tt <= T_SET + T_PATH); fin = tt > T_SET + T_PATH + 0.5
    E = np.array([g["e"] for g in lg]); EP = np.array([g["e_path"] for g in lg]); B = np.array([g["base"] for g in lg])
    jid = [m.joint(f"{SIDE}_{a}_joint").id for a in ARM]; lo, hi = m.jnt_range[jid, 0], m.jnt_range[jid, 1]
    Q = np.array([g["q"] for g in lg]); marg = np.rad2deg(np.minimum(Q - lo, hi - Q).min())
    VB = np.array([g["v_world"] for g in lg]); AB = np.array([g["a_base"] for g in lg]); T = np.array([g["tau"] for g in lg])
    res = dict(path=path, run=name, w=w, payload=payload, ik_start=r.status, e_max_mm=1e3 * E[mv].max(), e_rms_mm=1e3 * np.sqrt((E[mv] ** 2).mean()),
               e_path_max_mm=1e3 * EP[mv].max(), e_final_mm=1e3 * E[fin].mean(), base_dx=float(B[-1, 0] - B[0, 0]), base_dy=float(B[-1, 1] - B[0, 1]),
               base_dyaw_deg=float(np.rad2deg(B[-1, 2] - B[0, 2])), base_path_m=float(np.sum(np.linalg.norm(np.diff(B[:, :2], axis=0), axis=1))),
               v_base_max=float(np.linalg.norm(VB[:, :2], axis=1).max()), a_base_max=float(np.linalg.norm(AB[:, :2], axis=1).max()),
               w_base_max=float(np.abs(VB[:, 2]).max()), qd_arm_max=float(np.abs(np.array([g["qd"] for g in lg])).max()),
               joint_margin_min_deg=float(marg), vel_scaled_frac=float(np.mean([g["scale"] > 1.0001 for g in lg])),
               tau_max=np.abs(T).max(0).tolist(), reached=bool(1e3 * E[fin].mean() < 5.0))
    with open(os.path.join(OUT, f"wb_log_{path}_{name}.csv"), "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f); wr.writerow(["t", "x_ref", "y_ref", "z_ref", "x", "y", "z", "err_m", "path_err_m", "base_x", "base_y", "base_yaw",
                                         "vbody_x", "vbody_y", "wz", "tau_pitch", "tau_roll", "tau_yaw", "tau_elbow"])
        for g in lg: wr.writerow([f"{g['t']:.3f}"] + [f"{v:.5f}" for v in g["xr"] + g["x"] + [g["e"], g["e_path"]] + g["base"] + g["v_body"] + g["tau"]])
    return res, lg, m


RES = {}; LOGS = {}; GIF = "--gif" in sys.argv; frames = [] if GIF else None
for path, name, w, pay in RUNS:
    res, lg, mm = run(path, name, w, pay, frames=frames if (GIF and path == "A" and name == "wb10") else None)
    RES[f"{path}_{name}"] = res; LOGS[f"{path}_{name}"] = lg
save_json("wb_rrc.json", dict(results=RES, paths={k: [v[0].tolist(), v[1].tolist()] for k, v in PATHS.items()},
                              ctrl=dict(dt=DT_C, K=KCL, lam=LAM, T_path=T_PATH, vlim=VLIM, null="limit_avoid k 0.5")))

lab = dict(arm="팔만 (4 DOF)", wb10="전신 W 10 (팔 우선)", wb1="전신 W 1", wb100="전신 W 100", wb10_3kg="전신 W 10 + 3 kg(FF)")
Lm = ["# 베이스 + 팔 협조 resolved-rate 직선 추종 (week07, 함태훈) — 자동 생성 `w07_wb_rrc.py`", "",
      f"제어 100 Hz / 시뮬 500 Hz, CLIK K {KCL:g} 1/s, 가중 DLS λ {LAM}, 영공간 = 팔 관절 한계 회피, 속도 한계 팔 3.0 rad/s · 베이스 1.5 m/s · 90 °/s. "
      f"경로 5차(최소 저크) {T_PATH:g} s + 유지 {T_HOLD:g} s. 위치만(3×7, 4-DOF 라 그리퍼 축은 자유). 통일 게인 · 중력보상 B · a2_doc",
      "", "- 경로 A: z 0.75 m 전방 직선 x 0.30 → 0.80 m, y 0.287 (6주차 pick_point 높이 · y). 팔 단독 도달 x ≈ 0.44 m (z 0.7 ~ 0.8, 박종진 workspace.md)",
      "- 경로 B: z 0.90 m 측방 직선 y 0.40 → −0.20 m, x 0.40 (왼팔이 정중면 너머 0.2 m — 팔 단독은 0.095 m 까지)", "",
      "| 경로 | 방식 | 이동 중 오차 최대 / RMS mm | 경로(직선 수직) 오차 최대 mm | 끝 오차 mm | 도달 | 베이스 Δx · Δy m · Δyaw ° | 베이스 최대 속도 m/s · 가속 m/s² | 팔 최대 속도 rad/s | 관절 한계 최소 여유 ° |",
      "|---|---|---|---|---|---|---|---|---|---|"]
for k, r in RES.items():
    Lm.append(f"| {r['path']} | {lab[r['run']]} | {r['e_max_mm']:.1f} / {r['e_rms_mm']:.1f} | {r['e_path_max_mm']:.1f} | {r['e_final_mm']:.1f} | {'O' if r['reached'] else '**X**'} | "
              f"{r['base_dx']:.3f} · {r['base_dy']:.3f} · {r['base_dyaw_deg']:.1f} | {r['v_base_max']:.3f} · {r['a_base_max']:.2f} | {r['qd_arm_max']:.2f} | {r['joint_margin_min_deg']:.1f} |")
Lm += ["", "- 가중치 W(베이스 x · y · yaw, 팔 = 1): 클수록 베이스를 덜 쓰고 팔을 먼저 씀. 팔이 뻗어 그 방향 자코비안이 작아지면(특이점 근처) 그 성분을 베이스가 가져감",
       "- 영공간 차원 = 7 − 3 = 4 (팔만 4 − 3 = 1): 같은 끝점 속도에서 팔 한계 회피 · 베이스 이동 최소 같은 2차 목표에 씀",
       "- 베이스 명령은 world 속도를 로봇 기준 R(yaw)ᵀ 로 바꿔 기록(`wb_log_*.csv` vbody_x · vbody_y) — MuJoCo slide 는 world 고정이라 시뮬에선 world 그대로 넣음",
       f"", f"실행 시간 {time.time() - t0:.0f} s"]
save_md("wb_rrc.md", "\n".join(Lm) + "\n")

plt = plot_style()
fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
for i, (path, title) in enumerate((("A", "경로 A 전방 직선 (z 0.75 m)"), ("B", "경로 B 측방 직선 (z 0.90 m)"))):
    for k, c in (("arm", SERIES[1]), ("wb10", SERIES[0]), ("wb1", SERIES[2]), ("wb100", SERIES[3])):
        lg = LOGS[f"{path}_{k}"]; tt = [g["t"] for g in lg]
        ax[i].plot(tt, [1e3 * g["e"] for g in lg], color=c, lw=1.6, label=lab[k])
    ax[i].set_yscale("log"); ax[i].set_ylim(0.05, 1000); ax[i].set_title(title); ax[i].set_xlabel("시간 [s]"); ax[i].set_ylabel("끝점 오차 [mm]")
    ax[i].axvspan(T_SET, T_SET + T_PATH, color="#e6e5e1", alpha=0.4, lw=0)
ax[0].legend()
for k, c in (("arm", SERIES[1]), ("wb10", SERIES[0]), ("wb1", SERIES[2]), ("wb100", SERIES[3])):
    lg = LOGS[f"A_{k}"]; ax[2].plot([g["x"][0] for g in lg], [g["x"][2] for g in lg], color=c, lw=1.6)
    if k != "arm": ax[2].plot([g["base"][0] + 0.0 for g in lg], [0.42] * len(lg), color=c, lw=4, alpha=0.5)
ax[2].plot([PATHS["A"][0][0], PATHS["A"][1][0]], [0.75, 0.75], color="#52514e", ls="--", lw=1, label="목표 직선")
ax[2].set_xlabel("x [m] (world)"); ax[2].set_ylabel("z [m]"); ax[2].set_title("경로 A 끝점 (선) · 베이스 x 이동 (굵은 띠)"); ax[2].legend()
fig.tight_layout(); fig.savefig(os.path.join(IMG, "week07_wb_line.png"), dpi=130); plt.close(fig)

if GIF:
    import imageio
    mg = load(); rd = mujoco.Renderer(mg, 360, 480); dg = mujoco.MjData(mg); cam = mujoco.MjvCamera(); imgs = []
    cam.azimuth = 135; cam.elevation = -20; cam.distance = 2.6
    for qp, xr in frames:
        dg.qpos[:] = qp; mujoco.mj_forward(mg, dg); cam.lookat[:] = [dg.qpos[base_q(mg)[0]] + 0.3, 0.1, 0.8]
        rd.update_scene(dg, cam); sc = rd.scene
        for a_, b_, rgba, wdt in ((PATHS["A"][0], PATHS["A"][1], [0.2, 0.2, 0.2, 0.8], 0.004), (xr, xr + [0, 0, 0.001], [0.9, 0.2, 0.1, 1], 0.015)):
            if sc.ngeom < sc.maxgeom:
                g = sc.geoms[sc.ngeom]; mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.array(rgba, np.float32))
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, wdt, np.array(a_, float), np.array(b_, float)); sc.ngeom += 1
        imgs.append(rd.render().copy())
    imageio.mimsave(os.path.join(IMG, "week07_wb_line.gif"), imgs, duration=0.05, loop=0); rd.close()
print(open(os.path.join(OUT, "wb_rrc.md"), encoding="utf-8").read())
