# week06 | 「집기 동작」 시나리오: 박종진 DLS IK(ik.py v1) + 5차 다항식(최소 저크) 궤적 + position 액추에이터 추종 (함태훈, B1)
#  대기(영점) → 목표 위 접근(관절공간 5차) → 수직 하강(직선 · 5차 시간척도, 점마다 IK) → 잡기(짐 질량 켬) → 상승 → 복귀
#  목표 = 박종진 ik_results.json pick_point (0.2551, 0.2867, 0.7508) m base 기준 — 4-DOF 로 수직 접근이 실제로 되는 점 (테이블 높이)
#  게인 = w06_kpkv_tune.py 선정값(고정 kv), case a2_doc 기본 · cad_ri85 병기, 중력보상 B on (로봇 자체 질량만 보상)
#  실험: 짐 0 / 0.5 / 1.0 / 3.0 kg (모르는 짐 = 보상 없음) · 1.0 kg 짐 피드포워드(짐도 gravcomp) · 1.0 kg kp×2 · 베이스 동시 명령 · week05 게인
#  기록: 관절 추종 오차, 끝점 오차, 관절 구동 토크(= qfrc_actuator, 중력보상분 포함 = jointactuatorfrc 센서), 관절 속도
#  출력: results/pick_scenario.{md,json} · pick_log_<run>.csv, assets/screenshots/week06/week06_pick_*.png (+ GIF: --gif)
import csv, json, os, sys, time
import numpy as np, mujoco
from w06_common import *

t0 = time.time()
ik = ik_module()
G6 = {j: tuple(v.values()) for j, v in json.load(open(os.path.join(OUT, "kpkv_tune.json"), encoding="utf-8"))["gains"].items()}
PICK = np.array(json.load(open(os.path.join(V1, "results", "ik_results.json"), encoding="utf-8"))["pick_point"])
ABOVE = PICK + np.array([0, 0, 0.10])
SIDE = "left"
SEG = [("대기", 0.5), ("접근", 1.5), ("하강", 1.0), ("잡기", 0.5), ("상승", 1.0), ("복귀", 1.5), ("정지", 0.5)]
NLINE = 21          # 하강 · 상승 직선 IK 점 수
VEL_LIM = 3.0       # URDF velocity (C4)


def plan():
    """관절 기준 궤적 (시각 배열, q_ref, qd_ref) — 같은 IK 해 가지를 유지하려고 q_init 을 앞 점 해로 연속"""
    m = load(); d = mujoco.MjData(m)
    q_home = np.zeros(4)
    ra = ik.solve_ik(m, d, SIDE, ABOVE, q_init=np.array([-0.6, 0.0, 0.0, 1.0]))
    line = [ra]; q = ra.q
    for s in np.linspace(0, 1, NLINE)[1:]:
        r = ik.solve_ik(m, d, SIDE, ABOVE + s * (PICK - ABOVE), q_init=q); line.append(r); q = r.q
    rp = line[-1]
    dt = m.opt.timestep; T = []; Q = []; QD = []; tcur = 0.0; marks = []
    def add(tt, qq, qd):
        T.append(tt); Q.append(qq); QD.append(qd)
    Lq = np.array([r.q for r in line])                     # 하강 경로 관절각 (s = 0 … 1)
    sgrid = np.linspace(0, 1, NLINE)
    for name, dur in SEG:
        marks.append((name, tcur, tcur + dur)); n = int(round(dur / dt))
        for k in range(n):
            t = k * dt
            if name in ("대기",): qq, qd = q_home, np.zeros(4)
            elif name == "접근": qq, qd, _ = quintic(q_home, ra.q, dur, t)
            elif name in ("하강", "상승"):
                p, v, _ = quintic(np.zeros(1), np.ones(1), dur, t); s = float(p[0]) if name == "하강" else 1 - float(p[0])
                sd = float(v[0]) * (1 if name == "하강" else -1)
                qq = np.array([np.interp(s, sgrid, Lq[:, i]) for i in range(4)])
                dqds = np.array([np.interp(s, sgrid[1:] - 0.5 / (NLINE - 1), np.diff(Lq[:, i]) / np.diff(sgrid)) for i in range(4)])
                qd = dqds * sd
            elif name == "잡기": qq, qd = rp.q, np.zeros(4)
            elif name == "복귀": qq, qd, _ = quintic(ra.q, q_home, dur, t)
            else: qq, qd = q_home, np.zeros(4)
            add(tcur + t, qq, qd)
        tcur += dur
    ik_info = dict(above=ra.as_dict(), pick=rp.as_dict(), line_pos_err_max_mm=1e3 * max(r.pos_err for r in line),
                   line_success=all(r.success for r in line), line_status=sorted({r.status for r in line}))
    return np.array(T), np.array(Q), np.array(QD), marks, ik_info


def run(name, Tr, Qr, QDr, marks, payload=0.0, ff=False, case="a2_doc", gains=G6, base_cmd=None, vel_ff=False, log=False, frames=None):
    m = load(case=case, payload=max(payload, 1e-6), payload_ff=ff, gains=gains)
    d = mujoco.MjData(m)
    qi, di, u = arm_q(m, SIDE), arm_d(m, SIDE), arm_u(m, SIDE)
    pb = m.body(f"{SIDE}_payload").id; I0 = m.body_inertia[pb].copy() / max(payload, 1e-6)
    t_grasp = [a for a in marks if a[0] == "잡기"][0][1]
    def set_payload(mass):
        m.body_mass[pb] = mass; m.body_inertia[pb] = I0 * mass
    set_payload(1e-6)
    ee = m.site(f"{SIDE}_ee").id; bu = [m.actuator(n).id for n in ("base_vx", "base_vy", "base_wz")]
    # 기준 끝점 (같은 모델 기구학, q_ref 로) — 끝점 오차 = 실제 − 기준, base_link 좌표
    dk = mujoco.MjData(m); base = m.body("base_link").id
    n = len(Tr); E = np.zeros((n, 4)); TAU = np.zeros((n, 4)); V = np.zeros((n, 4)); EE = np.zeros(n); BV = np.zeros((n, 3))
    d.ctrl[u] = Qr[0]; d.qpos[qi] = Qr[0]; mujoco.mj_forward(m, d); grasped = False
    rows = []
    for k in range(n):
        t = Tr[k]
        if not grasped and t >= t_grasp:
            set_payload(payload if payload > 0 else 1e-6); grasped = True
        d.ctrl[u] = Qr[k]
        if vel_ff:   # 속도 피드포워드: position 액추에이터 감쇠 −kv·q̇ 를 kv·q̇_ref 로 상쇄 (Isaac 의 target velocity 와 같은 효과)
            d.qfrc_applied[di] = -m.actuator_biasprm[u, 2] * QDr[k]
        if base_cmd: d.ctrl[bu] = base_cmd(t)
        mujoco.mj_step(m, d)
        E[k] = d.qpos[qi] - Qr[k]; TAU[k] = d.qfrc_actuator[di] + d.qfrc_applied[di];   # 모터가 내는 토크 = 액추에이터(중력보상분 포함) + 속도 피드포워드
        V[k] = d.qvel[di]; BV[k] = d.qvel[:3]
        dk.qpos[:] = d.qpos; dk.qpos[qi] = Qr[k]; mujoco.mj_kinematics(m, dk)
        R = d.xmat[base].reshape(3, 3)
        EE[k] = np.linalg.norm(R.T @ (d.site_xpos[ee] - dk.site_xpos[ee]))
        if log and k % 5 == 0:
            rows.append([f"{t:.3f}"] + [f"{x:.6f}" for x in (*Qr[k], *d.qpos[qi], *TAU[k])] + [f"{EE[k]*1e3:.3f}"])
        if frames is not None and k % 25 == 0: frames.append(d.qpos.copy())
    if log:
        with open(os.path.join(OUT, f"pick_log_{name}.csv"), "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f); w.writerow(["t"] + [f"qref_{a}" for a in ARM] + [f"q_{a}" for a in ARM] + [f"tau_{a}" for a in ARM] + ["ee_err_mm"]); w.writerows(rows)
    seg = {}
    for sname, ta, tb in marks:
        msk = (Tr >= ta) & (Tr < tb)
        if not msk.any(): continue
        seg[sname] = dict(err_deg_max=float(np.rad2deg(np.abs(E[msk]).max())), ee_mm_max=float(EE[msk].max() * 1e3))
    hold = (Tr >= t_grasp + 0.3) & (Tr < t_grasp + 0.5)   # 잡고 0.3 s 뒤 (정상상태에 가까운 처짐)
    for sname, ta, tb in marks:
        if sname == "유지": hold = (Tr >= tb - 0.2) & (Tr < tb)   # 뻗기 시험: 유지 구간 마지막 0.2 s
    tau_hold = TAU[hold].mean(0)
    return dict(name=name, tau_hold={a: float(tau_hold[i]) for i, a in enumerate(ARM)}, payload=payload, ff=ff, case=case, base=bool(base_cmd), vel_ff=vel_ff,
                gains={j: list(g) for j, g in gains.items()} if gains else "week05",
                err_deg_max={a: float(np.rad2deg(np.abs(E[:, i]).max())) for i, a in enumerate(ARM)},
                ee_mm_max=float(EE.max() * 1e3), ee_mm_hold=float(EE[hold].mean() * 1e3),
                err_deg_hold={a: float(np.rad2deg(E[hold, i].mean())) for i, a in enumerate(ARM)},
                tau_max={a: float(np.abs(TAU[:, i]).max()) for i, a in enumerate(ARM)},
                vel_max={a: float(np.abs(V[:, i]).max()) for i, a in enumerate(ARM)},
                base_v_max=float(np.abs(BV[:, :2]).max()), seg=seg), dict(E=E, TAU=TAU, EE=EE, V=V)


def plan_reach(q_goal, t_go=1.5, t_hold=1.0, t_back=1.5):
    """최악 자세 뻗기: 영점 → q_goal (5차) → 유지 → 영점. 짐은 처음부터 들고 있음 (잡기 시각 = 0)"""
    dt = 0.002; T = []; Q = []; QD = []; marks = [("잡기", 0.0, 0.0), ("뻗기", 0.0, t_go), ("유지", t_go, t_go + t_hold), ("복귀", t_go + t_hold, t_go + t_hold + t_back)]
    for k in range(int(round((t_go + t_hold + t_back) / dt))):
        t = k * dt
        if t < t_go: q, qd, _ = quintic(np.zeros(4), q_goal, t_go, t)
        elif t < t_go + t_hold: q, qd = np.array(q_goal), np.zeros(4)
        else: q, qd, _ = quintic(np.array(q_goal), np.zeros(4), t_back, t - t_go - t_hold)
        T.append(t); Q.append(q); QD.append(qd)
    return np.array(T), np.array(Q), np.array(QD), marks


if __name__ == "__main__":
    Tr, Qr, QDr, marks, iki = plan()
    t_desc = [a for a in marks if a[0] == "하강"][0]
    base_cmd = lambda t: ((0.3, 0.0, 0.0) if 0.5 <= t < 2.0 else (0.0, 0.0, 0.3) if 4.0 <= t < 5.5 else (0.0, 0.0, 0.0))   # 접근 중 전진 0.3 m/s, 복귀 중 회전 0.3 rad/s
    week05 = None   # week05 게인 = dampratio (load 기본)
    RUNS = [("p0", dict(payload=0.0)), ("p05", dict(payload=0.5)), ("p1", dict(payload=1.0)), ("p3", dict(payload=3.0)),
            ("p1_ff", dict(payload=1.0, ff=True)), ("p1_kp2", dict(payload=1.0, gains={j: (2 * g[0], np.sqrt(2) * g[1]) for j, g in G6.items()})),
            ("p1_velff", dict(payload=1.0, vel_ff=True)), ("p1_ff_velff", dict(payload=1.0, ff=True, vel_ff=True)),
            ("p1_base", dict(payload=1.0, base_cmd=base_cmd)), ("p1_w05", dict(payload=1.0, gains=week05)),
            ("ri85_p0", dict(payload=0.0, case="cad_ri85")), ("ri85_p05", dict(payload=0.5, case="cad_ri85")),
            ("ri85_p1", dict(payload=1.0, case="cad_ri85")), ("ri85_p3", dict(payload=3.0, case="cad_ri85"))]
    R = {}; TR = {}
    for nm, kw in RUNS:
        r, tr = run(nm, Tr, Qr, QDr, marks, log=nm in ("p0", "p1", "p1_base"), **kw); R[nm] = r; TR[nm] = tr
        print(f"{nm:12s} err° " + " ".join(f"{r['err_deg_max'][a]:.2f}" for a in ARM) + f" | ee {r['ee_mm_max']:.1f} mm (hold {r['ee_mm_hold']:.1f}) | τ " + " ".join(f"{r['tau_max'][a]:.1f}" for a in ARM) + f" | v {max(r['vel_max'].values()):.2f}")
    # 이론 검산: 잡기 자세에서 정적 중력 토크 (gravcomp 뺀 모델 qfrc_bias, qvel = 0) vs 시뮬레이션 잡은 뒤 유지 토크
    #           끝점 처짐: 모르는 짐 Δτ = Jᵀ m g → Δq = K⁻¹ Δτ (K = diag(kp)) → Δp = J Δq
    chk = {}
    for nm, pl in (("p0", 0.0), ("p05", 0.5), ("p1", 1.0), ("p3", 3.0)):
        mm = load(payload=max(pl, 1e-6), no_gravcomp=True); dd = mujoco.MjData(mm); qi, di = arm_q(mm, SIDE), arm_d(mm, SIDE)
        dd.qpos[qi] = iki_q = np.array(iki["pick"]["q"]); mujoco.mj_forward(mm, dd); tb = dd.qfrc_bias[di].copy()
        tg = [a for a in marks if a[0] == "잡기"][0][1]; hm = (Tr >= tg + 0.3) & (Tr < tg + 0.5)
        dd.qpos[qi] = iki_q + TR[nm]["E"][hm].mean(0); mujoco.mj_forward(mm, dd); tbd = dd.qfrc_bias[di].copy()   # 처진 자세에서 다시
        m0 = load(payload=1e-6, no_gravcomp=True); d0 = mujoco.MjData(m0); d0.qpos[arm_q(m0, SIDE)] = iki_q; mujoco.mj_forward(m0, d0)
        jp = np.zeros((3, m0.nv)); mujoco.mj_jacSite(m0, d0, jp, None, m0.site(f"{SIDE}_ee").id); J = jp[:, arm_d(m0, SIDE)]
        dtau = J.T @ np.array([0, 0, -pl * G]); K = np.array([G6[a][0] for a in ARM]); dp = J @ (dtau / K)
        chk[nm] = dict(tau_static=[float(x) for x in tb], tau_static_drooped=[float(x) for x in tbd], tau_sim_hold=[R[nm]["tau_hold"][a] for a in ARM],
                       ee_droop_theory_mm=float(np.linalg.norm(dp) * 1e3), ee_droop_sim_mm=R[nm]["ee_mm_hold"],
                       q_droop_theory_deg=[float(x) for x in np.rad2deg(dtau / K)], q_droop_sim_deg=[R[nm]["err_deg_hold"][a] for a in ARM])
        print("check", nm, "τ static", np.round(tb, 2), "sim", np.round(chk[nm]["tau_sim_hold"], 2), "| droop th %.2f sim %.2f mm" % (chk[nm]["ee_droop_theory_mm"], R[nm]["ee_mm_hold"]))
    # 속도 지연 이론: position 액추에이터 τ = kp(q_ref − q) − kv q̇ → 등속 구간 정상 오차 ≈ −kv q̇ / kp
    lag = {a: float(np.rad2deg(G6[a][1] * np.abs(QDr[:, i]).max() / G6[a][0])) for i, a in enumerate(ARM)}
    print("lag theory deg", {a: round(v, 2) for a, v in lag.items()}, "qd_ref max", np.abs(QDr).max(0).round(3))
    # 토크 예산 최악 조건 (커리큘럼 팀 연계): 박종진 workspace_max_torque.json 의 관절별 최악 자세로 실제 뻗기 동작 → 동적 최대
    WMT = json.load(open(os.path.join(V1, "results", "workspace_max_torque.json"), encoding="utf-8"))["results"]
    REACH = {}
    for jw in ("shoulder_pitch", "shoulder_roll"):
        for pl, key in ((0.0, "0kg"), (0.5, "0.5kg"), (1.0, "1kg"), (3.0, "3kg")):
            qg = np.array(WMT[key][jw]["q"]); Tq, Qq, QDq, mk = plan_reach(qg)
            for case in ("a2_doc", "cad_ri85"):
                r, _ = run(f"reach_{jw}_{key}_{case}", Tq, Qq, QDq, mk, payload=pl, case=case)
                r["static_wmt"] = WMT[key][jw]["tau_abs_max"]; REACH[f"{jw}|{key}|{case}"] = r
            r = REACH[f"{jw}|{key}|a2_doc"]
            print(f"reach {jw:15s} {key:5s} τmax " + " ".join(f"{r['tau_max'][a]:.2f}" for a in ARM) + f" | static {WMT[key][jw]['tau_abs_max']:.2f} | hold τ {abs(r['tau_hold'][jw]):.2f} | ee hold {r['ee_mm_hold']:.1f} mm")
    # 가감속 민감도 (구술 ②): pitch 최악 자세 3 kg, 뻗기 시간 T 를 줄이면 관성 토크 ∝ 1/T². 5차 다항식 최고 속도 = 1.875 Δq / T
    SENS = []
    qg = np.array(WMT["3kg"]["shoulder_pitch"]["q"])
    for Tg in (2.0, 1.5, 1.0, 0.75, 0.6):
        Tq, Qq, QDq, mk = plan_reach(qg, t_go=Tg); r, _ = run(f"sens_{Tg}", Tq, Qq, QDq, mk, payload=3.0)
        vpk = float(np.abs(QDq[Tq < Tg]).max()); SENS.append(dict(T=Tg, tau_pitch=r["tau_max"]["shoulder_pitch"], static=WMT["3kg"]["shoulder_pitch"]["tau_abs_max"],
                                                       qd_ref_max=vpk, within_vel_limit=vpk <= VEL_LIM, err_deg_max=r["err_deg_max"]["shoulder_pitch"]))
        print(f"sens T {Tg:.2f} s  τ pitch {r['tau_max']['shoulder_pitch']:.2f} (static 42.92)  q̇ref max {vpk:.2f} rad/s  {'OK' if vpk <= VEL_LIM else '> 3.0 rad/s'}")
    save_json("pick_scenario.json", dict(sens=SENS, reach=REACH, static_check=chk, lag_theory_deg=lag, qd_ref_max=[float(x) for x in np.abs(QDr).max(0)],pick=list(PICK), above=list(ABOVE), segments=marks, ik=iki, runs=R, gains=G6, vel_limit=VEL_LIM))

    # ---- 결과표 (md) ----
    A = ["# 집기 시나리오 (week06, 함태훈) — 박종진 IK(ik.py v1) + 5차 궤적 + position 추종", "",
         f"- 목표 pick_point ({PICK[0]:.4f}, {PICK[1]:.4f}, {PICK[2]:.4f}) m (base 기준, 박종진 ik_results.json — 4-DOF 로 수직 접근 가능한 점), 위 0.10 m 에서 직선 하강 · 상승(IK {NLINE} 점, 위치만, 연속 q_init)",
         f"- IK: 접근점 {iki['above']['status']} (오차 {1e3*iki['above']['pos_err']:.3f} mm), 잡기점 {iki['pick']['status']} ({1e3*iki['pick']['pos_err']:.3f} mm), 직선 {NLINE} 점 모두 성공 = {iki['line_success']} (최대 {iki['line_pos_err_max_mm']:.3f} mm)",
         "- 구간: " + " → ".join(f"{a} {b:.1f}–{c:.1f} s" for a, b, c in marks) + ". 짐은 「잡기」 시작에 질량을 켬(모르는 짐 = 중력보상 대상 아님)",
         "- 게인 week06(kpkv_tune.md), case a2_doc, 중력보상 B on, dt 0.002 s. 토크 = 관절 구동 토크(qfrc_actuator + 피드포워드, 중력보상분 포함)", "",
         "## 1. 실행별 결과", "", "| 실행 | 짐 kg | 조건 | 관절 추종 오차 최대 ° (pitch · roll · yaw · elbow) | 끝점 오차 최대 mm | 잡은 뒤 끝점 처짐 mm | 최대 |τ| N·m (pitch · roll · yaw · elbow) | 최고 관절 속도 rad/s |", "|---|---|---|---|---|---|---|---|"]
    cond = dict(p0="기본", p05="기본", p1="기본", p3="기본", p1_ff="짐 질량 피드포워드", p1_kp2="kp×2 · kv×√2", p1_velff="속도 피드포워드", p1_ff_velff="짐 + 속도 피드포워드",
                p1_base="베이스 동시 (접근 중 전진 0.3 m/s, 복귀 중 회전 0.3 rad/s)", p1_w05="week05 게인", ri85_p0="cad_ri85", ri85_p05="cad_ri85", ri85_p1="cad_ri85", ri85_p3="cad_ri85")
    for nm, _ in RUNS:
        r = R[nm]
        A.append(f"| {nm} | {r['payload']:g} | {cond[nm]} | " + " · ".join(f"{r['err_deg_max'][a]:.2f}" for a in ARM) + f" | {r['ee_mm_max']:.1f} | {r['ee_mm_hold']:.1f} | "
                 + " · ".join(f"{r['tau_max'][a]:.2f}" for a in ARM) + f" | {max(r['vel_max'].values()):.2f} |")
    A += ["", "## 2. 이론 검산", "", "| 항목 | 이론 | 시뮬레이션 |", "|---|---|---|"]
    for a in ("shoulder_pitch", "elbow"):
        A.append(f"| 속도 지연 {a}: kv·q̇_ref,max / kp | {lag[a]:.2f}° | {R['p1']['err_deg_max'][a]:.2f}° (짐 1 kg, 전 구간 최대) |")
    for nm in ("p05", "p1", "p3"):
        c = chk[nm]
        A.append(f"| 모르는 짐 {R[nm]['payload']:g} kg 끝점 처짐 ‖J K⁻¹ Jᵀ m g‖ | {c['ee_droop_theory_mm']:.2f} mm | {c['ee_droop_sim_mm']:.2f} mm |")
        A.append(f"| 잡기 자세 정적 토크 pitch · elbow, {R[nm]['payload']:g} kg (qfrc_bias: IK 자세 → 처진 자세) | {abs(c['tau_static'][0]):.2f} · {abs(c['tau_static'][3]):.2f} → {abs(c['tau_static_drooped'][0]):.2f} · {abs(c['tau_static_drooped'][3]):.2f} | {abs(c['tau_sim_hold'][0]):.2f} · {abs(c['tau_sim_hold'][3]):.2f} N·m (유지 구간 평균) |")
    A += ["", "- 시뮬레이션 유지 토크는 처진 자세에서 다시 계산한 정적 토크와 같음(3 kg 은 처짐 16 mm 로 pitch 토크가 IK 자세보다 작아짐 — 잡기 자세에서 pitch 팔길이가 처지며 줄어드는 방향)",
          "", "## 3. 정리 (구술 ① 근거)", "",
          f"- 짐 0 → 3 kg: 관절 오차 최대는 거의 같음(pitch 1.09 → {R['p3']['err_deg_max']['shoulder_pitch']:.2f}°) — 오차의 대부분은 **속도 지연** kv·q̇/kp (position 액추에이터가 −kv·q̇ 로 감쇠하므로 움직이는 동안 kp·e = kv·q̇ 만큼 뒤처짐). 짐은 **잡은 뒤 정적 처짐** Δτ/kp 로 나타남: 0.5 · 1 · 3 kg → {R['p05']['ee_mm_hold']:.1f} · {R['p1']['ee_mm_hold']:.1f} · {R['p3']['ee_mm_hold']:.1f} mm",
          f"- 줄이는 법 (짐 1 kg): kp×2 → 처짐 {R['p1']['ee_mm_hold']:.1f} → {R['p1_kp2']['ee_mm_hold']:.1f} mm(절반, 대신 백래시 · 센서 잡음에 민감, 접촉 시 충격 큼) · 짐 질량 피드포워드 → {R['p1_ff']['ee_mm_hold']:.1f} mm(짐 질량을 알아야 함) · 속도 피드포워드 → 전 구간 최대 {R['p1']['ee_mm_max']:.1f} → {R['p1_velff']['ee_mm_max']:.1f} mm · 둘 다 → {R['p1_ff_velff']['ee_mm_max']:.1f} mm",
          f"- 베이스 동시 명령: 끝점 오차 {R['p1']['ee_mm_max']:.1f} → {R['p1_base']['ee_mm_max']:.1f} mm, yaw 토크 {R['p1']['tau_max']['shoulder_yaw']:.2f} → {R['p1_base']['tau_max']['shoulder_yaw']:.2f} N·m — 베이스 가속(시정수 0.38 s) 관성력이 작아 영향 작음",
          f"- case cad_ri85: 토크 · 오차 차이 ≤ 0.1 — 집기 속도에서 armature 관성 토크(armature × q̈)가 작음",
          f"- 최고 관절 속도 {max(max(r['vel_max'].values()) for r in R.values()):.2f} rad/s ≤ 3.0 (URDF velocity)",
          "", "그림: `assets/screenshots/week06/week06_pick_tracking.png` · `week06_pick_payload.png` · `week06_pick.gif`. 로그 CSV: `results/pick_log_{p0,p1,p1_base}.csv`"]
    save_md("pick_scenario.md", "\n".join(A) + "\n")

    plt = plot_style()
    tt = Tr
    fig, ax = plt.subplots(3, 1, figsize=(9, 7.2), sharex=True)
    for i, a in enumerate(ARM):
        ax[0].plot(tt, np.rad2deg(Qr[:, i]), color=JCOL[a], lw=1.6, label=a)
        ax[0].plot(tt, np.rad2deg(Qr[:, i] + TR["p1"]["E"][:, i]), "--", color=JCOL[a], lw=1.0)
        ax[1].plot(tt, np.rad2deg(TR["p1"]["E"][:, i]), color=JCOL[a], lw=1.4)
    for nm, col, lab in (("p0", "#8a8986", "짐 0 kg"), ("p05", "#eda100", "0.5 kg"), ("p1", "#2a78d6", "1.0 kg"), ("p1_ff", "#1baf7a", "1.0 kg + 짐 피드포워드")):
        ax[2].plot(tt, TR[nm]["TAU"][:, 0], color=col, lw=1.4, label=f"pitch {lab}")
    for x in ax:
        for sname, ta, tb in marks: x.axvline(ta, color="#e6e5e1", lw=0.8)
    for sname, ta, tb in marks: ax[0].text((ta + tb) / 2, ax[0].get_ylim()[1], sname, ha="center", va="bottom", fontsize=8, color="#52514e")
    ax[0].set_ylabel("관절각 (°)  실선 기준 · 점선 실제"); ax[0].legend(ncol=4, loc="lower left")
    ax[1].set_ylabel("추종 오차 q − q_ref (°)\n짐 1.0 kg (모르는 짐)")
    ax[2].set_ylabel("shoulder_pitch 구동 토크 (N·m)"); ax[2].set_xlabel("t (s)"); ax[2].legend(ncol=2, loc="lower left")
    fig.suptitle("집기 시나리오 — IK(박종진 ik.py) + 5차 궤적, a2_doc · 게인 week06", fontsize=10); fig.tight_layout()
    fig.savefig(os.path.join(IMG, "week06_pick_tracking.png"), dpi=130)

    fig, ax = plt.subplots(1, 2, figsize=(10, 3.4))
    names = [("p0", "0"), ("p05", "0.5"), ("p1", "1.0"), ("p3", "3.0")]
    x = np.arange(4); w = 0.2
    for i, a in enumerate(ARM):
        ax[0].bar(x + (i - 1.5) * w, [R[n]["tau_max"][a] for n, _ in names], w, color=JCOL[a], label=a)
    ax[0].set_xticks(x, [f"{l} kg" for _, l in names]); ax[0].set_ylabel("집기 중 최대 |τ| (N·m)"); ax[0].legend(fontsize=7)
    ax[0].axhline(52, color="#c9c8c4", lw=1); ax[0].text(3.45, 52, "PDR 52", fontsize=7, va="bottom", ha="right", color="#52514e")
    labs = [("p1", "기본"), ("p1_ff", "짐 FF"), ("p1_kp2", "kp×2"), ("p1_velff", "속도 FF"), ("p1_ff_velff", "짐+속도 FF"), ("p1_base", "베이스 동시"), ("p1_w05", "week05 게인")]
    ax[1].bar(range(len(labs)), [R[n]["ee_mm_max"] for n, _ in labs], color="#2a78d6", label="전 구간 최대")
    ax[1].bar(range(len(labs)), [R[n]["ee_mm_hold"] for n, _ in labs], color="#eb6834", width=0.5, label="잡은 뒤 유지")
    ax[1].set_xticks(range(len(labs)), [l for _, l in labs], fontsize=7); ax[1].set_ylabel("끝점 오차 (mm), 짐 1.0 kg"); ax[1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(os.path.join(IMG, "week06_pick_payload.png"), dpi=130)

    if "--gif" in sys.argv:
        import imageio.v2 as imageio
        frames = []; run("gif", Tr, Qr, QDr, marks, payload=1.0, base_cmd=None, frames=frames)
        m = load(payload=1.0); d = mujoco.MjData(m); rnd = mujoco.Renderer(m, 360, 480)
        cam = mujoco.MjvCamera(); cam.azimuth = 135; cam.elevation = -15; cam.distance = 2.2; cam.lookat[:] = [0.2, 0.2, 0.95]
        opt = mujoco.MjvOption(); opt.sitegroup[3] = 1
        imgs = []
        for qp in frames:
            d.qpos[:] = qp; mujoco.mj_forward(m, d); rnd.update_scene(d, cam, opt); imgs.append(rnd.render())
        imageio.mimsave(os.path.join(IMG, "week06_pick.gif"), imgs, duration=0.05, loop=0)
        print("gif frames", len(imgs))
    print(f"done {time.time()-t0:.1f} s")
