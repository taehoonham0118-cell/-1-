# week03 | 2링크 팔 position 액추에이터 계단 응답 실험 — kp · kv · armature 변화 비교 (함태훈, B1)
# 실행: python sim/scripts/step_response.py          (저장소 루트에서)
# 출력: sim/scripts/step_results.csv,
#       assets/screenshots/week03/week03_step_kp.png / week03_step_kv.png / week03_step_armature.png
#
# 실험 조건
#   - 모델: sim/mjcf/scene.xml (= two_link_arm_v2.xml include)
#   - 초기 자세 q = [0, 0] (아래로 늘어짐), t = 0 에서 어깨 목표 ctrl = 0.5 rad 계단 입력, 팔꿈치 목표 0
#   - position 액추에이터 토크: τ = kp·(ctrl − q) − kv·q̇  (forcerange ±25 N·m 로 포화)
#   - 각 조건 2 s 시뮬레이션, timestep 1 ms, integrator implicitfast
#   - 어깨 기준 팔 관성(팔꿈치 0°) I ≈ 2·0.0075 + 1·0.15² + 1·0.45² = 0.24 kg·m²
#     → 무감쇠 고유진동수 ωn = √(kp/I), 임계감쇠 kv_c = 2·√(kp·I)
import csv, os
import numpy as np
import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))          # sim/
REPO = os.path.dirname(ROOT)                                                # 저장소 루트
MODEL = os.path.join(ROOT, "mjcf", "scene.xml")
OUT_CSV = os.path.join(ROOT, "scripts", "step_results.csv")
FIG_DIR = os.path.join(REPO, "assets", "screenshots", "week03")

TARGET = 0.5      # rad, 어깨 계단 목표
T_END = 2.0       # s
DT = 0.001
I_ARM = 0.24      # kg·m², 해석용 어깨 기준 관성 (팔꿈치 잠금 근사)


def run(kp, kv, armature):
    m = mujoco.MjModel.from_xml_path(MODEL)
    m.opt.timestep = DT
    # position 액추에이터 내부 표현: gainprm[0] = kp, biasprm[1] = −kp, biasprm[2] = −kv
    m.actuator_gainprm[:, 0] = kp
    m.actuator_biasprm[:, 1] = -kp
    m.actuator_biasprm[:, 2] = -kv
    m.dof_armature[:] = armature            # 모든 관절에 같은 armature (로터 반영 관성)
    d = mujoco.MjData(m)
    d.qpos[:] = 0.0
    mujoco.mj_forward(m, d)

    n = int(T_END / DT)
    t = np.arange(n) * DT
    q = np.zeros(n); tau = np.zeros(n)
    sh_act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "shoulder_pos")
    for i in range(n):
        d.ctrl[sh_act] = TARGET          # 어깨 목표 0.5 rad, 팔꿈치 목표 0 (기본값)
        mujoco.mj_step(m, d)
        q[i] = d.qpos[0]
        tau[i] = d.actuator_force[sh_act]

    # 성능 지표
    final = q[int(1.0 / DT):].mean()                       # 1~2 s 평균 (감쇠 없는 경우엔 진동 중심 ≈ 평형점)
    ss_err = TARGET - final                                # 정상상태 오차 (중력 때문에 남음)
    peak = q.max(); overshoot = max(0.0, (peak - final) / abs(final) * 100) if abs(final) > 1e-6 else 0.0
    i10 = np.argmax(q >= 0.1 * final); i90 = np.argmax(q >= 0.9 * final)
    rise = t[i90] - t[i10] if q.max() >= 0.9 * final else float("nan")
    band = 0.02 * abs(final)
    outside = np.where(np.abs(q - final) > band)[0]
    settle = t[outside[-1] + 1] if len(outside) and outside[-1] + 1 < n else (0.0 if not len(outside) else float("nan"))
    settled = not np.isnan(settle)                         # 2 s 안에 ±2 % 띠에 들어와 머물면 True
    return t, q, tau, dict(kp=kp, kv=kv, armature=armature, settled=settled, final=final, ss_err=ss_err,
                           overshoot_pct=overshoot, rise_10_90=rise, settle_2pct=settle,
                           tau_peak=float(np.abs(tau).max()),
                           wn_theory=float(np.sqrt(kp / (I_ARM + armature))),
                           kv_crit_theory=float(2 * np.sqrt(kp * (I_ARM + armature))))


def plot(cases, title, fname, label_fn):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams["font.family"] = ["NanumGothic", "Malgun Gothic", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(2, 1, figsize=(8, 7), sharex=True)
    for t, q, tau, s in cases:
        ax[0].plot(t, q, label=label_fn(s))
        ax[1].plot(t, tau)
    ax[0].axhline(TARGET, color="k", ls="--", lw=0.8, label="target 0.5 rad")
    ax[0].set_ylabel("shoulder q [rad]"); ax[0].legend(); ax[0].grid(alpha=0.3); ax[0].set_title(title)
    ax[1].axhline(25, color="r", ls=":", lw=0.8); ax[1].axhline(-25, color="r", ls=":", lw=0.8)
    ax[1].set_ylabel("actuator torque [N·m]"); ax[1].set_xlabel("time [s]"); ax[1].grid(alpha=0.3)
    fig.tight_layout()
    os.makedirs(FIG_DIR, exist_ok=True)
    fig.savefig(os.path.join(FIG_DIR, fname), dpi=130)
    plt.close(fig)


def main():
    rows = []
    # (1) kp 변화 (kv = 0, armature = 0): 강성 ↑ → 빠르고 정상상태 오차 ↓, 감쇠 없어 진동 ↑
    kp_cases = [run(kp, 0.0, 0.0) for kp in (20, 50, 200)]
    plot(kp_cases, "(1) kp sweep  (kv=0, armature=0)", "week03_step_kp.png",
         lambda s: f"kp={s['kp']:.0f}  (ωn≈{s['wn_theory']:.1f} rad/s)")
    # (2) kv 변화 (kp = 200): 감쇠 ↑ → 오버슈트 ↓, 임계감쇠 kv_c = 2√(kp·I) ≈ 13.9
    kv_cases = [run(200, kv, 0.0) for kv in (0, 5, 14)]
    plot(kv_cases, "(2) kv sweep  (kp=200, armature=0)   kv_crit = 2√(kp·I) ≈ 13.9", "week03_step_kv.png",
         lambda s: f"kv={s['kv']:.0f}")
    # (3) armature 변화 (kp = 200, kv = 5): 로터 반영 관성 ↑ → 유효 관성 ↑ → 느려지고 진동 주기 ↑
    arm_cases = [run(200, 5.0, a) for a in (0.0, 0.01, 0.1)]
    plot(arm_cases, "(3) armature sweep  (kp=200, kv=5)   armature = J_rotor·N²", "week03_step_armature.png",
         lambda s: f"armature={s['armature']:g} kg·m²  (ωn≈{s['wn_theory']:.1f})")

    for group, cases in (("kp", kp_cases), ("kv", kv_cases), ("armature", arm_cases)):
        for _, _, _, s in cases:
            s = dict(group=group, **s); rows.append(s)
            print(f"[{group:8s}] kp={s['kp']:5.0f} kv={s['kv']:4.0f} arm={s['armature']:<5g} | "
                  f"{'settled' if s['settled'] else 'OSCILL.'} final={s['final']:.4f} rad  ss_err={s['ss_err']:.4f}  overshoot={s['overshoot_pct']:5.1f} %  "
                  f"rise={s['rise_10_90']:.3f} s  settle(2%)={s['settle_2pct']:.3f} s  τ_peak={s['tau_peak']:.1f} N·m  "
                  f"| theory ωn={s['wn_theory']:.1f} rad/s, kv_c={s['kv_crit_theory']:.1f}")
    with open(OUT_CSV, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print("saved:", OUT_CSV, "and", FIG_DIR)


if __name__ == "__main__":
    main()
