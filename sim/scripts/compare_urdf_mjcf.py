# week02 | 2링크 팔 URDF vs MJCF 비교 + 관성 변경 실험 (함태훈, B1)
# 실행: python sim/scripts/compare_urdf_mjcf.py   (저장소 루트에서)
# 출력: 두 포맷의 관절 궤적 최대 차이, 관성/질량 변경 시 낙하 거동 수치, results.csv
import csv, os, sys
import numpy as np
import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, "urdf", "two_link_arm.urdf")
MJCF = os.path.join(ROOT, "mjcf", "two_link_arm.xml")

Q0 = np.array([1.5708, 0.0])   # 어깨 90°(수평), 팔꿈치 0° 에서 놓아 자유 낙하
T_END = 5.0                    # s
DT = 0.001


def load(model, no_inertial=False, lock_elbow=False):
    import re
    xml = open(model, encoding="utf-8").read()
    if no_inertial:  # <inertial> 태그를 지우고 로드 → MuJoCo가 geom 밀도(기본 1000 kg/m³)로 자동 계산
        xml = re.sub(r"\s*<inertial[^>]*/>", "", xml)
    if lock_elbow:   # 팔꿈치 joint 제거 → 팔 전체가 한 강체(단진자)로 낙하. 관절 한계 충돌 없이 관성 효과만 본다
        xml = re.sub(r"\s*<joint name=\"elbow\"[^>]*/>", "", xml)
        xml = re.sub(r"\s*<position name=\"elbow_pos\"[^>]*/>", "", xml)
    return mujoco.MjModel.from_xml_string(xml)


def simulate(model, q0=Q0, t_end=T_END, mass_scale=None, inertia_scale=None, tag="",
             no_inertial=False, lock_elbow=False):
    """자유 낙하 시뮬레이션. 반환: 시간, q, 요약 수치"""
    m = load(model, no_inertial, lock_elbow)
    q0 = np.asarray(q0)[: m.nq]
    # 자유 낙하 비교를 위해 위치 액추에이터(kp)는 끈다 (URDF 로드 시에는 액추에이터가 아예 없음)
    if m.nu:
        m.actuator_gainprm[:, 0] = 0
        m.actuator_biasprm[:, :] = 0
    # 링크 질량/관성 변경 (base 제외: 관절 있는 body만)
    for b in range(1, m.nbody):
        if m.body_mass[b] >= 5.0:   # 베이스(5 kg) 제외
            continue
        if mass_scale is not None:
            m.body_mass[b] *= mass_scale
        if inertia_scale is not None:
            m.body_inertia[b] *= inertia_scale
    m.opt.timestep = DT
    d = mujoco.MjData(m)
    d.qpos[:] = q0
    mujoco.mj_forward(m, d)

    n = int(t_end / DT)
    t = np.arange(n) * DT
    q = np.zeros((n, m.nq)); qd = np.zeros((n, m.nv))
    for i in range(n):
        q[i] = d.qpos; qd[i] = d.qvel
        mujoco.mj_step(m, d)

    # 요약 수치: 어깨가 처음 0(수직 아래)을 지나는 시각, 최대 각속도, 5초 후 잔류 진폭
    sh = q[:, 0]
    cross = np.where(np.diff(np.sign(sh)) != 0)[0]
    t_first_cross = float(t[cross[0]]) if len(cross) else float("nan")
    peak_w = float(np.max(np.abs(qd[:, 0])))
    tail = sh[int(4.0 / DT):]
    residual_amp = float((tail.max() - tail.min()) / 2)
    return t, q, dict(tag=tag, t_first_cross=t_first_cross, peak_w=peak_w,
                      residual_amp=residual_amp,
                      mass=[round(float(x), 3) for x in m.body_mass],
                      damping=[round(float(x), 3) for x in m.dof_damping])


def main():
    rows = []
    print("=== 1) 포맷 비교: URDF vs MJCF (같은 기하·질량·관성·감쇠) ===")
    t, qu, su = simulate(URDF, tag="URDF")
    _, qm, sm = simulate(MJCF, tag="MJCF")
    diff = np.abs(qu - qm).max(axis=0)
    print(f"  URDF : {su}")
    print(f"  MJCF : {sm}")
    print(f"  5초간 관절각 최대 차이 [rad]  shoulder={diff[0]:.2e}  elbow={diff[1]:.2e}")
    # 해석값과 비교 (팔꿈치 잠금 단진자): I_pivot = 2·0.0075 + 1·0.15² + 1·0.45² = 0.24 kg·m², τ_max = g·(1·0.15+1·0.45) = 5.886 N·m
    print(f"  해석 참고: I_pivot=0.24 kg·m², τ_g,max=5.886 N·m, 무감쇠 최대 각속도 = sqrt(2·5.886/0.24) = {np.sqrt(2*5.886/0.24):.2f} rad/s")
    rows += [su, sm]

    print("\n=== 2) 관성 변경 실험 (MJCF, 팔꿈치 잠금 → 단진자 낙하, 어깨 90°에서 놓음) ===")
    print("   ※ 팔꿈치를 풀어 두면 낙하 중 팔꿈치가 관절 한계(-1.047/2.094 rad)에 부딪혀 충돌이 섞이므로, 관성 효과만 보려고 잠금")
    hit = np.abs(qm[:, 1] - np.clip(qm[:, 1], -1.047, 2.094)).max() > 1e-6 or (qm[:, 1].min() < -1.04) or (qm[:, 1].max() > 2.09)
    print(f"   (1)의 2관절 낙하에서 팔꿈치 범위: {qm[:,1].min():.3f} ~ {qm[:,1].max():.3f} rad → 한계 접촉 {'있음' if hit else '없음'}")
    for tag, kw in [("기준 (mass 1kg, I=0.0075)", dict()),
                    ("mass x5 + inertia x5", dict(mass_scale=5.0, inertia_scale=5.0)),
                    ("inertia x5 (mass 1kg 유지)", dict(inertia_scale=5.0)),
                    ("inertia x0.2 (mass 1kg 유지)", dict(inertia_scale=0.2)),
                    ("<inertial> 태그 생략 (geom 밀도로 자동)", dict(no_inertial=True))]:
        _, _, s = simulate(MJCF, tag=tag, lock_elbow=True, **kw)
        print(f"  {tag:34s} 질량={s['mass']} 첫 통과 {s['t_first_cross']:.3f}s  최대 각속도 {s['peak_w']:.2f} rad/s  4~5s 잔류 진폭 {s['residual_amp']:.3f} rad")
        rows.append(s)

    out = os.path.join(ROOT, "scripts", "results.csv")
    with open(out, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"\n저장: {out}")


if __name__ == "__main__":
    main()
