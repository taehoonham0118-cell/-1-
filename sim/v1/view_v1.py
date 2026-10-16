# week05 | 노트북 MuJoCo 뷰어로 v1 확인 · 캡처 (함태훈, B1) — 커리큘럼 5주차 수: 「뷰어에서 모든 액추에이터 슬라이더가 정상 동작하는 캡처」
# 실행 (5주차_작업 폴더 = 저장소 루트에서):  py sim/v1/view_v1.py <모드>
#   sliders  : (기본) 중력보상 B 를 켠 모델을 뷰어로 연다. 오른쪽 Control 패널 슬라이더 11개(base_vx·vy·wz + 팔 8)를 직접 움직여 확인
#   sag      : 양팔 앞으로 수평(pitch −90°) 목표, 중력보상 없음 → kp 300 이라 약 4.8° 처짐 (검증 ②)
#   gravcomp : 같은 자세, 중력보상 B(gravcomp + actuatorgravcomp) → 처짐 0°
#   mirror   : 양팔에 같은 관절각 명령 (pitch −60°, roll 40°, yaw 30°, elbow 60°) + 중력보상 B → 좌우 거울 대칭 (검증 ⑤ · 팀 연계)
# 콘솔에 1 초마다 최대 관절 오차[°]가 찍힌다. 창을 닫으면 끝. 한글 경로는 v1_common.load() 가 메모리로 읽어서 괜찮음
import sys, time
import numpy as np, mujoco, mujoco.viewer
import v1_common as c

mode = sys.argv[1] if len(sys.argv) > 1 else "sliders"
POSE = {"sag": [-np.pi / 2, 0, 0, 0], "gravcomp": [-np.pi / 2, 0, 0, 0], "mirror": np.radians([-60, 40, 30, 60])}
m = c.load(xml_edit=c.gravcomp_edit if mode in ("sliders", "gravcomp", "mirror") else None)
d = mujoco.MjData(m)
print(f"[view_v1] 모드 {mode} · MuJoCo {mujoco.__version__} · nu {m.nu} · {sum(m.body_mass):.3f} kg · 중력보상 B {'켬' if m.jnt_actgravcomp.any() else '끔'}")

if mode == "sliders":
    print("  오른쪽 Control 패널 슬라이더로 관절 · 베이스를 움직여 보세요 (팔: 목표 각도 rad, 베이스: 속도). 캡처 = 화면 캡처(Win+Shift+S)")
    mujoco.viewer.launch(m, d)
    sys.exit()

for s in c.SIDES: c.set_arm(m, d, POSE[mode], s)
mujoco.mj_forward(m, d)
acts = {f"{s}_{a}": m.actuator(f"{s}_{a}").id for s in c.SIDES for a in c.ARM}
q0 = {n: d.qpos[c.qa(m, f"{n}_joint")] for n in acts}
with mujoco.viewer.launch_passive(m, d) as v:
    v.cam.azimuth, v.cam.elevation, v.cam.distance, v.cam.lookat[:] = (135 if mode != "mirror" else 180), -15, 2.2, (0.2, 0, 1.0)
    t_print = 0.0
    while v.is_running():
        t0 = time.time()
        for n, i in acts.items(): d.ctrl[i] = q0[n]
        d.ctrl[:3] = 0
        mujoco.mj_step(m, d)
        if d.time - t_print >= 1.0:
            t_print = d.time
            e = max(abs(d.qpos[c.qa(m, f"{n}_joint")] - q0[n]) for n in acts)
            L = d.site_xpos[m.site("left_ee").id]; R = d.site_xpos[m.site("right_ee").id]
            print(f"  t {d.time:4.1f} s  최대 관절 오차 {np.degrees(e):.3f}°  left_ee {np.round(L, 4)}  right_ee {np.round(R, 4)}")
        v.sync()
        time.sleep(max(0.0, m.opt.timestep - (time.time() - t0)))
