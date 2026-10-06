# week03 | mjSpec attach 연습 — 2링크 팔 모델을 (임시) 홀로노믹 베이스에 좌·우 2개 붙이기 (함태훈, B1)
# 실행: python sim/scripts/attach_arm_to_base.py        (저장소 루트에서)
# 출력: sim/mjcf/arm_on_base.xml  →  python -m mujoco.viewer --mjcf=sim/mjcf/arm_on_base.xml
#
# USD 의 reference(다른 파일의 prim 을 내 stage 에 가져와 재사용)에 해당하는 MuJoCo 기능이 mjSpec.attach.
# 같은 팔 파일을 prefix 만 바꿔 두 번 붙이면 left_/right_ 이름으로 body·joint·actuator 가 복제된다.
# 베이스는 4주차 전까지 치수 미정이라 box + 4 바퀴(시각용) 로 임시 표현, freejoint 로 월드에 떠 있게 둔다.
import os
import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))   # sim/
ARM = os.path.join(ROOT, "mjcf", "two_link_arm_v2.xml")
OUT = os.path.join(ROOT, "mjcf", "arm_on_base.xml")

# 1) 베이스 spec (임시 홀로노믹 베이스: 0.575 × 0.472 m 상판 치수는 노션 BASE 3.STEP 미리보기 기준, 높이는 가정)
base = mujoco.MjSpec()
base.modelname = "arm_on_base"
base.compiler.degree = False          # radian
base.option.timestep = 0.001
base.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST

floor = base.worldbody.add_geom(name="floor", type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.05],
                               rgba=[0.9, 0.9, 0.9, 1])
base.worldbody.add_light(pos=[0, 0, 3], dir=[0, 0, -1])

body = base.worldbody.add_body(name="base", pos=[0, 0, 0.35])
body.add_freejoint()                   # 베이스는 자유 6-DOF (고정하려면 이 줄을 지우면 됨)
body.add_geom(name="deck", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.2875, 0.236, 0.006],
              mass=3.7, rgba=[0.3, 0.3, 0.35, 1])                   # 상판: 알루미늄 기준 약 3.7 kg (노션 미리보기)
body.add_geom(name="frame", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.25, 0.2, 0.15], pos=[0, 0, -0.16],
              mass=20.0, rgba=[0.2, 0.2, 0.2, 1])                   # 프레임·배터리 임시 (질량은 실측 전 가정)
for i, (sx, sy) in enumerate([(1, 1), (1, -1), (-1, 1), (-1, -1)]):
    body.add_geom(name=f"wheel{i}", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.06, 0.02],
                  pos=[sx * 0.24, sy * 0.21, -0.29], quat=[0.7071, 0.7071, 0, 0],   # 축 Y
                  contype=0, conaffinity=0, mass=0.5, rgba=[0.1, 0.1, 0.1, 1])

# 임시 몸통(torso): 노션 메모대로 "몸통이 없으므로 임시 box, 어깨 장착 위치만 맞춤"
torso = body.add_body(name="torso", pos=[0, 0, 0.3])
torso.add_geom(name="torso_box", type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.1, 0.15, 0.3], mass=10.0,
               rgba=[0.5, 0.5, 0.6, 1])

# 2) 팔 spec 을 읽어 좌·우 어깨 위치에 attach (prefix 로 이름 충돌 방지)
for side, y in (("left", 0.3), ("right", -0.3)):
    arm = mujoco.MjSpec.from_file(ARM)
    arm_root = arm.body("base_link")                     # 팔 파일의 최상위 body (천장 고정판)
    arm_root.pos = [0, 0, 0]                             # 파일 안의 pos="0 0 1.0" 은 버리고 프레임 위치를 쓴다
    frame = torso.add_frame(pos=[0, y, 0.25])            # 어깨 장착점: 몸통 상단 좌/우 (상판 폭 0.472 m 바깥, ±0.3 m)
    frame.attach_body(arm_root, prefix=f"{side}_")       # left_base_link / left_shoulder / left_shoulder_pos ...

base.compile()
xml = base.to_xml()
with open(OUT, "w", encoding="utf-8") as f:
    f.write("<!-- week03 | attach_arm_to_base.py 가 생성. 직접 수정하지 말고 스크립트를 고칠 것 -->\n" + xml)

m = mujoco.MjModel.from_xml_path(OUT)
names = [m.joint(i).name or '(free)' for i in range(m.njnt)]
print(f"saved {OUT}: nbody={m.nbody} njnt={m.njnt} nu={m.nu} ngeom={m.ngeom}")
print("joints   :", names)
print("actuators:", [m.actuator(i).name for i in range(m.nu)])
print("total mass [kg]:", round(float(m.body_subtreemass[1]), 2))
