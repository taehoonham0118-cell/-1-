# week04 | 우리 로봇 v0 URDF 조립: 베이스(planar, 임시 box) + 몸통(임시 box + 어깨 RI85 고정부) + 양팔 4링크 (함태훈, B1)
# 입력 (step_to_links.py · find_joint_axes.py 결과): sim/robot/link_mass_props.csv, joint_axes_cad.csv, linkmap_arm.json, meshes/
# 출력: sim/robot/our_robot_v0.urdf  (+ 가정값은 아래 ASSUME 표 — 확정 시 교체)
# 팀장 회신(10/14) 규칙 반영:
#   ① base_link = REP 103, CAD → 로봇 고정 변환은 하나(cad_frame, torso_link 아래 고정 프레임)로만 표현. CAD 점 p[mm] = cad_frame 기준 p/1000
#   ② URDF 는 공칭값: 축은 단위축(직교), 오른팔 joint origin 은 왼팔의 거울(y 부호 반대). CAD 편차(1.37° · 7 mm)는 joint 원점 검증표에 기록
#   ③ 영점 = 팔 내린 자세 + 팔꿈치 전방 굽힘. CAD 자세와의 90° 차이는 shoulder_yaw origin rpy (0, 0, ∓π/2)(left −, right +)로 흡수
#      → CAD 자세 = 양팔 shoulder_yaw q = +π/2. 메시 · 관성은 링크 프레임 기준이라 그대로
#   ④ 축 부호: 왼팔 pitch +y · roll +x · yaw +z · elbow +x, 오른팔은 거울(roll −x · yaw −z · elbow −x) — 박종진 ICD v0 3.4 와 같음
# 실행: python sim/scripts/build_v0_urdf.py
import csv, json, math, os
import numpy as np

ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robot")
# ---------------------------------------------------------------- 가정값 표 (근거 없음 = 확정 시 교체)
ASSUME = dict(
    plate_size=(0.575, 0.472, 0.012),   # 상판 BASE 3.STEP 바운딩 박스 (10/14 STEP 확인) — 형상 · 질량은 base_plate 메시(sim/robot/base/)
    plate_top_z=0.30,                    # 바닥 → 상판 윗면 [m]  가정 (하체 실측 보류)
    base_mass=30.0,                      # 하체 전체 [kg] 가정 (상판 CAD 3.69 kg 포함, 나머지 휠 · 프레임 · 배터리는 실측 보류)
    base_clear=0.05,                     # base 충돌 box 아랫면 높이 [m] — planar 라 바닥과 닿지 않게
    shoulder_z=0.80,                     # 바닥 → 어깨 pitch 축 [m] 가정 (몸통 CAD 없음)
    torso_box=(0.125, 0.150, 0.55),      # 몸통 box [m]: 앞뒤 = RI85 지름, 좌우 = 두 어깨 RI85 사이 150 mm(CAD), 높이 가정
    torso_box_top=0.05,                  # 어깨 축 위로 몸통 box 가 나온 길이 [m] 가정
    torso_box_mass=5.0,                  # 몸통 box 질량 [kg] 가정
    limit=1.57, effort=50.0, velocity=3.0)  # 관절 한계 · effort[Nm] · velocity[rad/s] 가정 — 박종진 구동기 사양표 / RI85 ICD 수령 시 교체
SIDES = ("left", "right")
CHAIN = [("shoulder_pitch", "torso_link", "J1"), ("shoulder_roll", "shoulder_pitch", "J2"),
         ("shoulder_yaw", "shoulder_roll", "J3"), ("elbow", "shoulder_yaw", "J4")]
ACT = {"left": [1, 2, 3, 4], "right": [6, 5, 7, 8]}                  # 관절 J1~J4 ↔ RI85 제작건 번호
R = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], float)             # 열 = 로봇 x·y·z 축의 CAD 방향 (linkmap rpy 와 같음)


def box_I(m, s):
    a, b, c = s
    return np.diag([m * (b*b + c*c), m * (a*a + c*c), m * (a*a + b*b)]) / 12


def combine(items):
    """[(m, c, I_c)] → 합친 질량 · COM · COM 기준 관성 (평행축 정리)."""
    M = sum(m for m, _, _ in items); C = sum(m * np.asarray(c) for m, c, _ in items) / M
    I = sum(I + m * ((np.asarray(c) - C) @ (np.asarray(c) - C) * np.eye(3) - np.outer(np.asarray(c) - C, np.asarray(c) - C))
            for m, c, I in items)
    return M, C, I


f = lambda v: "0" if abs(v) < 1e-12 else f"{v:.6g}"
vec = lambda v: " ".join(f(x) for x in v)


def inertial(m, c, I):
    return (f'    <inertial><origin xyz="{vec(c)}" rpy="0 0 0"/><mass value="{f(m)}"/>\n'
            f'      <inertia ixx="{f(I[0,0])}" iyy="{f(I[1,1])}" izz="{f(I[2,2])}" ixy="{f(I[0,1])}" ixz="{f(I[0,2])}" iyz="{f(I[1,2])}"/></inertial>\n')


def mesh_geoms(name):
    return (f'    <visual><geometry><mesh filename="{name}_visual.stl"/></geometry></visual>\n'
            f'    <collision><geometry><mesh filename="{name}_collision.stl"/></geometry></collision>\n')


props = {r["link"]: r for r in csv.DictReader(open(os.path.join(ROOT, "link_mass_props.csv"), encoding="utf-8-sig"))}
axes = {int(r["actuator"].split(":")[1]): r for r in csv.DictReader(open(os.path.join(ROOT, "joint_axes_cad.csv"), encoding="utf-8-sig"))}
lm = {L["name"]: L for L in json.load(open(os.path.join(ROOT, "linkmap_arm.json"), encoding="utf-8"))["links"]}
cad = lambda name: (float(props[name]["mass_occ"]), np.array(json.loads(props[name]["com_occ"])), np.array(json.loads(props[name]["I_occ"])))
A = ASSUME
out = ['<?xml version="1.0"?>\n<!-- our_robot_v0: build_v0_urdf.py 생성 (week04, 함태훈). 형상 · 질량은 「양팔로봇 최종버전.stp」, 베이스 · 몸통 · 한계는 가정값 -->\n',
       '<robot name="our_robot_v0">\n',
       '  <mujoco><compiler meshdir="meshes" balanceinertia="false" discardvisual="false" fusestatic="false"/></mujoco>\n',
       '  <link name="world"/>\n',
       '  <!-- 팀장 회신(10/8): 베이스 v0 = planar(slide x · slide y · hinge z). 10/13 카톡 「확인 후 연락」 → 확정 대기 -->\n',
       '  <joint name="base_joint" type="planar"><parent link="world"/><child link="base_link"/><origin xyz="0 0 0"/><axis xyz="0 0 1"/>\n'
       '    <limit lower="-10" upper="10" effort="500" velocity="1.5"/></joint>\n']
# base: 원점 = 바닥 중심 (ICD 3.1)
bh = A["plate_top_z"] - A["plate_size"][2] - A["base_clear"]
bc = [0, 0, A["base_clear"] + bh / 2]
plate = {r["link"]: r for r in csv.DictReader(open(os.path.join(ROOT, "base", "link_mass_props.csv"), encoding="utf-8-sig"))}["base_plate"]
pm, pc, pI = float(plate["mass_occ"]), np.array(json.loads(plate["com_occ"])), np.array(json.loads(plate["I_occ"]))
rest = A["base_mass"] - pm                                       # 상판 외 하체(가정) = box
bm, bcom, bI = combine([(pm, pc, pI), (rest, bc, box_I(rest, (A["plate_size"][0], A["plate_size"][1], bh)))])
out += ['  <link name="base_link">\n', inertial(bm, bcom, bI),
        '    <visual><geometry><mesh filename="base_plate_visual.stl"/></geometry></visual>\n',
        f'    <visual><origin xyz="{vec(bc)}"/><geometry><box size="{f(A["plate_size"][0]*0.9)} {f(A["plate_size"][1]*0.9)} {f(bh)}"/></geometry></visual>\n',
        f'    <collision><origin xyz="0 0 {f((A["base_clear"] + A["plate_top_z"])/2)}"/><geometry><box size="{f(A["plate_size"][0])} {f(A["plate_size"][1])} {f(A["plate_top_z"] - A["base_clear"])}"/></geometry></collision>\n',
        '  </link>\n']
# torso: CAD 어깨 RI85 고정부 + 임시 box
tb = A["torso_box"]; tc = np.array([0, 0, A["torso_box_top"] - tb[2] / 2])
m, c, I = combine([cad("torso_link"), (A["torso_box_mass"], tc, box_I(A["torso_box_mass"], tb))])
out += [f'  <joint name="torso_joint" type="fixed"><parent link="base_link"/><child link="torso_link"/><origin xyz="0 0 {f(A["shoulder_z"])}"/></joint>\n',
        '  <link name="torso_link">\n', inertial(m, c, I), mesh_geoms("torso_link"),
        f'    <visual><origin xyz="{vec(tc)}"/><geometry><box size="{vec(tb)}"/></geometry></visual>\n',
        f'    <collision><origin xyz="{vec(tc - [0, 0, 0.0])}"/><geometry><box size="{f(tb[0])} {f(tb[1] - 0.002)} {f(tb[2])}"/></geometry></collision>\n',
        '  </link>\n']
# ---- cad_frame: CAD 좌표계를 몸통 아래 고정 프레임 하나로 (CAD 원점 · 축 그대로, m 단위)
mid_mm = np.array(lm["torso_link"]["origin_mm"])
cad_xyz = R.T @ (-mid_mm) / 1000                              # 몸통 원점 → CAD 원점 (로봇 축)
out += [f'  <joint name="cad_frame_joint" type="fixed"><parent link="torso_link"/><child link="cad_frame"/>\n'
        f'    <origin xyz="{vec(np.round(cad_xyz, 5))}" rpy="1.5708 0 1.5708"/></joint>\n',
        '  <link name="cad_frame"/>   <!-- CAD 점 p[mm] → 이 프레임 기준 p/1000. 정면(+z_CAD) 확정 시 이 변환만 바꿈 -->\n']
AXIS = {"shoulder_pitch": (0, 1, 0), "shoulder_roll": (1, 0, 0), "shoulder_yaw": (0, 0, 1), "elbow": (1, 0, 0)}
MIRROR = {"shoulder_pitch": 1, "shoulder_roll": -1, "shoulder_yaw": -1, "elbow": -1}     # 오른팔 축 부호
YAW_RPY = {"left": -np.pi / 2, "right": np.pi / 2}
# 공칭 joint origin: 왼팔 CAD 값에서 1e-3 m 미만 성분(축 기울기 · 조립 오차로 생긴 것)을 0 으로, 오른팔은 y 거울
nominal = {}
for (part, parent, jk), act in zip(CHAIN, ACT["left"]):
    child = f"left_{part}_link"; par = parent if parent == "torso_link" else f"left_{parent}_link"
    xyz = R.T @ (np.array(lm[child]["origin_mm"]) - np.array(lm[par]["origin_mm"])) / 1000
    nominal[part] = np.where(np.abs(xyz) < 1e-3, 0.0, np.round(xyz, 5))
rows = []
for side in SIDES:
    for (part, parent, jk), act in zip(CHAIN, ACT[side]):
        child = f"{side}_{part}_link"
        par = parent if parent == "torso_link" else f"{side}_{parent}_link"
        xyz = nominal[part] * (np.array([1, -1, 1]) if side == "right" else 1)
        ax = np.array(AXIS[part], float) * (MIRROR[part] if side == "right" else 1)
        rpy = (0, 0, YAW_RPY[side]) if part == "shoulder_yaw" else (0, 0, 0)
        m, c, I = cad(child)
        out += [f'  <joint name="{side}_{part}_joint" type="revolute"><parent link="{par}"/><child link="{child}"/>\n'
                f'    <origin xyz="{vec(xyz)}" rpy="{vec(np.round(rpy, 4))}"/><axis xyz="{vec(ax)}"/>\n'
                f'    <limit lower="{-A["limit"]}" upper="{A["limit"]}" effort="{A["effort"]}" velocity="{A["velocity"]}"/></joint>\n',
                f'  <link name="{child}">\n', inertial(m, c, I), mesh_geoms(child), '  </link>\n']
        rows.append((f"{side}_{part}_joint", act, xyz, ax))
out.append('</robot>\n')
p = os.path.join(ROOT, "our_robot_v0.urdf"); open(p, "w", encoding="utf-8").write("".join(out))
print("저장:", p)
for r in rows:
    print(f"  {r[0]:28s} RI85:{r[1]}  origin {np.round(r[2], 5).tolist()}  axis {np.round(r[3], 4).tolist()}")
