# week04 | 테스트 STEP 파이프라인 검산 — 손계산 · 직접 적분 · MuJoCo 질량행렬 3중 확인 (함태훈, B1)
# 실행 (저장소 루트에서, 먼저 make_test_step.py → step_to_links.py 실행):
#   python sim/scripts/verify_test_step.py
# 확인하는 것
#   (1) step_to_links.py 가 낸 질량 · COM · COM 기준 관성 = 손계산 (상자 · 원통 공식 + 평행축 정리)
#   (2) 평행축 정리 I_O = I_C + m(|d|²E − d dᵀ) 를, 메시를 링크 원점 기준으로 직접 적분한 값과 비교
#   (3) 생성한 inertial 로 URDF 를 만들어 MuJoCo 에 넣고, 팔꿈치 자유도의 질량행렬 원소 = 관절축 기준 관성인지 확인
import csv, json, os
import numpy as np
import trimesh, mujoco
from mj_load import load_model      # 한글 경로 대비 (같은 폴더의 mj_load.py)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))     # sim/
OUT = os.path.join(ROOT, "test_cad", "out")
rho = 2700.0


def box_I(m, a, b, c):                 # 변 a(x) b(y) c(z), COM 기준
    return np.diag([m * (b*b + c*c) / 12, m * (a*a + c*c) / 12, m * (a*a + b*b) / 12])


def cyl_I(m, r, h, axis):              # 축 방향 axis(0/1/2), COM 기준
    I = np.full(3, m * (3*r*r + h*h) / 12); I[axis] = m * r*r / 2
    return np.diag(I)


def shift(I_c, m, d):                  # 평행축 정리: COM 기준 → COM 에서 d 떨어진 점 기준
    d = np.asarray(d, float)
    return I_c + m * (d @ d * np.eye(3) - np.outer(d, d))


# ---- (1) 손계산
m_box = rho * 0.04 * 0.04 * 0.2;           c_box = np.array([0, 0, -0.1])
m_bolt = rho * np.pi * 0.005**2 * 0.04;    c_bolt = np.array([0.04, 0, -0.1])
m_up = m_box + m_bolt
c_up = (m_box * c_box + m_bolt * c_bolt) / m_up
I_up = shift(box_I(m_box, 0.04, 0.04, 0.2), m_box, c_box - c_up) + shift(cyl_I(m_bolt, 0.005, 0.04, 0), m_bolt, c_bolt - c_up)
m_fo = rho * np.pi * 0.02**2 * 0.18;       c_fo = np.array([0, 0, -0.09])      # 링크 원점 = 팔꿈치 (조립 z = -200 mm)
I_fo = cyl_I(m_fo, 0.02, 0.18, 2)
hand = {"upper_arm_link": (m_up, c_up, I_up), "forearm_link": (m_fo, c_fo, I_fo)}

rows = {r["link"]: r for r in csv.DictReader(open(os.path.join(OUT, "link_mass_props.csv"), encoding="utf-8-sig"))}
print("=== (1) step_to_links.py (OpenCascade) vs 손계산 ===")
for name, (m, c, I) in hand.items():
    r = rows[name]
    mo, co, Io = float(r["mass_occ"]), np.array(json.loads(r["com_occ"])), np.array(json.loads(r["I_occ"]))
    print(f"  {name:15s} 질량 {mo:.6f} / 손 {m:.6f} kg (오차 {abs(mo-m)/m*100:.2e} %) | COM 차 {np.linalg.norm(co-c)*1000:.2e} mm"
          f" | 관성 상대오차 {np.linalg.norm(Io-I)/np.linalg.norm(I):.2e} | 메시(trimesh) 관성오차 {float(r['I_rel_err'])*100:.3f} %")
    print(f"      손계산 Ixx Iyy Izz = {np.diag(I).round(8).tolist()} kg·m²")


# ---- (2) 평행축 정리 vs 링크 원점 기준 직접 적분 (사면체 분해, 원점 기준 2차 모멘트)
def inertia_about_origin(mesh, density):
    C = np.zeros((3, 3)); can = (np.ones((3, 3)) + np.eye(3)) / 120.0
    for tri in mesh.triangles:                    # 원점-삼각형 사면체 (부호 있는 부피)
        A = np.array(tri).T; det = np.linalg.det(A)
        C += det * A @ can @ A.T
    C *= density
    return np.trace(C) * np.eye(3) - C            # I = tr(C)E − C


print("\n=== (2) 평행축 정리 검산: 링크 원점(관절축) 기준 관성 ===")
for name, (m, c, I) in hand.items():
    mesh = trimesh.load(os.path.join(OUT, "meshes", f"{name}_visual.stl"))
    I_direct = inertia_about_origin(mesh, rho)
    I_pa = shift(I, m, -c)                         # COM 기준 → 링크 원점 기준
    print(f"  {name:15s} 평행축 I_O 대각 = {np.diag(I_pa).round(8).tolist()}")
    print(f"  {'':15s} 직접 적분 대각  = {np.diag(I_direct).round(8).tolist()}  (상대차 {np.linalg.norm(I_direct-I_pa)/np.linalg.norm(I_pa)*100:.3f} %)")
    print(f"  {'':15s} → 링크 원점 기준이 COM 기준보다 Ixx {np.diag(I_pa)[0]/np.diag(I)[0]:.1f} 배 (m·d² = {m*(c@c):.6f} kg·m² 추가)")

# ---- (3) URDF 로 만들어 MuJoCo 에 로드 → 질량행렬
blocks = open(os.path.join(OUT, "inertial_blocks.xml"), encoding="utf-8").read()
blk = {n: b for n, b in zip(("upper_arm_link", "forearm_link"), blocks.split("<!-- ")[2:])}
urdf = f'''<?xml version="1.0"?>
<robot name="test_arm">
  <mujoco><compiler meshdir="meshes" fusestatic="false" discardvisual="false"/></mujoco>
  <link name="base_link"/>
  <joint name="shoulder" type="revolute"><parent link="base_link"/><child link="upper_arm_link"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="5"/></joint>
  <link name="upper_arm_link">
{"  <!-- " + blk["upper_arm_link"]}    <visual><geometry><mesh filename="upper_arm_link_visual.stl"/></geometry></visual>
    <collision><geometry><mesh filename="upper_arm_link_collision.stl"/></geometry></collision>
  </link>
  <joint name="elbow" type="revolute"><parent link="upper_arm_link"/><child link="forearm_link"/>
    <origin xyz="0 0 -0.2"/><axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="5"/></joint>
  <link name="forearm_link">
{"  <!-- " + blk["forearm_link"]}    <visual><geometry><mesh filename="forearm_link_visual.stl"/></geometry></visual>
    <collision><geometry><mesh filename="forearm_link_collision.stl"/></geometry></collision>
  </link>
</robot>
'''
upath = os.path.join(OUT, "test_arm.urdf"); open(upath, "w", encoding="utf-8").write(urdf)
mdl = load_model(upath); d = mujoco.MjData(mdl); mujoco.mj_forward(mdl, d)
M = np.zeros((mdl.nv, mdl.nv))
try:
    mujoco.mj_fullM(mdl, d, M)          # mujoco 3.15: (model, data, dst)
except TypeError:
    mujoco.mj_fullM(mdl, M, d.qM)       # mujoco 3.14 (노트북): (model, dst, qM)
b = mdl.body("forearm_link").id
print("\n=== (3) MuJoCo 로드 (mujoco", mujoco.__version__, ") ===")
print(f"  body {mdl.nbody}, joint {mdl.njnt}, 총질량 {mdl.body_subtreemass[1]:.6f} kg (손 {m_up+m_fo:.6f})")
print(f"  forearm body_ipos {mdl.body_ipos[b].round(6).tolist()}, 주관성 {np.sort(mdl.body_inertia[b]).round(8).tolist()}")
I_elbow = shift(I_fo, m_fo, -c_fo)[1, 1]
print(f"  질량행렬 M[elbow,elbow] = {M[1,1]:.8f}  vs  손계산 Iyy_C + m·d² = {I_fo[1,1]:.8f} + {m_fo:.6f}·0.09² = {I_elbow:.8f}"
      f"  (상대차 {abs(M[1,1]-I_elbow)/I_elbow*100:.3f} %)")
I_sh = shift(I_up, m_up, -c_up)[1, 1] + shift(I_fo, m_fo, -(c_fo + np.array([0, 0, -0.2])))[1, 1]
print(f"  질량행렬 M[shoulder,shoulder] = {M[0,0]:.8f}  vs  손계산 (팔 전체, 어깨축 기준) {I_sh:.8f}  (상대차 {abs(M[0,0]-I_sh)/I_sh*100:.3f} %)")
print("  저장:", upath)

# ---- (4) 흔한 실수: 링크 원점 기준 관성을 URDF <inertia> 에 그대로 넣으면?
I_O = shift(I_fo, m_fo, -c_fo)
wrong = urdf.replace(blk["forearm_link"].split("<inertia ")[1].split("/>")[0],
                     f'ixx="{I_O[0,0]:.8g}" iyy="{I_O[1,1]:.8g}" izz="{I_O[2,2]:.8g}" ixy="0" ixz="0" iyz="0"')
open(os.path.join(OUT, "test_arm_wrong_inertia.urdf"), "w", encoding="utf-8").write(wrong)
m2 = load_model(os.path.join(OUT, "test_arm_wrong_inertia.urdf")); d2 = mujoco.MjData(m2); mujoco.mj_forward(m2, d2)
M2 = np.zeros((m2.nv, m2.nv))
try:
    mujoco.mj_fullM(m2, d2, M2)
except TypeError:
    mujoco.mj_fullM(m2, M2, d2.qM)
print(f"\n=== (4) 원점 기준 관성을 그대로 넣은 경우 ===\n  M[elbow,elbow] = {M2[1,1]:.8f} (올바른 값 {M[1,1]:.8f}의 {M2[1,1]/M[1,1]:.2f} 배)"
      f" = I_O + m·d² = {I_O[1,1]:.8f} + {m_fo*0.09**2:.8f}")
