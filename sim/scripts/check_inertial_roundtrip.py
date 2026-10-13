# week04 | URDF <inertial>(COM 기준 전체 텐서) ↔ MJCF(pos · quat · diaginertia) 왕복 검산 (함태훈, B1)
# G1 공식 값(MJCF 식)을 URDF 식 전체 텐서로 바꾼 g1_left_arm_inertial_blocks.xml 을 다시 MuJoCo 에 넣어,
# MuJoCo 가 고유값 분해로 되돌린 주관성 · 주축이 원래 g1.xml 값과 같은지 본다.
# 실행: python sim/scripts/check_inertial_roundtrip.py --g1 mujoco_menagerie/unitree_g1
import argparse, os, re, xml.etree.ElementTree as ET
import numpy as np
import mujoco

ap = argparse.ArgumentParser(); ap.add_argument("--g1", default="mujoco_menagerie/unitree_g1"); a = ap.parse_args()
here = os.path.dirname(os.path.abspath(__file__))
blocks = open(os.path.join(os.path.dirname(here), "urdf", "g1_left_arm_inertial_blocks.xml"), encoding="utf-8").read()
g1 = ET.parse(os.path.join(a.g1, "g1.xml")).getroot()


def mat_from(quat, diag):
    m = np.zeros(9); mujoco.mju_quat2Mat(m, np.asarray(quat, float)); R = m.reshape(3, 3)
    return R @ np.diag(diag) @ R.T


worst = 0.0
for name, body in re.findall(r"<!-- (\S+): .*?-->\n(  <inertial>.*?</inertial>)", blocks, re.S):
    urdf = f'<robot name="t"><link name="world_dummy"/><joint name="j" type="floating"><parent link="world_dummy"/><child link="{name}"/></joint><link name="{name}">{body}</link></robot>'
    m = mujoco.MjModel.from_xml_string(urdf); b = m.body(name).id
    I_back = mat_from(m.body_iquat[b], m.body_inertia[b])                     # MuJoCo 가 저장한 값 → 전체 텐서
    o = g1.find(f".//body[@name='{name}']/inertial")
    I_orig = mat_from(o.get("quat").split(), np.array(o.get("diaginertia").split(), float))
    err = np.linalg.norm(I_back - I_orig) / np.linalg.norm(I_orig); worst = max(worst, err)
    print(f"{name:26s} 질량 {m.body_mass[b]:.6f} / {float(o.get('mass')):.6f}  COM 차 {np.linalg.norm(m.body_ipos[b]-np.array(o.get('pos').split(), float))*1000:.1e} mm"
          f"  관성 상대오차 {err:.1e}")
print(f"최대 상대오차 {worst:.1e} (출력 유효숫자 6자리 반올림 수준이면 정상)")
