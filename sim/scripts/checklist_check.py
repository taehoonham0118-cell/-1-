# week03 | 박종진 「우리 로봇 URDF → MuJoCo 로딩 체크리스트」 v0.1 로 내 파일 점검 (함태훈, B1)
# 실행: python sim/scripts/checklist_check.py          (저장소 루트에서)
# 대상: sim/urdf/two_link_arm.urdf (A·B·D·F절), sim/mjcf/two_link_arm_v2.xml + scene.xml (D·E·E-1·F절)
# 출력: 항목 번호별 OK / NG / N-A 와 근거 수치 → notes/07 §6 에 표로 옮김
import os, re
import numpy as np
import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
URDF = os.path.join(ROOT, "urdf", "two_link_arm.urdf")
V2 = os.path.join(ROOT, "mjcf", "two_link_arm_v2.xml")
SCENE = os.path.join(ROOT, "mjcf", "scene.xml")
CONV = os.path.join(ROOT, "mjcf", "two_link_arm_from_urdf.xml")

urdf_txt = open(URDF, encoding="utf-8").read()
v2_txt = open(V2, encoding="utf-8").read()
mu = mujoco.MjModel.from_xml_path(URDF)      # URDF 로드 (확장 반영)
mv = mujoco.MjModel.from_xml_path(V2)        # 손 작성 MJCF
ms = mujoco.MjModel.from_xml_path(SCENE)     # include 된 scene
rows = []


def R(no, ok, evidence):
    rows.append((no, {True: "OK", False: "NG", None: "N-A"}[ok], evidence))


# ---------- A. URDF 자체 ----------
links = re.findall(r'<link name="([^"]+)">(.*?)</link>', urdf_txt, re.S)
moving = [n for n, body in links if n != "base_link"]
R("A1", all("<inertial>" in b for n, b in links if n in moving), f"inertial 있는 링크 = {[n for n, b in links if '<inertial>' in b]}")
R("A2", None, "가상(질량 0) 링크 없음")
tri = []
for n, b in links:
    m_ = re.search(r'ixx="([^"]+)" iyy="([^"]+)" izz="([^"]+)"', b)
    if m_:
        a, bb, c = map(float, m_.groups()); tri.append((n, a + bb >= c and a + c >= bb and bb + c >= a))
R("A3", all(t for _, t in tri), f"삼각부등식 {tri}")
R("A4", "<mesh" not in urdf_txt, "메시 없음(원시 도형), 단위 m·kg·rad")
joints = re.findall(r'<joint name="([^"]+)" type="([^"]+)">(.*?)</joint>', urdf_txt, re.S)
lim_ok = all(all(k in j for k in ("lower", "upper", "effort", "velocity")) for _, t, j in joints if t == "revolute")
R("A5", lim_ok, f"revolute {[(n, t) for n, t, _ in joints]} 모두 lower·upper·effort·velocity 있음; 변환 후 effort→actuatorfrcrange={mu.jnt_actfrcrange[0].tolist()}, velocity 소실")
R("A6", mu.nbody == len(links) + 1, f"트리: URDF link {len(links)} → body {mu.nbody}(world 포함), 폐루프 없음")
R("A7", "<mujoco>" in urdf_txt and mu.nbody == 4 and mu.ngeom == 5,
  f"<mujoco><compiler fusestatic=false discardvisual=false/> 추가(10/7) → 로드 결과 body {mu.nbody}, geom {mu.ngeom} 으로 반영 확인(오타 없음)")
R("A8", "<transmission>" not in urdf_txt and mu.nu == 0, f"transmission 미사용, 변환 nu={mu.nu} → 액추에이터는 MJCF(E절)")
R("A9", True, f"dynamics damping=0.1 friction=0 → joint damping={mu.dof_damping.tolist()}, frictionloss={mu.dof_frictionloss.tolist()}(0이라 변환본에서 생략)")

# ---------- B. compiler 확장 ----------
R("B1", "base_link" in [mu.body(i).name for i in range(mu.nbody)], f"fusestatic=false → body {[mu.body(i).name for i in range(mu.nbody)]}, base_link 5 kg 유지 (총질량 {mu.body_subtreemass[1]:.1f} kg)")
R("B2", mu.ngeom == 5, f"discardvisual=false → geom {mu.ngeom} (visual 3 + collision 2), visual 은 contype=conaffinity=0")
R("B3", 'angle="radian"' in v2_txt, "손 작성 MJCF에 compiler angle=radian 명시 (URDF 는 자동 radian)")
R("B4", None, "메시 없음 → meshdir 불필요 (4주차 STL 때 적용)")
R("B5", True, "inertiafromgeom auto, A1 충족이라 영향 없음")
R("B6", True, "balanceinertia=false, A3 충족")
R("B7", True, f"autolimits: URDF 변환본 jnt_limited={mu.jnt_limited.tolist()}, v2 autolimits=true")

# ---------- C. 베이스 ----------
R("C1", mu.nq == 2, f"팔 단독 모델 = 고정 베이스 (nq={mu.nq}); 베이스 결합은 arm_on_base.xml 에서 freejoint(C2). 평면 3-DOF(C3)는 팀장 결정 후")

# ---------- D. 자기 충돌 ----------
def contacts_after_fall(m, q0):
    d = mujoco.MjData(m); d.qpos[: len(q0)] = q0
    if m.nu: m.actuator_gainprm[:, 0] = 0; m.actuator_biasprm[:] = 0
    mx = 0
    for _ in range(2000):
        mujoco.mj_step(m, d); mx = max(mx, d.ncon)
    return mx
cu = contacts_after_fall(mujoco.MjModel.from_xml_path(URDF), [1.5708, 0])
cv = contacts_after_fall(mujoco.MjModel.from_xml_path(V2), [1.5708, 0])
R("D1", cu == 0 and cv == 0, f"world 용접 base_link ↔ upper_arm: 베이스에 collision geom 없음 → 수평 낙하 2 s 동안 최대 접촉 수 URDF {cu}, v2 {cv}")
R("D2", None, "D1 이 0이라 exclude 불필요. 베이스 collision 을 쓰게 되면 <contact><exclude base_link upper_arm/> (fusestatic=false 라 이름 참조 가능)")
R("D3", None, "팔 1개 모델 — 비부모자식 쌍 없음. arm_on_base.xml(팔 2 + 몸통)에서 적용 예정")
vis = [i for i in range(mv.ngeom) if mv.geom_group[i] == 2]
R("D4", all(mv.geom_contype[i] == 0 and mv.geom_conaffinity[i] == 0 for i in vis), f"v2 class=visual geom {len(vis)}개 contype=conaffinity=0, group 2")

# ---------- E. 액추에이터 ----------
R("E1", mv.nu == 2 and ms.nu == 2, "v2: actuator 를 팔 파일 안 <actuator> + default class 로 두고 scene.xml 은 include 만 (menagerie 방식). 체크리스트 E1 의 'scene 쪽 추가' 와 위치가 다름 → 협의 항목")
R("E2", all(mv.actuator_biasprm[i, 1] == -mv.actuator_gainprm[i, 0] for i in range(mv.nu)), f"position: gainprm kp={mv.actuator_gainprm[:,0].tolist()}, biasprm={mv.actuator_biasprm[:,:3].tolist()} (f = kp(ctrl−q) − kv·q̇)")
R("E3", None, "베이스 평면 관절 없음 (C3 결정 후 velocity 액추에이터)")
R("E4", np.allclose(mv.actuator_forcerange, [[-25, 25]] * 2) and np.allclose(mv.actuator_ctrlrange, mv.jnt_range) and np.allclose(mv.jnt_actfrcrange, [[-25, 25]] * 2),
  f"forcerange={mv.actuator_forcerange.tolist()} = URDF effort, ctrlrange=inheritrange→{mv.actuator_ctrlrange.tolist()} = jnt_range, joint actuatorfrcrange={mv.jnt_actfrcrange.tolist()} (10/7 추가, 변환본과 일치)")
R("E5", mv.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST, f"kv 사용 + integrator=implicitfast ({mv.opt.integrator}); 실험 kv=14(≈kv_c 13.9) 오버슈트 0 %")
R("E6", False, f"armature={mv.dof_armature.tolist()} — RI85 로터 관성·감속비(BOM) 미확보라 0. 실험으로 0.01/0.1 영향만 기록 → BOM 수령 후 입력 (보류)")
R("E7", True, "정상상태 오차 e=τg/kp: 실험 0.0137 rad vs 이론 2.75/200=0.0138 rad (kp=200)")

# ---------- E-1. 손 작성 MJCF ----------
cyl = [i for i in range(mv.ngeom) if mv.geom_type[i] == mujoco.mjtGeom.mjGEOM_CYLINDER]
R("H1", all(abs(mv.geom_size[i, 1] - 0.15) < 1e-9 for i in cyl), f"cylinder size=(0.02, 0.15) = 반지름·반길이 (URDF length 0.3 의 절반), {len(cyl)}개")
R("H2", 'angle="radian"' in v2_txt, "compiler angle=radian 명시")
# H3: 변환본 vs v2 자동 비교 (질량·관성·중력토크)
mc = mujoco.MjModel.from_xml_path(CONV)
def grav_torque(m):
    d = mujoco.MjData(m); d.qpos[:] = [1.5708, 0.0]; mujoco.mj_forward(m, d); return d.qfrc_bias[:2].copy()
def arm_bodies(m): return {m.body(i).name: (float(m.body_mass[i]), m.body_inertia[i].round(6).tolist()) for i in range(m.nbody) if m.body(i).name in ("upper_arm", "forearm")}
gc, gv = grav_torque(mc), grav_torque(mv)
same = arm_bodies(mc) == arm_bodies(mv) and np.allclose(gc, gv, atol=1e-9) and np.allclose(mc.jnt_range, mv.jnt_range) and np.allclose(mc.dof_damping, mv.dof_damping)
R("H3", same, f"변환본 vs v2: 질량·관성 {arm_bodies(mv)}, 수평 중력토크 변환본 {gc.round(4).tolist()} / v2 {gv.round(4).tolist()} N·m (손계산 어깨 0.6·9.81=5.886, 팔꿈치 0.15·9.81=1.4715), range·damping 일치")

# ---------- F. 로드 후 ----------
names_u = [mu.body(i).name for i in range(1, mu.nbody)]
R("F1", names_u == ["base_link", "upper_arm", "forearm"] and [mu.joint(i).name for i in range(mu.njnt)] == ["shoulder", "elbow"], f"body {names_u}, joint {[mu.joint(i).name for i in range(mu.njnt)]} = URDF")
R("F2", abs(mu.body_subtreemass[1] - 7.0) < 1e-9, f"총질량 {mu.body_subtreemass[1]:.3f} kg = 5+1+1")
R("F3", np.allclose(gc, [5.886, 1.4715], atol=1e-3), f"수평 자세 중력토크 {gc.round(4).tolist()} N·m = 손계산 (5.886, 1.4715)")
R("F4", "fusestatic" in urdf_txt and mu.nbody == 4, "확장 속성이 로드 결과(body 4·geom 5)에 반영 → mj_saveLastXML 결과 two_link_arm_from_urdf.xml 갱신")
R("F5", True, "뷰어: week02 캡처(영점 정지·관절 이동) + week03 오프스크린 렌더(scene_v2, arm_on_base), 자유 낙하 궤적 v1=v2 차이 0")

print(f"{'No.':5s} {'결과':4s} 근거")
for no, res, ev in rows:
    print(f"{no:5s} {res:4s} {ev}")
summary = {k: sum(1 for _, r, _ in rows if r == k) for k in ("OK", "NG", "N-A")}
print("\n합계:", summary)
