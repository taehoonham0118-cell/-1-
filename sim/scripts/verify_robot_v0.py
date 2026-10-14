# week04 | 우리 로봇 v0 URDF → MuJoCo 로드 · 3단 파일 · 점검 · 캡처 (함태훈, B1)
# 실행: python sim/scripts/verify_robot_v0.py            (뷰어로 보기: python -m mujoco.viewer --mjcf=sim/robot/scene.xml)
# 하는 일
#   1) our_robot_v0.urdf 로드 → mj_saveLastXML 로 robot.xml (3단 구조 1단, 수정 금지)
#   2) robot_actuated.xml (팔 position 액추에이터, kp · kv 가정) · scene.xml (바닥 · 조명) 생성
#   3) 점검: 질량 합 · 관성 삼각부등식 · 초기 자세 접촉 · 관절 앵커 = CAD 관절점(월드 변환) · 중력 토크 · 2 s 유지 · 캡처
import csv, json, os, sys
import numpy as np
import mujoco
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mj_load import load_model

ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robot")
KP, KV = 200.0, 10.0          # 가정 — 3주차 계단 응답 실험 범위, RI85 사양 수령 후 재조정
SHOULDER_Z = 0.80             # build_v0_urdf.py ASSUME 와 같아야 함
R = np.array([[0, 1, 0], [0, 0, 1], [1, 0, 0]], float)

m = load_model(os.path.join(ROOT, "our_robot_v0.urdf"))
mujoco.mj_saveLastXML(os.path.join(ROOT, "robot.xml"), m)
arm = [m.joint(i).name for i in range(m.njnt) if m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE and "shoulder" in m.joint(i).name or "elbow" in m.joint(i).name]
act = "\n".join(f'    <position name="{j}" joint="{j}" kp="{KP}" kv="{KV}" inheritrange="1"/>' for j in arm)
open(os.path.join(ROOT, "robot_actuated.xml"), "w", encoding="utf-8").write(
    f'<!-- 3단 구조 2단: robot.xml(수정 금지) + 액추에이터. kp {KP} · kv {KV} 는 가정값 -->\n<mujoco model="our_robot_v0_actuated">\n'
    f'  <include file="robot.xml"/>\n  <actuator>\n{act}\n  </actuator>\n</mujoco>\n')
open(os.path.join(ROOT, "scene.xml"), "w", encoding="utf-8").write(
    '<!-- 3단 구조 3단: 장면(바닥 · 조명 · 카메라) -->\n<mujoco model="our_robot_v0_scene">\n  <include file="robot_actuated.xml"/>\n'
    '  <visual><headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/><global azimuth="150" elevation="-15" offwidth="1280" offheight="960"/></visual>\n'
    '  <asset><texture name="grid" type="2d" builtin="checker" rgb1=".85 .85 .85" rgb2=".7 .7 .7" width="300" height="300"/>\n'
    '    <material name="grid" texture="grid" texrepeat="8 8"/></asset>\n'
    '  <worldbody><light pos="1 -1 3" dir="-0.3 0.3 -1"/><geom name="floor" type="plane" size="3 3 0.05" material="grid" group="2"/>\n'
    '    <camera name="front" pos="2.0 0 0.9" xyaxes="0 1 0 -0.25 0 1"/>\n'
    '    <camera name="iso" pos="1.5 1.4 1.3" xyaxes="-0.68 0.73 0 -0.3 -0.28 0.9"/></worldbody>\n</mujoco>\n')


def load_scene():
    assets = {}
    for fn in ("robot.xml", "robot_actuated.xml"):
        assets[fn] = open(os.path.join(ROOT, fn), "rb").read()
    for fn in os.listdir(os.path.join(ROOT, "meshes")):
        assets[fn] = open(os.path.join(ROOT, "meshes", fn), "rb").read()
    xml = open(os.path.join(ROOT, "scene.xml"), encoding="utf-8").read()
    return mujoco.MjModel.from_xml_string(xml, assets)


s = load_scene(); d = mujoco.MjData(s); mujoco.mj_forward(s, d)
print(f"mujoco {mujoco.__version__} | body {s.nbody} · joint {s.njnt} · dof {s.nv} · actuator {s.nu} · geom {s.ngeom} · mesh {s.nmesh}")
print("관절:", [s.joint(i).name for i in range(s.njnt)])
props = {r["link"]: r for r in csv.DictReader(open(os.path.join(ROOT, "link_mass_props.csv"), encoding="utf-8-sig"))}
cad_mass = sum(float(r["mass_occ"]) for r in props.values())
print(f"총질량 {s.body_subtreemass[0]:.3f} kg  (CAD 링크 합 {cad_mass:.3f} + 가정 베이스 30 + 몸통 box 5)")
arm_m = {side: sum(float(props[f"{side}_{p}_link"]["mass_occ"]) for p in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow")) for side in ("left", "right")}
print(f"팔 1개 질량 (CAD, 밀도 가정): 왼 {arm_m['left']:.3f} kg · 오른 {arm_m['right']:.3f} kg")
bad = [s.body(b).name for b in range(1, s.nbody) if s.body_mass[b] > 0 and
       not (lambda w: w[0] + w[1] >= w[2] * (1 - 1e-9))(np.sort(s.body_inertia[b]))]
print("관성 삼각부등식 위반 body:", bad or "없음 (balanceinertia=false 로 컴파일 통과)")
pairs = [(s.geom(c.geom1).name or s.body(s.geom_bodyid[c.geom1]).name, s.geom(c.geom2).name or s.body(s.geom_bodyid[c.geom2]).name, round(c.dist * 1000, 1))
         for c in d.contact[:d.ncon]]
print(f"초기 자세 접촉 {d.ncon}개:", pairs[:12])

# 관절 앵커 · 축: 공칭 URDF 를 「CAD 자세」(양팔 shoulder_yaw q = +π/2)로 두고 CAD 관절점 · 축과 비교 = 편차 기록 (팀장 10/14: URDF 공칭, 편차는 검증표)
lm = {L["name"]: L for L in json.load(open(os.path.join(ROOT, "linkmap_arm.json"), encoding="utf-8"))["links"]}
axes_cad = {int(r["actuator"].split(":")[1]): np.array([float(r[k]) for k in ("axis_x", "axis_y", "axis_z")])
            for r in csv.DictReader(open(os.path.join(ROOT, "joint_axes_cad.csv"), encoding="utf-8-sig"))}
ACT = {"left": [1, 2, 3, 4], "right": [6, 5, 7, 8]}
mid = np.array(lm["torso_link"]["origin_mm"])
dc = mujoco.MjData(s)
for side in ("left", "right"):
    dc.qpos[s.joint(f"{side}_shoulder_yaw_joint").qposadr[0]] = np.pi / 2
mujoco.mj_forward(s, dc)
rows = []
for side in ("left", "right"):
    for part, act in zip(("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"), ACT[side]):
        j = s.joint(f"{side}_{part}_joint")
        cad_w = np.array([0, 0, SHOULDER_Z]) + R.T @ (np.array(lm[f"{side}_{part}_link"]["origin_mm"]) - mid) / 1000
        a_cad = R.T @ axes_cad[act]; a_cad /= np.linalg.norm(a_cad)
        v = dc.xanchor[j.id] - cad_w; ax = dc.xaxis[j.id]
        perp = np.linalg.norm(v - (v @ a_cad) * a_cad) * 1000
        ang = np.degrees(np.arccos(min(1.0, abs(ax @ a_cad))))
        rows.append(dict(joint=j.name, urdf_anchor_m=np.round(dc.xanchor[j.id], 5).tolist(), cad_point_m=np.round(cad_w, 5).tolist(),
                         dev_perp_mm=round(perp, 2), dev_axis_deg=round(ang, 2), urdf_axis_world=np.round(ax, 4).tolist(),
                         cad_axis_world=np.round(a_cad, 4).tolist()))
with open(os.path.join(ROOT, "joint_origin_check.csv"), "w", newline="", encoding="utf-8-sig") as fp:
    w = csv.DictWriter(fp, fieldnames=list(rows[0])); w.writeheader(); w.writerows(rows)
print("공칭 URDF vs CAD (CAD 자세, yaw = +π/2) 편차:")
for r in rows: print(f"  {r['joint']:28s} 축 직선까지 {r['dev_perp_mm']:5.2f} mm · 축 각도 {r['dev_axis_deg']:4.2f}°")
# 영점 정의 확인: 팔 내림 자세에서 elbow + 가 앞(+x)으로 굽는지
for side in ("left", "right"):
    de = mujoco.MjData(s); mujoco.mj_forward(s, de); b = s.body(f"{side}_elbow_link").id; x0 = de.xipos[b].copy()
    de.qpos[s.joint(f"{side}_elbow_joint").qposadr[0]] = 0.5; mujoco.mj_forward(s, de)
    print(f"영점 확인 [{side}]: elbow +0.5 rad → 팔꿈치 링크 COM x {x0[0]*1000:+.1f} → {de.xipos[b][0]*1000:+.1f} mm (앞 = +x)")

# 중력 토크: 팔을 내린 자세(0) · 앞으로 수평(pitch -90°)
for label, q in (("팔 내림(q=0)", 0.0), ("팔 앞으로 수평(pitch -π/2)", -np.pi / 2)):
    d2 = mujoco.MjData(s); d2.qpos[s.joint("left_shoulder_pitch_joint").qposadr[0]] = q; mujoco.mj_forward(s, d2)
    tau = {n: d2.qfrc_bias[s.joint(n).dofadr[0]] for n in ("left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_elbow_joint")}
    print(f"중력 토크 [{label}]:", {k: round(v, 3) for k, v in tau.items()}, "Nm")
# 해석 검산: 팔 수평일 때 pitch 토크 = 팔 질량 × g × (pitch 축 → 팔 COM 수평 거리)
d3 = mujoco.MjData(s); d3.qpos[s.joint("left_shoulder_pitch_joint").qposadr[0]] = -np.pi / 2; mujoco.mj_forward(s, d3)
b0 = s.body("left_shoulder_pitch_link").id
arm_com = d3.subtree_com[b0]; jp = d3.xanchor[s.joint("left_shoulder_pitch_joint").id]
print(f"  검산: 팔 질량 {s.body_subtreemass[b0]:.3f} kg × 9.81 × COM 수평거리 {abs(arm_com[0]-jp[0]):.4f} m = {s.body_subtreemass[b0]*9.81*abs(arm_com[0]-jp[0]):.3f} Nm")

# 2 s 유지 (ctrl = 0 목표, 가정 kp · kv)
d4 = mujoco.MjData(s)
for _ in range(int(2 / s.opt.timestep)): mujoco.mj_step(s, d4)
dev = {s.joint(n).name: round(float(np.degrees(d4.qpos[s.joint(n).qposadr[0]])), 3) for n in arm}
print(f"2 s 유지 후 관절각 [deg] (kp {KP}):", dev, "| 베이스 이동", np.round(d4.qpos[:3], 5).tolist(), "| 접촉", d4.ncon)

# 캡처: visual(group 1)만 · collision(group 0, convex hull)만 따로 — 렌더할 때만 색을 칠함(파일은 그대로)
try:
    from PIL import Image
    IMG = os.path.join(os.path.dirname(os.path.dirname(ROOT)), "assets", "screenshots", "week04")   # 저장소 규칙: 캡처는 assets/screenshots/weekNN/
    os.makedirs(IMG, exist_ok=True)
    pal = {"base": (.35, .35, .38), "torso": (.55, .55, .6), "pitch": (.85, .55, .2), "roll": (.25, .55, .85), "yaw": (.35, .75, .4), "elbow": (.8, .3, .3)}
    for g in range(s.ngeom):
        bn = s.body(s.geom_bodyid[g]).name
        for k, c in pal.items():
            if k in bn: s.geom_rgba[g] = (*c, 1)
    r = mujoco.Renderer(s, 720, 960)
    def shot(dd, cam, group, fn):
        opt = mujoco.MjvOption(); opt.geomgroup[:] = 0; opt.geomgroup[group] = 1; opt.geomgroup[2] = 1
        r.update_scene(dd, camera=cam, scene_option=opt); Image.fromarray(r.render()).save(os.path.join(IMG, "week04_" + fn))
    d5 = mujoco.MjData(s)
    for n, q in (("left_shoulder_pitch_joint", -1.2), ("left_elbow_joint", 0.8), ("right_shoulder_roll_joint", -0.6)):
        d5.qpos[s.joint(n).qposadr[0]] = q
    mujoco.mj_forward(s, d5)
    shot(d, "front", 1, "v0_front.png"); shot(d, "iso", 1, "v0_iso.png"); shot(d5, "iso", 1, "v0_pose.png"); shot(d, "iso", 0, "v0_iso_collision.png")
    print("캡처 저장: assets/screenshots/week04/week04_v0_front.png · _iso · _pose · _iso_collision")
except Exception as e:
    print("캡처 생략 (노트북에서는 뷰어 스크린샷):", e)
