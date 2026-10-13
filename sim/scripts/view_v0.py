# week04 | 우리 로봇 v0 를 MuJoCo 뷰어로 열기 — 한글 경로(바탕 화면 · OneDrive)에서도 열리게 (함태훈, B1)
# 이유: Windows 에서 경로에 한글이 있으면 `python -m mujoco.viewer --mjcf=...` 가 xml · STL 을 못 여는 경우가 있음
#       → Python 이 scene.xml · include 파일 · STL 을 읽어 메모리(assets)로 넘긴 뒤 뷰어를 띄운다
# 실행 (4주차_작업 또는 저장소 루트에서): python sim/scripts/view_v0.py
import os
import mujoco, mujoco.viewer

ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "robot")
assets = {fn: open(os.path.join(ROOT, fn), "rb").read() for fn in ("robot.xml", "robot_actuated.xml")}
for fn in os.listdir(os.path.join(ROOT, "meshes")):
    assets[fn] = open(os.path.join(ROOT, "meshes", fn), "rb").read()
m = mujoco.MjModel.from_xml_string(open(os.path.join(ROOT, "scene.xml"), encoding="utf-8").read(), assets)
print(f"mujoco {mujoco.__version__}: body {m.nbody} · joint {m.njnt} · actuator {m.nu} · 총질량 {m.body_subtreemass[0]:.3f} kg")
mujoco.viewer.launch(m)
