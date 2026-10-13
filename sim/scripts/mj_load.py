# week04 | MuJoCo 모델을 「파일 경로 대신 문자열 + 메모리 asset」으로 여는 도우미 (함태훈, B1)
# 이유: Windows 에서 경로에 한글(예: C:\Users\함태훈\...)이 있으면 MjModel.from_xml_path 가 파일을 못 여는 경우가 있다
#       (박종진 week03: mjio.py 로 우회). Python 이 파일을 읽어 넘기면 경로 인코딩 문제가 없다.
# 사용: from mj_load import load_model;  m = load_model("sim/out/robot.urdf")
import os, re
import mujoco


def load_model(path):
    path = os.path.abspath(path)
    base = os.path.dirname(path)
    xml = open(path, encoding="utf-8").read()
    md = re.search(r'meshdir="([^"]*)"', xml)
    meshdir = os.path.join(base, md.group(1)) if md else base
    assets = {}
    for f in re.findall(r'(?:file|filename)="([^"]+\.(?:stl|STL|obj|OBJ|msh))"', xml):
        f = f.replace("package://", "")
        p = f if os.path.isabs(f) else os.path.join(meshdir, f)
        if os.path.exists(p):
            assets[os.path.basename(f)] = open(p, "rb").read()
            assets[f] = assets[os.path.basename(f)]
    return mujoco.MjModel.from_xml_string(xml, assets)
