# week05 | v1 공용 모델(B1, 박종진 build_v1.py 빌드) 로더 + 공통 도우미 (함태훈, B1)
#  모델 위치: 기본 = sim/v1/b1_snapshot/v1 (박종진 652b89e 사본). 환경변수 B1_V1 로 박종진 저장소 원본을 가리킬 수 있다.
#    예) set B1_V1=C:\...\PJJ0337-1-\b1\models\v1     (Windows)   /  export B1_V1=~/pjj/b1/models/v1
#  Windows 한글 경로 대응: xml · STL 을 파이썬이 읽어 메모리 assets 로 넘긴다 (4주차 mj_load.py 와 같은 방식)
import glob, json, os
import numpy as np, mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
V1 = os.environ.get("B1_V1", os.path.join(HERE, "b1_snapshot", "v1"))     # 기본 = 저장소 안 스냅샷 (박종진 652b89e)
V1 = os.path.abspath(V1)
OUT = os.environ.get("V1_OUT", os.path.join(HERE, "results")); os.makedirs(OUT, exist_ok=True)          # V1_OUT: 노트북 재현 시 다른 폴더로
IMG = os.environ.get("V1_IMG", os.path.join(HERE, "..", "..", "assets", "screenshots", "week05")); os.makedirs(IMG, exist_ok=True)
ARM = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"]
SIDES = ("left", "right")
G = 9.81


def assets():
    a = {}
    for f in glob.glob(os.path.join(V1, "mjcf", "*.xml")): a[os.path.basename(f)] = open(f, "rb").read()
    for f in glob.glob(os.path.join(V1, "meshes", "*.stl")): a[os.path.basename(f)] = open(f, "rb").read()
    return a


def gravcomp_edit(x):
    """중력보상 방식 B: 팔 body gravcomp="1" + joint actuatorgravcomp (메모리에서만, 파일 그대로)"""
    import re
    x = re.sub(r'<body name="((left|right)_(shoulder|elbow|tool0|gripper)[^"]*)"', r'<body name="\1" gravcomp="1"', x)
    return x.replace('<joint armature=', '<joint actuatorgravcomp="true" armature=')


def load(name="scene.xml", xml_edit=None):
    """scene.xml 을 읽는다. xml_edit(문자열→문자열)로 b1.xml 을 메모리에서만 바꿔 실험할 수 있다 (파일은 그대로)."""
    a = assets()
    if xml_edit:
        a["b1.xml"] = xml_edit(a["b1.xml"].decode("utf-8")).encode("utf-8")
    return mujoco.MjModel.from_xml_string(open(os.path.join(V1, "mjcf", name), encoding="utf-8").read(), assets=a)


def params():
    return json.load(open(os.path.join(V1, "actuator_params.json"), encoding="utf-8"))


def assumptions():
    return json.load(open(os.path.join(V1, "..", "assumptions_b1.json"), encoding="utf-8"))


def qa(m, j): return m.jnt_qposadr[m.joint(j).id]
def da(m, j): return m.jnt_dofadr[m.joint(j).id]


def set_arm(m, d, q, side="left"):
    for a, v in zip(ARM, q): d.qpos[qa(m, f"{side}_{a}_joint")] = v


# ---- 그림 공통 (matplotlib, 저장소 PNG) — 계열 색은 고정 순서, 글자는 무채색
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]   # 기준 팔레트 1~6번 고정 순서 (validate_palette.js 통과)
JCOL = dict(zip(ARM, SERIES))


def plot_style():
    import logging, matplotlib
    matplotlib.use("Agg"); logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = [f for f in ("NanumGothic", "Malgun Gothic", "AppleGothic") if f in have] + ["DejaVu Sans"]
    plt.rcParams.update({"axes.unicode_minus": False, "axes.edgecolor": "#8a8986", "axes.labelcolor": "#52514e",
                         "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e1",
                         "grid.linewidth": 0.6, "lines.linewidth": 2, "axes.titlesize": 10, "axes.labelsize": 9,
                         "legend.fontsize": 8, "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False})
    return plt
