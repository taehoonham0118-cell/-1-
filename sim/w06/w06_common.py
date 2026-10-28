# week06 | B1 공용 v1 모델(박종진 d6d6304 빌드, A2 case a2_doc · 중력보상 B on) 로더 + 공통 도우미 (함태훈, B1)
#  모델: 기본 = sim/w06/b1_snapshot/v1 (박종진 d6d6304 사본). 환경변수 B1_V1 로 박종진 저장소 원본 지정 가능.
#  메모리 편집(파일은 그대로):
#    case="cad_ri85"   → 어깨 3관절 armature 0.219→0.098, forcerange 381.7→181.5 (8개 모두 RI85, CAD 대로)
#    payload=m [kg]    → {side}_ee site 위치에 질점 body 추가 (기본: 중력보상 대상 아님 = 모르는 짐)
#    payload_ff=True   → 짐도 gravcomp="1" (= 짐 질량을 아는 중력보상 피드포워드)
#    gains={...}       → 관절군별 kp · kv 직접 지정 (dampratio 대신 고정 kv = Isaac ImplicitActuatorCfg 와 같은 방식)
#    no_gravcomp=True  → 중력보상 B 제거 (비교용)
import glob, json, os, re
import numpy as np, mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
V1 = os.path.abspath(os.environ.get("B1_V1", os.path.join(HERE, "b1_snapshot", "v1")))
OUT = os.environ.get("W06_OUT", os.path.join(HERE, "results")); os.makedirs(OUT, exist_ok=True)
IMG = os.environ.get("W06_IMG", os.path.join(HERE, "..", "..", "assets", "screenshots", "week06")); os.makedirs(IMG, exist_ok=True)
ARM = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"]
CLS = {"shoulder_pitch": "shoulder", "shoulder_roll": "shoulder", "shoulder_yaw": "shoulder", "elbow": "elbow"}
SIDES = ("left", "right")
G = 9.81
A2 = json.load(open(os.path.join(V1, "inputs", "a2_actuators.json"), encoding="utf-8"))


def rated(case, joint):   # 관절 → (기종, 정격, 피크)
    c = A2["cases"][case]["classes"][CLS[joint]]; return c["actuator"], c["rated"], c["peak"]


def assets():
    a = {}
    for f in glob.glob(os.path.join(V1, "mjcf", "*.xml")): a[os.path.basename(f)] = open(f, "rb").read()
    for f in glob.glob(os.path.join(V1, "meshes", "**", "*.stl"), recursive=True):
        a[os.path.relpath(f, os.path.join(V1, "meshes")).replace(os.sep, "/")] = open(f, "rb").read()
    return a


def edit_b1(x, case="a2_doc", payload=0.0, payload_ff=False, gains=None, no_gravcomp=False, sides=SIDES):
    if case == "cad_ri85":
        src = A2["cases"]["cad_ri85"]["classes"]["shoulder"]
        x = x.replace('armature="0.219"', 'armature="%g"' % src["armature"]).replace("381.7", "%g" % src["peak"])
    elif case != "a2_doc":
        raise ValueError(case)
    if no_gravcomp:
        x = x.replace(' gravcomp="1"', "").replace(' actuatorgravcomp="true"', "")
    if gains:   # {"shoulder_pitch": (kp, kv), ...} → 액추에이터별 kp · kv 직접 (dampratio 제거)
        for s in SIDES:
            for j, (kp, kv) in gains.items():
                pat = r'<position class="(\w+)" name="%s_%s" joint="%s_%s_joint" />' % (s, j, s, j)
                x, n = re.subn(pat, r'<position class="\1" name="%s_%s" joint="%s_%s_joint" kp="%g" kv="%g" />' % (s, j, s, j, kp, kv), x)
                assert n == 1, (s, j)
        x = x.replace('dampratio="1.0" ', "")
    if payload > 0:
        r = 0.03; I = 0.4 * payload * r * r   # 반지름 3 cm 균일 구 (질점에 가깝게)
        for s in sides:
            site = '<site name="%s_ee" pos="0 0 -0.1" size="0.012" rgba="1 0.3 0.1 1" group="3" />' % s
            assert site in x
            gc = ' gravcomp="1"' if payload_ff else ""
            x = x.replace(site, site + '<body name="%s_payload" pos="0 0 -0.1"%s><inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g" />'
                          '<geom name="%s_payload_geom" type="sphere" size="%g" contype="0" conaffinity="0" group="1" density="0" rgba="0.9 0.6 0.1 1" /></body>'
                          % (s, gc, payload, I, I, I, s, r))
    return x


def load(name="scene.xml", **kw):
    a = assets()
    a["b1.xml"] = edit_b1(a["b1.xml"].decode("utf-8"), **kw).encode("utf-8")
    return mujoco.MjModel.from_xml_string(open(os.path.join(V1, "mjcf", name), encoding="utf-8").read(), assets=a)


def ik_module():
    import sys
    sys.path.insert(0, os.path.join(V1, "scripts")); import ik
    return ik


def qa(m, j): return m.jnt_qposadr[m.joint(j).id]
def da(m, j): return m.jnt_dofadr[m.joint(j).id]
def arm_q(m, side): return np.array([qa(m, f"{side}_{a}_joint") for a in ARM])
def arm_d(m, side): return np.array([da(m, f"{side}_{a}_joint") for a in ARM])
def arm_u(m, side): return np.array([m.actuator(f"{side}_{a}").id for a in ARM])


def mirror(q_left):
    """왼팔 관절각 → 오른팔 (mirror_rules.md: 오른팔 축이 거울이라 같은 값 = 거울 자세)"""
    return np.array(q_left, float).copy()


def quintic(q0, q1, T, t):
    """최소 저크 5차 다항식: 위치 · 속도 · 가속도 (시작 · 끝 속도 = 가속도 = 0)"""
    s = np.clip(t / T, 0, 1)
    p = 10 * s**3 - 15 * s**4 + 6 * s**5
    v = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
    a = (60 * s - 180 * s**2 + 120 * s**3) / T**2
    dq = np.asarray(q1) - np.asarray(q0)
    return q0 + p * dq, v * dq, a * dq


# ---- 그림 공통 (week05 와 같은 팔레트 · 스타일)
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100", "#e87ba4", "#008300"]
JCOL = dict(zip(ARM, SERIES))


def plot_style():
    import logging, matplotlib
    matplotlib.use("Agg"); logging.getLogger("matplotlib.font_manager").setLevel(logging.ERROR)
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams["font.family"] = [f for f in ("NanumGothic", "Malgun Gothic", "AppleGothic", "Noto Sans CJK KR", "Noto Sans CJK JP", "Noto Serif CJK KR") if f in have] + ["DejaVu Sans"]
    plt.rcParams.update({"axes.unicode_minus": False, "axes.edgecolor": "#8a8986", "axes.labelcolor": "#52514e",
                         "xtick.color": "#52514e", "ytick.color": "#52514e", "axes.grid": True, "grid.color": "#e6e5e1",
                         "grid.linewidth": 0.6, "lines.linewidth": 2, "axes.titlesize": 10, "axes.labelsize": 9,
                         "legend.fontsize": 8, "legend.frameon": False, "axes.spines.top": False, "axes.spines.right": False})
    return plt


def save_md(name, text):
    open(os.path.join(OUT, name), "w", encoding="utf-8").write(text)


def save_json(name, obj):
    json.dump(obj, open(os.path.join(OUT, name), "w", encoding="utf-8"), ensure_ascii=False, indent=1)
