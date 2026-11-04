# week07 | B1 공용 v1 모델(게인 통일 빌드) 로더 + 공통 도우미 (함태훈, B1)
#  모델: 기본 = sim/w07/b1_snapshot/v1 — 박종진 d6d6304 `b1/models/v1` + 함태훈 week07 게인 통일 패치
#        (actuator_params.json "joint_gains" + build_v1.py 몇 줄, proposal/build_v1_joint_gains.diff) 로 다시 빌드한 사본.
#        팔 position 액추에이터 = 관절별 고정 kp · kv (pitch 400·31 / roll 350·27.5 / yaw 300·15 / elbow 300·11), A2 case a2_doc, 중력보상 B on.
#        환경변수 B1_V1 로 박종진 저장소 원본(합의 후 빌드본)을 가리킬 수 있음.
#  메모리 편집(파일은 그대로):
#    payload=m [kg]     → {side}_ee site 위치에 짐 body (반지름 3 cm 구)
#    payload_gc=f       → 짐 body gravcomp = f (0 = 모르는 짐, 1 = 질량을 아는 중력보상 FF, 2/3 = 2 kg 로 가정했는데 실제 3 kg …)
#    no_gravcomp=True   → 중력보상 B 제거 (비교용)
#  짐을 시뮬 중에 잡기 · 놓기: set_payload(m, side, mass) — body_mass · body_inertia 를 런타임에 바꿈(질량행렬 · 중력 · gravcomp 모두 매 스텝 이 값을 씀)
import glob, json, os, re
import numpy as np, mujoco

HERE = os.path.dirname(os.path.abspath(__file__))
V1 = os.path.abspath(os.environ.get("B1_V1", os.path.join(HERE, "b1_snapshot", "v1")))
OUT = os.environ.get("W07_OUT", os.path.join(HERE, "results")); os.makedirs(OUT, exist_ok=True)
IMG = os.environ.get("W07_IMG", os.path.join(HERE, "..", "..", "assets", "screenshots", "week07")); os.makedirs(IMG, exist_ok=True)
ARM = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"]
BASE = ["base_planar_joint_TX", "base_planar_joint_TY", "base_planar_joint_RZ"]
BASE_ACT = ["base_vx", "base_vy", "base_wz"]
CLS = {"shoulder_pitch": "shoulder", "shoulder_roll": "shoulder", "shoulder_yaw": "shoulder", "elbow": "elbow"}
SIDES = ("left", "right")
G = 9.81
R_PAY = 0.03
A2 = json.load(open(os.path.join(V1, "inputs", "a2_actuators.json"), encoding="utf-8"))
PARAMS = json.load(open(os.path.join(V1, "actuator_params.json"), encoding="utf-8"))
GAINS = {j: (v["kp"], v["kv"]) for j, v in PARAMS["joint_gains"].items() if not j.startswith("_")}


def rated(case, joint):   # 관절 → (기종, 정격, 피크)
    c = A2["cases"][case]["classes"][CLS[joint]]; return c["actuator"], c["rated"], c["peak"]


def assets():
    a = {}
    for f in glob.glob(os.path.join(V1, "mjcf", "*.xml")): a[os.path.basename(f)] = open(f, "rb").read()
    for f in glob.glob(os.path.join(V1, "meshes", "**", "*.stl"), recursive=True):
        a[os.path.relpath(f, os.path.join(V1, "meshes")).replace(os.sep, "/")] = open(f, "rb").read()
    return a


def edit_b1(x, payload=0.0, payload_gc=0.0, no_gravcomp=False, sides=SIDES, gains=None):
    if no_gravcomp:
        x = x.replace(' gravcomp="1"', "").replace(' actuatorgravcomp="true"', "")
    if gains:   # {"shoulder_pitch": (kp, kv)} → 액추에이터 kp · kv 덮어쓰기 (비교용, 예: week05 dampratio 대신)
        for s in SIDES:
            for j, (kp, kv) in gains.items():
                x, n = re.subn(r'(<position class="\w+" name="%s_%s" joint="%s_%s_joint") kp="[\d.]+" kv="[\d.]+"' % (s, j, s, j),
                               r'\1 kp="%g" kv="%g"' % (kp, kv), x); assert n == 1, (s, j)
    if payload > 0:
        I = 0.4 * payload * R_PAY ** 2
        for s in sides:
            site = '<site name="%s_ee" pos="0 0 -0.1" size="0.012" rgba="1 0.3 0.1 1" group="3" />' % s
            assert site in x
            gc = ' gravcomp="%g"' % payload_gc if payload_gc else ""
            x = x.replace(site, site + '<body name="%s_payload" pos="0 0 -0.1"%s><inertial pos="0 0 0" mass="%g" diaginertia="%g %g %g" />'
                          '<geom name="%s_payload_geom" type="sphere" size="%g" contype="0" conaffinity="0" group="1" density="0" rgba="0.9 0.6 0.1 1" /></body>'
                          % (s, gc, payload, I, I, I, s, R_PAY))
    return x


def load(name="scene.xml", **kw):
    a = assets()
    a["b1.xml"] = edit_b1(a["b1.xml"].decode("utf-8"), **kw).encode("utf-8")
    return mujoco.MjModel.from_xml_string(open(os.path.join(V1, "mjcf", name), encoding="utf-8").read(), assets=a)


def set_payload(m, side, mass):
    """잡기 · 놓기: 짐 질량을 런타임에 바꿈 (0 대신 1e-9 kg)"""
    b = m.body(f"{side}_payload").id; mass = max(mass, 1e-9)
    m.body_mass[b] = mass; m.body_inertia[b] = 0.4 * mass * R_PAY ** 2


def qa(m, j): return m.jnt_qposadr[m.joint(j).id]
def da(m, j): return m.jnt_dofadr[m.joint(j).id]
def arm_q(m, side): return np.array([qa(m, f"{side}_{a}_joint") for a in ARM])
def arm_d(m, side): return np.array([da(m, f"{side}_{a}_joint") for a in ARM])
def arm_u(m, side): return np.array([m.actuator(f"{side}_{a}").id for a in ARM])
def base_q(m): return np.array([qa(m, j) for j in BASE])
def base_d(m): return np.array([da(m, j) for j in BASE])
def base_u(m): return np.array([m.actuator(a).id for a in BASE_ACT])


def drive_torque(m, d, dofs):
    """관절 구동 토크 = qfrc_actuator(PD + actuatorgravcomp 보상분) + qfrc_applied(외부 FF)"""
    return d.qfrc_actuator[dofs] + d.qfrc_applied[dofs]


def quintic(q0, q1, T, t):
    s = np.clip(t / T, 0, 1)
    p = 10 * s**3 - 15 * s**4 + 6 * s**5
    v = (30 * s**2 - 60 * s**3 + 30 * s**4) / T
    a = (60 * s - 180 * s**2 + 120 * s**3) / T**2
    dq = np.asarray(q1) - np.asarray(q0)
    return q0 + p * dq, v * dq, a * dq


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
    json.dump(obj, open(os.path.join(OUT, name), "w", encoding="utf-8"), ensure_ascii=False, indent=1, default=float)
