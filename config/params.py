# config/params.yaml 로더 · 파생값 (week10, 함태훈)
#   load() → dict, rated(p, joint, case=None), payload_effective(p), margin(rated, need)
#   auto 값(m_max · r_max)은 sim/w10/w10_common 의 자세 스윕으로 계산 — 결과 파일이 아니라 모델에서 직접 계산
import os,yaml
HERE=os.path.dirname(os.path.abspath(__file__))
DEFAULT=os.path.join(HERE,"params.yaml")
KEYS=("schema","model","actuators","cases","torque_rule","payload","gripper","base_accel","limits","regression")
def load(path=None):
    p=yaml.safe_load(open(path or os.environ.get("B1_PARAMS",DEFAULT),encoding="utf-8"))
    miss=[k for k in KEYS if k not in p]; assert not miss,f"params.yaml 키 없음: {miss}"
    assert p["model"]["shoulder_case"] in p["cases"],"shoulder_case 가 cases 에 없음"
    assert p["torque_rule"]["compare_to"]=="rated"
    return p
def actuator(p,joint,case=None): return p["cases"][case or p["model"]["shoulder_case"]][joint]
def rated(p,joint,case=None): return float(p["actuators"][actuator(p,joint,case)]["rated"])
def margin(r,need): return (r-need)/need
def required(p,need): return need*(1+p["torque_rule"]["min_margin"])
def payload_effective(p,m_max=None):
    """짐 제한 플래그가 켜져 있으면 min(목표 짐, max_kg) — max_kg=auto 면 m_max(호출자가 계산해 넘김)"""
    t=float(p["payload"]["target_kg"]); L=p["limits"]["payload_limit"]
    if not L["enable"]: return t
    mx=m_max if L["max_kg"]=="auto" else float(L["max_kg"]); assert mx is not None
    return min(t,mx)
