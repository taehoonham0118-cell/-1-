# 토크 회귀 · 판정 테스트 (week10, 함태훈) — 여유율 = (정격 − 필요)/필요 ≥ 20 % 하나
#   필요 = 정적 최악(왼팔 3,000 자세) + 베이스 설계 가속 추가분, 짐 @ 파지 중심, 그리퍼 0.9 kg (8주차 방법, 팀장 승인 48.7)
import numpy as np,pytest
import params as PRM
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
def need(arm,par,mm):
    add=np.array([par["base_accel"]["add_at_worst_Nm"][a] for a in ARM])
    return (np.abs(arm["TG"]+mm*arm["SZ"])+add).max(0)
def m_max(arm,par,case,joint="shoulder_pitch"):
    i=ARM.index(joint); lim=PRM.rated(par,joint,case)/(1+par["torque_rule"]["min_margin"])
    g=np.arange(0,15.0001,0.01); return float(max(x for x in g if need(arm,par,x)[i]<=lim))

def test_margin_definition(par):
    assert PRM.margin(72.6,60.5)==pytest.approx(0.20,abs=1e-3)              # 정격 = 필요 × 1.2 ⇔ 필요 대비 20 %
    assert (72.6-60.5)/72.6==pytest.approx(1/6,abs=1e-3)                    # 같은 조건을 정격 대비로 쓰면 16.7 %
    assert par["torque_rule"]["compare_to"]=="rated"

def test_need_regression(arm,par):
    n=need(arm,par,3.0); R=par["regression"]["need3kg_Nm"]
    assert n[0]==pytest.approx(R["shoulder_pitch"],abs=R["tol"]); assert n[1]==pytest.approx(R["shoulder_roll"],abs=R["tol"])

@pytest.mark.parametrize("case",["a2_doc","cad_ri85"])
def test_both_cases_pass_at_target(arm,par,case):
    n=need(arm,par,par["payload"]["target_kg"])
    for i,a in enumerate(ARM): assert PRM.margin(PRM.rated(par,a,case),n[i])>=par["torque_rule"]["min_margin"],(case,a)

def test_configured_case_with_flags(arm,par):
    case=par["model"]["shoulder_case"]; mm=PRM.payload_effective(par,m_max(arm,par,case)); n=need(arm,par,mm)
    for i,a in enumerate(ARM): assert PRM.margin(PRM.rated(par,a),n[i])>=par["torque_rule"]["min_margin"]-1e-3

@pytest.mark.parametrize("act,case",[("TQ","a2_doc"),("RI85","cad_ri85")])
def test_mmax_regression_and_linear(arm,par,act,case):
    R=par["regression"]["mmax_kg"]; mx=m_max(arm,par,case)
    assert mx==pytest.approx(R[act],abs=R["tol"]); assert mx>=par["payload"]["target_kg"]
    i=int((np.abs(arm["TG"]+3*arm["SZ"])[:,0]).argmax()); s=np.sign(arm["TG"][i,0]+3*arm["SZ"][i,0])
    tau0=s*arm["TG"][i,0]+par["base_accel"]["add_at_worst_Nm"]["shoulder_pitch"]; gr=s*arm["SZ"][i,0]
    lin=(PRM.rated(par,"shoulder_pitch",case)/1.2-tau0)/gr                    # m_max = (τ정격/1.2 − τ0)/(g·r)
    assert lin==pytest.approx(mx,abs=0.02)

def test_payload_limit_flag(arm,par):
    p=dict(par); p["limits"]={**par["limits"],"payload_limit":{"enable":True,"max_kg":2.0}}
    assert PRM.payload_effective(p)==2.0
    p["payload"]={**par["payload"],"target_kg":6.0}; p["limits"]["payload_limit"]={"enable":True,"max_kg":"auto"}
    mx=m_max(arm,par,"cad_ri85"); mm=PRM.payload_effective(p,mx); assert mm==mx
    assert PRM.margin(72.6,need(arm,par,mm)[0])>=0.20-1e-3

def test_reach_limit_flag(arm,par):
    # 짐 5.5 kg · RI85 이면 도달 제한이 걸림: r_max 이하 자세는 모두 20 % 만족, 그보다 먼 자세 중 하나는 불만족
    mm=5.5; add=np.array([par["base_accel"]["add_at_worst_Nm"][a] for a in ARM[:2]])
    npz=np.abs(arm["TG"]+mm*arm["SZ"])[:,:2]+add; o=np.argsort(arm["RHO"]); env=np.maximum.accumulate(npz[o],axis=0)
    ok=(env<=72.6/1.2).all(1); j=int(np.argmin(ok)); r=arm["RHO"][o][j-1]
    assert 0.5<r<0.65
    inside=arm["RHO"]<=r; assert (npz[inside]<=72.6/1.2).all() and not (npz[~inside]<=72.6/1.2).all()

def test_hand_check_slope(arm,par):
    i=int((np.abs(arm["TG"]+3*arm["SZ"])[:,0]).argmax())
    assert abs(arm["SZ"][i,0])/9.81==pytest.approx(0.638,abs=0.002)          # 기울기 6.26 N·m/kg = g × 0.638 m
