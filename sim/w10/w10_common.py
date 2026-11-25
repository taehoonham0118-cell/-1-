# 함태훈 week10 공통: 집기 scene 모델(박종진 pick_env, 그리퍼 0.9 kg) 위에서 왼팔 자세별 토크 성분을 한 번에 계산
#   τ(q, m, a_b) = τ_g0(q) + m·Sz(q) + A_r(q; a_b) + m·A_p(q; a_b)   — 짐 m 은 파지 중심(ee − 45 mm) 점질량
#   τ_g0 = 로봇 자체 중력(qfrc_bias, q̇ = 0), Sz = g·J_zᵀ(파지 중심), A_r = M_arm,base · q̈_b(베이스 가속에 따른 로봇 관성 토크),
#   A_p = J_pᵀ · a_p(짐 점의 가속, a_p = a_lin + α ẑ × (p − 베이스 원점)) — 모두 짐 질량 m 에 선형
import os,sys,numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(os.path.dirname(HERE))
PJJ=os.environ.get("PJJ",os.path.join(ROOT,"..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P
sys.path.insert(0,os.path.join(ROOT,"config")); import params as PRM
PAR=PRM.load()                                                            # 값은 config/params.yaml 한 곳에서
G=9.81; SF=1+PAR["torque_rule"]["min_margin"]; ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
RATED={k:v["rated"] for k,v in PAR["actuators"].items() if k!="source"}; PEAK={k:v["peak"] for k,v in PAR["actuators"].items() if k!="source"}
CASES=PAR["cases"]
ACC_LIN=PAR["base_accel"]["lin_mps2"]; ACC_YAW=PAR["base_accel"]["yaw_radps2"]
ADD_W07=PAR["base_accel"]["add_at_worst_Nm"]
def margin(rated,need): return (rated-need)/need*100.0               # 여유율(필요 대비) — 10주차부터 이 정의 하나

class Arm:
    def __init__(s):
        s.m=P.build(); s.d=mujoco.MjData(s.m); m=s.m
        jid=[m.joint(f"left_{a}_joint").id for a in ARM]
        s.qa=m.jnt_qposadr[jid]; s.da=m.jnt_dofadr[jid]; s.lo=m.jnt_range[jid,0]; s.hi=m.jnt_range[jid,1]
        s.ee=m.site("left_ee").id; s.off=P.GRIPPER["grasp_off"]
        s.bd=np.arange(3)                                                 # 베이스 planar: x · y · yaw (qpos 0 · 1 · 2)
        s.sh=m.body("left_shoulder_pitch_link").id
    def terms(s,q):
        m,d=s.m,s.d; d.qpos[:]=m.qpos0; d.qpos[s.qa]=q; d.qvel[:]=0; mujoco.mj_forward(m,d)
        tg=d.qfrc_bias[s.da].copy()
        R=d.site_xmat[s.ee].reshape(3,3); p=d.site_xpos[s.ee]-s.off*R[:,2]
        jp=np.zeros((3,m.nv)); mujoco.mj_jac(m,d,jp,None,p,m.site_bodyid[s.ee]); Jp=jp[:,s.da]
        Sz=G*Jp[2]
        base0=d.xpos[m.body("base_link").id] if True else None
        Ar=[];Ap=[]
        for acc in ((ACC_LIN,0,0),(0,ACC_LIN,0),(0,0,ACC_YAW)):
            qa=np.zeros(m.nv); qa[s.bd]=acc; out=np.zeros(m.nv); mujoco.mj_mulM(m,d,out,qa); Ar.append(out[s.da].copy())
            rel=p-base0; ap=np.array([acc[0],acc[1],0.0])+np.cross([0,0,acc[2]],rel); Ap.append(Jp.T@ap)
        shp=d.xanchor[m.joint("left_shoulder_pitch_joint").id]
        rho=float(np.hypot(*(p-shp)[:2]))                                 # 어깨 pitch 관절점 → 파지 중심 수평 거리
        return tg,Sz,np.array(Ar),np.array(Ap),rho,p.copy()
    def sample(s,n=3000,seed=0):
        return s.lo+(s.hi-s.lo)*np.random.default_rng(seed).random((n,4))
