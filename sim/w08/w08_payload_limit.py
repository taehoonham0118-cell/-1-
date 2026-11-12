# 함태훈 week08 — PDR 52 N·m 에서 × 1.2 규칙(필요 ≤ 43.33 N·m)을 지키는 최대 짐 (A팀 추가 문안 근거)
#   w08_grasp_torque.py 와 같은 가정: 짐 = 파지 중심(ee − 45 mm), 그리퍼 0.9 kg(모델 손바닥 0.5 + 손가락 0.2 × 2), 왼팔 무작위 3,000 자세(rng 0) 정적 최악
#   + 7주차 베이스 설계 가속 추가분(최악 자세 값: pitch 2.16 · roll 4.19 N·m). 출력: results/payload_limit.md
import sys,os,numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__))
PJJ=os.environ.get("PJJ",os.path.join(HERE,"..","..","..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P
ARM=["shoulder_pitch","shoulder_roll"]; G=9.81; OFF=P.GRIPPER["grasp_off"]; ADD={"shoulder_pitch":2.16,"shoulder_roll":4.19}; LIM=52/1.2
m=P.build(); d=mujoco.MjData(m)
J4=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; jid=[m.joint(f"left_{a}_joint").id for a in J4]
qa=m.jnt_qposadr[jid]; da=m.jnt_dofadr[jid]; ee=m.site("left_ee").id; lo=m.jnt_range[jid,0]; hi=m.jnt_range[jid,1]
Q=lo+(hi-lo)*np.random.default_rng(0).random((3000,4)); T0=[]; S=[]
for q in Q:   # τ(m) = τ_로봇(그리퍼 포함) + m · g · J_z(파지 중심)ᵀ — 짐 질량에 선형
    d.qpos[:]=m.qpos0; d.qpos[qa]=q; mujoco.mj_forward(m,d); T0.append(d.qfrc_bias[da].copy())
    R=d.site_xmat[ee].reshape(3,3); p=d.site_xpos[ee]-OFF*R[:,2]; jp=np.zeros((3,m.nv)); mujoco.mj_jac(m,d,jp,None,p,m.site_bodyid[ee]); S.append(G*jp[2,da])
T0=np.array(T0); S=np.array(S); ms=np.round(np.arange(0,3.0005,0.001),3)
L=["# PDR 52 N·m × 1.2 를 지키는 최대 짐 (week08, 함태훈) — 자동 생성 `w08_payload_limit.py`","",f"기준: 필요 토크 ≤ 52/1.2 = {LIM:.2f} N·m. 필요 = 3,000 자세 정적 최악(짐 @ 파지 중심 + 그리퍼 0.9 kg) + 베이스 설계 가속 추가분","",
   "| 짐 kg | pitch 필요 N·m | roll 필요 N·m |","|---|---|---|"]
need={a:np.array([np.abs(T0[:,i]+x*S[:,i]).max()+ADD[a] for x in ms]) for i,a in enumerate(ARM)}
for x in (0,1,1.5,2,2.5,3): k=int(round(x*1000)); L.append(f"| {x} | {need['shoulder_pitch'][k]:.2f} | {need['shoulder_roll'][k]:.2f} |")
L.append("")
for a in ARM:
    ok=ms[need[a]<=LIM]; sl=(need[a][3000]-need[a][2000])
    L.append(f"- {a}: 최대 짐 **{ok.max():.2f} kg**, 기울기 {sl:.2f} N·m/kg (≈ g × 수평거리 {sl/G:.3f} m)")
L.append(f"- 검산: 3 kg 에서 pitch {need['shoulder_pitch'][3000]:.2f} · roll {need['shoulder_roll'][3000]:.2f} N·m = grasp_torque.md 표(48.69 · 46.71)")
os.makedirs(os.path.join(HERE,"results"),exist_ok=True); open(os.path.join(HERE,"results","payload_limit.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))
