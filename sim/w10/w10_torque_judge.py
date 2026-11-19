# 함태훈 week10 — 팀장 11/19 회신 반영: 토크 판정 = 「실제 정격 ≥ 요구 정격 = 필요 토크 × 1.2」 하나로 (「PDR 52 N·m 기준 여유 %」 삭제)
#   ① 8주차 토크 예산(sim/w08/results/torque_budget_w08.csv, 필요 = 3 kg 파지 + 그리퍼 0.9 kg + 베이스 가속)을 새 기준으로 다시 판정
#   ② 8주차 「52 N·m 최대 짐 2.14 kg」(w08_payload_limit.py) 대신 → 실제 정격(TQ 118.8 · RI85 72.6)에서 요구 정격을 지키는 최대 짐
#      같은 가정: 짐 @ 파지 중심(ee − 45 mm), 왼팔 무작위 3,000 자세(rng 0) 정적 최악 + 베이스 설계 가속 추가분(pitch 2.16 · roll 4.19)
#   실행: python sim/w10/w10_torque_judge.py  (~10 s, PJJ = 박종진 저장소 경로) → results/torque_judge_w10.md · .csv
import sys,os,csv,numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(os.path.dirname(HERE))
PJJ=os.environ.get("PJJ",os.path.join(ROOT,"..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P
SF=1.2; G=9.81; OFF=P.GRIPPER["grasp_off"]; ADD={"shoulder_pitch":2.16,"shoulder_roll":4.19}
rows=list(csv.DictReader(open(os.path.join(ROOT,"sim","w08","results","torque_budget_w08.csv"))))
out=[]
for r in rows:
    need=float(r["need_w08"]); rated=float(r["rated"]); req=need*SF
    out.append(dict(case=r["case"],joint=r["joint"],actuator=r["actuator"],rated=rated,need=round(need,2),required=round(req,2),
                    ok=rated>=req,margin_need=round((rated/need-1)*100,1),margin_req=round((rated/req-1)*100,1)))
# ② 최대 짐 (정적 최악 + 베이스 가속, 짐 질량에 선형인 τ = τ0 + m · g · J_zᵀ)
m=P.build(); d=mujoco.MjData(m); J4=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; jid=[m.joint(f"left_{a}_joint").id for a in J4]
qa=m.jnt_qposadr[jid]; da=m.jnt_dofadr[jid]; ee=m.site("left_ee").id; lo=m.jnt_range[jid,0]; hi=m.jnt_range[jid,1]
Q=lo+(hi-lo)*np.random.default_rng(0).random((3000,4)); T0=[]; S=[]
for q in Q:
    d.qpos[:]=m.qpos0; d.qpos[qa]=q; mujoco.mj_forward(m,d); T0.append(d.qfrc_bias[da].copy())
    R=d.site_xmat[ee].reshape(3,3); p=d.site_xpos[ee]-OFF*R[:,2]; jp=np.zeros((3,m.nv)); mujoco.mj_jac(m,d,jp,None,p,m.site_bodyid[ee]); S.append(G*jp[2,da])
T0=np.array(T0); S=np.array(S); ms=np.round(np.arange(0,15.0005,0.01),2)
need={a:np.array([np.abs(T0[:,i]+x*S[:,i]).max()+ADD[a] for x in ms]) for i,a in enumerate(J4[:2])}
k3=int(round(3/0.01)); chk={a:round(float(need[a][k3]),2) for a in need}
pay=[]
for act,rated in (("TQ",118.8),("RI85",72.6)):
    for a in need:
        okm=ms[need[a]<=rated/SF]; sl=(need[a][k3]-need[a][int(round(2/0.01))])
        hand=3+(rated/SF-need[a][k3])/sl
        pay.append(dict(actuator=act,joint=a,limit=round(rated/SF,2),max_kg=round(float(okm.max()),2),hand=round(float(hand),2)))
L=["# 토크 판정 정리 — 실제 정격 ≥ 요구 정격(필요 × 1.2) (week10, 함태훈) — 자동 생성 `w10_torque_judge.py`","",
"- 팀장 11/19 회신: 52 N·m 는 구동기 사양이 아니라 6주차 필요 토크 42.9 × 1.2 에서 나온 요구치 → 필요 토크가 48.7 로 오르면 요구치도 58.4 로 같이 오름. **판정 = 실제 정격 ≥ 요구 정격**, 「PDR 52 기준 여유 %」 열은 삭제",
"- 필요 토크 = 8주차 `sim/w08/results/torque_budget_w08.csv` 그대로(3 kg @ 파지 중심 ee − 45 mm, 그리퍼 0.9 kg, 정적 최악 3,000 자세 + 베이스 가속 1 m/s² · 1.57 rad/s² + 옆면 집기 실행 확인). 정격 여유 % = 실제 정격 / 필요 − 1 (박종진 torque_budget_b1.md 와 같은 정의, 참고)",
f"- 검산: 이 스크립트의 3 kg 필요 토크 pitch {chk['shoulder_pitch']} · roll {chk['shoulder_roll']} N·m = 8주차 표(48.69 · 46.71)","",
"## 1. 판정표 (짐 3 kg)","","| case | 관절 | 실제 정격 | 필요 | **요구 정격 (× 1.2)** | 판정 | 정격 여유 % (정격/필요 − 1) | 요구 정격 대비 여유 % |","|---|---|---|---|---|---|---|---|"]
for o in out: L.append(f"| {o['case']} | {o['joint']} | {o['actuator']} {o['rated']} | {o['need']} | **{o['required']}** | {'**OK**' if o['ok'] else '부족'} | {o['margin_need']} | {o['margin_req']} |")
L+=["",f"- 어깨 요구 정격 = pitch **{out[0]['required']}** · roll **{out[1]['required']}** N·m → a2_doc(TQ 118.8) · cad_ri85(RI85 72.6) **둘 다 통과, 짐 3 kg 그대로**. 가장 빠듯한 곳 = cad_ri85 pitch(요구 정격 대비 +{out[4]['margin_req']} %)",
"","## 2. 실제 정격에서 요구 정격을 지키는 최대 짐 (8주차 「52 N·m → 2.14 kg」 대체)","","| 구동기 | 관절 | 필요 한도 = 정격 / 1.2 | 최대 짐 (3,000 자세 스윕) | 손 검산 3 + (한도 − 필요₃) / 기울기 |","|---|---|---|---|---|"]
for p in pay: L.append(f"| {p['actuator']} | {p['joint']} | {p['limit']} | **{p['max_kg']} kg** | {p['hand']} kg |")
L+=["","- 기울기(짐 1 kg 당 필요 토크) = pitch 6.26 · roll 5.91 N·m/kg (≈ g × 수평거리 0.638 · 0.603 m, 8주차와 같음)",
"- 어깨 기준으로 RI85 여도 짐 약 4.9 kg 까지(목표 3 kg 의 1.6배), TQ 면 11 kg → 짐 3 kg 에서는 두 case 모두 토크 여유가 있어 어깨 구동기 결정이 토크 판정으로 갈리지 않음(팀장 회신과 같은 결론)",
"- 최대 짐은 어깨 2축만 계산 — 3 kg 넘는 짐에서 팔꿈치 · 그리퍼 파지력(m ≤ 2μF/g, 40 N · μ 1 → 8.2 kg) · 구조 강도는 따로 확인해야 함",
"- 한계: 정적 최악 + 설계 가속까지. 빠른 동작 · 충돌은 피크(TQ 381.7 · RI85 181.5)로 따로 봄"]
open(os.path.join(HERE,"results","torque_judge_w10.md"),"w",encoding="utf-8").write("\n".join(L)+"\n")
with open(os.path.join(HERE,"results","torque_judge_w10.csv"),"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=list(out[0])); w.writeheader(); w.writerows(out)
print("\n".join(L))
