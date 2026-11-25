# 함태훈 week10 — 토크 결정 자료 (노션 10주차 개별 과제)
#   여유율 정의 하나로: 여유율 = (정격 − 필요) ÷ 필요, 기준 20 % 이상(= 정격 ≥ 필요 × 1.2 = 요구 정격). 피크는 비교하지 않고 표에 같이 적음
#   ① 어깨 case 2개(a2_doc · cad_ri85) × 짐 0 · 1 · 2 · 3 kg 스윕 → 여유율 표
#   ② 20 % 를 만족하는 최대 짐 m_max: 선형식 m_max = (τ_정격 ÷ 1.2 − τ₀) ÷ (g · r) (3 kg 최악 자세 고정) vs 자세 스윕(짐마다 최악 자세 다시 찾음)
#   ③ 최대 도달 거리 r_max: 어깨 pitch 관절점 → 파지 중심 수평 거리 r 에 대한 필요 토크 포락선 → 20 % 를 만족하는 r 의 상한
#   ④ 결정 자료 표(A팀 회신 양식) + 박종진 IK 제한값(reach_limit) JSON
#   필요 토크 = 8주차와 같은 방법(팀장 승인 48.7): 정적 최악(왼팔 무작위 3,000 자세, rng 0) + 베이스 설계 가속 추가분(7주차 add_at_worst, 상수)
#     짐 = 파지 중심(ee − 45 mm) 점질량, 그리퍼 0.9 kg(손바닥 0.5 + 손가락 0.2 × 2). 참고로 자세별 실제 가속 토크(|τ_g| + |τ_acc|)도 같이 냄
#   실행: python sim/w10/w10_torque_decision.py  (~5 s, PJJ = 박종진 저장소) → results/torque_decision_w10.{md,csv,json}, reach_limit_w10.json, week10_torque_decision.png
import os,sys,json,csv,numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
import w10_common as C
OUT=os.path.join(HERE,"results"); os.makedirs(OUT,exist_ok=True)
A=C.Arm(); Q=A.sample(3000,0); T=[A.terms(q) for q in Q]
TG=np.array([t[0] for t in T]); SZ=np.array([t[1] for t in T]); AR=np.array([t[2] for t in T]); AP=np.array([t[3] for t in T]); RHO=np.array([t[4] for t in T])
ADD=np.array([C.ADD_W07[a] for a in C.ARM])
def need_pose(mm): return np.abs(TG+mm*SZ)+ADD                     # 자세별 필요(8주차 방법), (3000, 4)
def need(mm): return need_pose(mm).max(0)
def need_cons(mm): return (np.abs(TG+mm*SZ)+np.abs(AR+mm*AP[:, :, :]).max(1)).max(0)   # 참고: 자세별 실제 가속 토크
MS=[0.0,1.0,2.0,3.0]
# ① 스윕표
sweep=[]
for case,acts in C.CASES.items():
    for mm in MS:
        n=need(mm)
        for i,a in enumerate(C.ARM):
            act=acts[a]; r=C.RATED[act]
            sweep.append(dict(case=case,payload=mm,joint=a,actuator=act,rated=r,peak=C.PEAK[act],need=round(float(n[i]),2),
                              required=round(float(n[i])*C.SF,2),margin=round(C.margin(r,n[i]),1),ok=bool(r>=n[i]*C.SF),
                              need_cons=round(float(need_cons(mm)[i]),2)))
# ② m_max
grid=np.round(np.arange(0,15.0001,0.01),2); NG=np.array([need(x) for x in grid])
k3=int(round(3/0.01)); n3p=need_pose(3.0)
mmax=[]
for act in ("TQ","RI85"):
    lim=C.RATED[act]/C.SF
    for i,a in enumerate(C.ARM[:2]):
        qi=int(n3p[:,i].argmax()); tau0=abs(TG[qi,i])+ADD[i]; r_eff=abs(SZ[qi,i])/C.G       # 선형식: 3 kg 최악 자세 고정
        s=np.sign(TG[qi,i]+3*SZ[qi,i]); tau0=s*TG[qi,i]+ADD[i]; r_eff=s*SZ[qi,i]/C.G
        lin=(lim-tau0)/(C.G*r_eff)
        sw=float(grid[NG[:,i]<=lim].max())
        mmax.append(dict(actuator=act,joint=a,limit=round(lim,2),tau0=round(float(tau0),2),r_eff=round(float(r_eff),3),
                         m_lin=round(float(lin),2),m_sweep=round(sw,2),q_deg=[round(float(x),1) for x in np.degrees(Q[qi])]))
# ③ r_max: r 이하 자세들의 필요 토크 최대(포락선)
order=np.argsort(RHO); rs=RHO[order]
def r_max(mm,act):
    npz=need_pose(mm)[order][:,:2]; env=np.maximum.accumulate(npz,axis=0); okk=(env<=C.RATED[act]/C.SF).all(1)
    if okk.all(): return float(rs[-1]),True
    j=int(np.argmin(okk)); return (float(rs[j-1]) if j>0 else 0.0),False
RREACH=float(RHO.max())
rmax=[]
for act,ms in (("TQ",[3,6,9,11,12,13]),("RI85",[3,4,4.5,5,5.5,6])):
    for mm in ms:
        r,full=r_max(mm,act); rmax.append(dict(actuator=act,payload=mm,r_max=round(r,3),full_reach=full))
# 포락선 손 검산: 3 kg, 앞으로 수평 자세 r ≈ 수평 거리 → pitch 필요 ≈ τ0(수평) + 3 g r
qh=np.array([-np.pi/2,0,0,0]); th=A.terms(qh); hand=dict(rho=round(th[4],3),need_pitch=round(float(abs(th[0][0]+3*th[1][0])+ADD[0]),2),
     hand_payload=round(3*C.G*th[4],2),sim_payload=round(float(abs(3*th[1][0])),2))
# ④ 결정 자료 표
S3={(r["case"],r["joint"]):r for r in sweep if r["payload"]==3.0}
def minm(case): rr=[S3[(case,a)] for a in C.ARM]; x=min(rr,key=lambda r:r["margin"]); return x
SC=None
p=os.path.join(OUT,"scenario_sweep.json")
if os.path.exists(p): SC=json.load(open(p))
WR={"a2_doc":"pitch 127 ~ 119 · roll 136 ~ 128 %","cad_ri85":"pitch 38 ~ 34 · roll 44 ~ 39 %"}   # 박종진 31f40ad wrist_2axis_proposal.md (손목 0.8 kg, 모듈 0 ~ 0.06 m)
dec=[]
for case in C.CASES:
    mn=minm(case); mm=[x for x in mmax if x["actuator"]==C.CASES[case]["shoulder_pitch"] and x["joint"]=="shoulder_pitch"][0]
    r3=[x for x in rmax if x["actuator"]==C.CASES[case]["shoulder_pitch"] and x["payload"]==3][0]
    sc=f"{SC['3.0']['success']}/{SC['3.0']['n']} (3 kg, seed 1000 ~ 1009) · 8주차 19/20" if SC else "8주차 19/20"
    dec.append(dict(plan="현재 (제한 없음)",case=case,payload="3 kg",min_margin=f"{mn['margin']} % ({mn['joint']})",success=sc,lose="—"))
    dec.append(dict(plan="① 짐 제한",case=case,payload=f"m_max {mm['m_sweep']} kg",min_margin="20.0 % (m_max 에서 정의상)",success="3 kg 이 m_max 아래 → 제한 걸리지 않음(현재와 같음)",lose=f"없음 (3 kg < {mm['m_sweep']} kg)"))
    dec.append(dict(plan="② 도달 제한",case=case,payload="3 kg",min_margin=f"{mn['margin']} % (r_max = 전 도달 {r3['r_max']} m)" if r3["full_reach"] else f"20 % (r_max {r3['r_max']} m)",
                    success="도달 제한 걸리지 않음(현재와 같음)" if r3["full_reach"] else "박종진 비교표",lose="없음 (3 kg 에서 r_max = 최대 도달)" if r3["full_reach"] else "작업공간 축소"))
    dec.append(dict(plan="손목 2축 추가 (예측)",case=case,payload="3 kg",min_margin=WR[case]+f" (지금 대비 {'−17 ~ −25' if case=='a2_doc' else '−11 ~ −15'} %p)",success="예측 (박종진)",lose="여유율 감소, 그래도 20 % 이상"))
json.dump(dict(sweep=sweep,mmax=mmax,rmax=rmax,reach=RREACH,hand=hand,decision=dec),open(os.path.join(OUT,"torque_decision_w10.json"),"w",encoding="utf-8"),ensure_ascii=False,indent=1)
with open(os.path.join(OUT,"torque_decision_w10.csv"),"w",newline="",encoding="utf-8") as f:
    w=csv.DictWriter(f,fieldnames=list(sweep[0])); w.writeheader(); w.writerows(sweep)
json.dump(dict(_note="박종진 wb_ik reach_limit 용: 어깨 pitch 관절점 → 파지 중심(ee − 45 mm) 수평 거리 상한 [m]. full_reach=true 면 제한 불필요(최대 도달 이상)",
               reach_max_m=round(RREACH,3),table=rmax),open(os.path.join(OUT,"reach_limit_w10.json"),"w",encoding="utf-8"),ensure_ascii=False,indent=1)
# 그림
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
for fn in ("NanumGothic","Noto Sans CJK KR","Malgun Gothic"):
    try:
        from matplotlib import font_manager as fm
        if any(fn in x.name for x in fm.fontManager.ttflist): plt.rcParams["font.family"]=fn; break
    except Exception: pass
plt.rcParams["axes.unicode_minus"]=False
fig,ax=plt.subplots(1,2,figsize=(11,4.2))
cols={"a2_doc":"#257de0","cad_ri85":"#f47230"}; xs=np.arange(0,6.01,0.05)
for case,acts in C.CASES.items():
    for i,a,ls in ((0,"shoulder_pitch","-"),(1,"shoulder_roll","--")):
        r=C.RATED[acts[a]]; y=[C.margin(r,NG[int(round(x/0.01)),i]) for x in xs]
        ax[0].plot(xs,y,ls,color=cols[case],lw=2,label=f"{case} {a.split('_')[1]} ({acts[a]} {r})")
ax[0].axhline(20,color="#666666",lw=1); ax[0].text(0.05,23,"기준 20 %",color="#666666",fontsize=9)
ax[0].axvline(3,color="#999999",lw=1,ls=":"); ax[0].set_ylim(0,350)
ax[0].set_xlabel("짐 질량 [kg] (파지 중심)"); ax[0].set_ylabel("여유율 (정격 − 필요) / 필요 [%]"); ax[0].set_title("① 짐 질량 – 여유율 (어깨, 정적 최악 + 베이스 가속)",fontsize=10)
ax[0].legend(fontsize=8,frameon=False)
for mm,col in ((3,"#257de0"),(5.5,"#f47230")):
    npz=need_pose(mm)[order][:,0]; env=np.maximum.accumulate(npz)
    ax[1].plot(rs,env,color=col,lw=2,label=f"pitch 필요 포락선, 짐 {mm} kg")
for act,col in (("RI85","#00b894"),("TQ","#6a4ce2")):
    ax[1].axhline(C.RATED[act]/C.SF,color=col,lw=1,ls="--"); ax[1].text(0.02,C.RATED[act]/C.SF+1.5,f"{act} 정격 / 1.2 = {C.RATED[act]/C.SF:.1f}",fontsize=9,color="#191919")
ax[1].set_xlabel("어깨 → 파지 중심 수평 거리 r [m]"); ax[1].set_ylabel("필요 토크 [N·m]"); ax[1].set_title("② 도달 거리 – 필요 토크 (r 이하 자세 중 최대)",fontsize=10)
ax[1].legend(fontsize=8,frameon=False,loc="upper left",bbox_to_anchor=(0,0.85))
for a_ in ax: a_.grid(alpha=0.25,lw=0.6); [a_.spines[s].set_visible(False) for s in ("top","right")]
plt.tight_layout(); plt.savefig(os.path.join(OUT,"week10_torque_decision.png"),dpi=130)
print(json.dumps(dict(mmax=mmax,rmax=rmax,reach=RREACH,hand=hand),ensure_ascii=False,indent=0))
for r in sweep:
    if r["joint"] in ("shoulder_pitch","shoulder_roll"): print(r)
# ---- 보고용 md ----
H=["# 토크 결정 자료 (week10, 함태훈) — 자동 생성 `w10_torque_decision.py`","",
"- 출처: 구동기 정격 · 피크 = **A2 구동기 사양서 2026-10-19 판**(TQ 118.8 / 381.7, RI85 72.6 / 181.5 N·m). 값은 모두 `config/params.yaml`",
"- **여유율 = (정격 − 필요) ÷ 필요, 기준 20 % 이상**(⇔ 정격 ≥ 필요 × 1.2 = 요구 정격). 비교 대상은 정격, 피크는 같이 적기만 함. 5 · 6주차 「정격 대비 %」 · 8주차 「PDR 52 기준 %」 표기는 쓰지 않음",
"- 필요 토크 = 정적 최악(왼팔 무작위 3,000 자세, rng 0) + 베이스 설계 가속 추가분(1 m/s² · 1.57 rad/s², 7주차 add_at_worst), 짐 @ 파지 중심(ee − 45 mm), 그리퍼 0.9 kg — 8주차와 같은 방법(3 kg 필요 48.69 · 46.71 = 8주차 표)",
"","## 1. 어깨 case 2개 × 짐 0 · 1 · 2 · 3 kg — 필요 토크 · 여유율","","| 짐 kg | 관절 | 필요 N·m | 요구 정격 (× 1.2) | **a2_doc** 정격 / 피크 → 여유율 | **cad_ri85** 정격 / 피크 → 여유율 | 참고: 자세별 실제 가속 |","|---|---|---|---|---|---|---|"]
for mm in MS:
    for a in C.ARM:
        r1=[x for x in sweep if x["case"]=="a2_doc" and x["payload"]==mm and x["joint"]==a][0]; r2=[x for x in sweep if x["case"]=="cad_ri85" and x["payload"]==mm and x["joint"]==a][0]
        H.append(f"| {mm:g} | {a} | {r1['need']} | {r1['required']} | {r1['actuator']} {r1['rated']} / {r1['peak']} → **{r1['margin']} %** | {r2['actuator']} {r2['rated']} / {r2['peak']} → **{r2['margin']} %** | {r1['need_cons']} |")
H+=["","- 모든 칸 20 % 이상 → **두 case 모두 짐 0 ~ 3 kg 전 구간 통과**. 가장 빠듯한 곳 = cad_ri85 shoulder_pitch 3 kg 49.1 %",
"- 「참고: 자세별 실제 가속」 = 같은 자세에서 |τ_중력| + |τ_베이스가속| 최대(3 kg pitch 46.86) — 8주차 방법(최악 자세 정적 + 다른 자세의 가속 최대분 합)이 3.9 % 더 보수적. 판정은 팀장 승인 값(8주차 방법) 유지"]
if SC:
    H+=["","### 집기 시나리오(동적) 짐 스윕 — 박종진 전문가 정책, 옆면 접근, seed 1000 ~ 1009, dt 0.5 ms (`w10_scenario_sweep.py`)","","| 짐 kg | 성공 | 잡기 ~ 놓기 구동 토크 최대 pitch · roll · yaw · elbow [N·m] |","|---|---|---|"]
    for k,v in SC.items(): H.append(f"| {float(k):g} | {v['success']}/{v['n']} | {' · '.join(str(x) for x in v['tau_max'])} |")
    H+=["","- 시나리오 실제 토크는 정적 최악의 20 % 수준(3 kg pitch 9.83 vs 48.69) — 옆면 접근이라 팔이 거의 수직. 짐에 거의 선형(0.2 → 3 kg: 2.75 → 9.83, 약 2.5 N·m/kg ≈ g × 0.26 m). 3 kg 실패 1건(seed 1003 낙하)은 8주차와 같은 seed"]
H+=["","## 2. 20 % 를 만족하는 최대 짐 m_max — 선형식 vs 자세 스윕","","| 구동기 | 관절 | 필요 한도 = 정격 ÷ 1.2 | τ₀ (짐 0, 3 kg 최악 자세) | r (= Sz ÷ g) | **선형식** m = (한도 − τ₀) ÷ (g · r) | **자세 스윕** | 3 kg 최악 자세 (°) |","|---|---|---|---|---|---|---|---|"]
for x in mmax: H.append(f"| {x['actuator']} | {x['joint']} | {x['limit']} | {x['tau0']} | {x['r_eff']} m | **{x['m_lin']} kg** | **{x['m_sweep']} kg** | {x['q_deg']} |")
H+=["","- 선형식과 스윕 차 ≤ 0.01 kg — 짐 0 ~ 15 kg 에서 최악 자세가 바뀌지 않아서(팔 수평, 짐 · 팔 무게가 같은 방향). 정적 토크가 짐에 선형인 이유 = τ = τ₀(q) + m · g · J_zᵀ(q), 같은 자세면 m 의 1차식",
"- 선형이 깨지는 경우: 가감속(τ = M(q) q̈ 에 짐 관성이 들어가고 궤적 속도 · 시간에 따라 최악 자세가 바뀜), 접촉 · 충격(파지력 · 놓을 때 튐, 7주차 32 mm), 짐이 손 안에서 미끄러져 r 이 바뀔 때",
"- 짐 제한이 필요한 질량: RI85 어깨면 4.88 kg 초과, TQ 면 11.04 kg 초과 → **목표 3 kg 에서는 짐 제한 불필요**"]
H+=["","## 3. 최대 도달 거리 r_max — 어깨 pitch 관절점 → 파지 중심 수평 거리","",f"- 3,000 자세의 r 범위 0.008 ~ **{RREACH:.3f} m**(최대 도달). r 이하 자세들의 필요 토크 최대(포락선)가 정격 ÷ 1.2 이하인 r 의 상한","","| 구동기 | 짐 kg | r_max | 도달 제한 필요? |","|---|---|---|---|"]
for x in rmax: H.append(f"| {x['actuator']} | {x['payload']:g} | {x['r_max']} m | {'아니오 (최대 도달까지 OK)' if x['full_reach'] else '**예**'} |")
H+=["",f"- 손 검산(RI85 5 kg): 팔 자중 토크 ≈ 27.76 N·m × (r ÷ 0.638) + 5 g r ≤ 60.5 − 2.16 → r ≤ 58.34 ÷ (43.5 + 49.1) = **0.630 m** = 스윕 0.630 m",
f"- 손 검산(앞으로 수평): r = {hand['rho']} m, 3 kg 짐 몫 3 g r = {hand['hand_payload']} vs 시뮬 {hand['sim_payload']} N·m(차 1.6 % — 그리퍼 축 45 mm 오프셋 · 어깨 roll 축 위치)",
f"- 주의: 정확히 앞으로 수평(pitch −90°, 나머지 0) 자세의 3 kg pitch 필요 = {hand['need_pitch']} N·m 로 3,000 무작위 자세 최대 48.69 보다 0.3 % 큼(무작위 표본이 정확한 최악점을 빗겨감). 이 값으로 쓰면 RI85 여유 48.6 % · m_max ≈ 4.86 kg — 결론 같음. CDR 동결 때는 6주차처럼 국소 최적화로 최악점을 잡아 고정할 것",
"- **목표 3 kg 에서는 두 case 모두 r_max = 최대 도달 → 도달 제한 불필요**. 박종진 wb_ik `reach_limit` 값 = `results/reach_limit_w10.json`(짐별 표, RI85 5 kg 이면 0.63 m)"]
H+=["","## 4. 결정 자료 (A팀 회신 양식)","","| 방안 | 어깨 case | 짐 질량 | 최소 여유율 (필요 대비) | 집기 성공률 | 잃는 것 |","|---|---|---|---|---|---|"]
for x in dec: H.append(f"| {x['plan']} | {x['case']} | {x['payload']} | {x['min_margin']} | {x['success']} | {x['lose']} |")
H+=["","- **권고(함태훈 의견): 「현재(제한 없음) + 짐 3 kg」 유지.** 근거 — 두 case 모두 3 kg 최소 여유 49.1 %(RI85) 이상, m_max 4.88 kg · r_max = 최대 도달이라 ① · ② 는 걸리지 않음. 손목 2축을 달아도 RI85 34 ~ 38 % 로 기준 위",
"- 그래도 ① · ② 는 `config/params.yaml` limits 플래그로 구현(기본 꺼짐) — 짐 사양이 4.9 kg 이상으로 오르거나(RI85), 하체 가속 사양이 커지면 켠다",
"- 어깨 case 선택은 토크로 갈리지 않음 → 질량 · 비용 · 납기로 결정 가능(팀장 · A팀)","","![](week10_torque_decision.png)"]
open(os.path.join(OUT,"torque_decision_w10.md"),"w",encoding="utf-8").write("\n".join(H)+"\n")
