# 함태훈 week08 — 파지 중 토크 예산 갱신: 짐 3 kg(파지 중심 = ee − 45 mm) + 그리퍼 0.9 kg(손바닥 0.5 + 손가락 0.2 × 2)
#  A. 정적 최악: 왼팔 무작위 3,000 자세(관절 한계 안, rng 0) — 7주차 가정(3 kg 을 ee 점에, 손가락 없음) vs 8주차 가정, 같은 자세 집합
#  B. 옆면 접근(yaw ≈ −90°) 집기 실행 중: 박종진 전문가 정책(week08 da55741)으로 3.0 kg 고정 10 에피소드 → 단계별 구동 토크 최대
#  출력: results/grasp_torque.{json,md}, results/torque_budget_w08.csv
import sys,os,json,numpy as np,mujoco
PJJ=os.environ.get("PJJ",os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","..","..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P, expert as E
HERE=os.path.dirname(os.path.abspath(__file__)); OUT=os.path.join(HERE,"results"); os.makedirs(OUT,exist_ok=True)
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]; MPAY=3.0; G=9.81; OFF=P.GRIPPER["grasp_off"]
DTS=float(os.environ.get("DT","0.0005"))   # 접촉 스윕 결론(contact_sweep.md): 3 kg 원기둥은 dt 0.5 ms 에서만 안정
P.OPTION=f'timestep="{DTS}" cone="elliptic" impratio="10"'; E.NSUB=int(round(0.02/DTS))
m=P.build(); d=mujoco.MjData(m)
qa=np.array([m.jnt_qposadr[m.joint(f"left_{a}_joint").id] for a in ARM]); da=np.array([m.jnt_dofadr[m.joint(f"left_{a}_joint").id] for a in ARM])
ee=m.site("left_ee").id; fing=[m.body("left_finger_l").id,m.body("left_finger_r").id]
lo=m.jnt_range[[m.joint(f"left_{a}_joint").id for a in ARM],0]; hi=m.jnt_range[[m.joint(f"left_{a}_joint").id for a in ARM],1]
def park(d):   # 물체 2개는 멀리(바닥), 손가락 접촉 없음
    pass
def tau_static(q,with_fingers,pay_at_grasp):
    d.qpos[:]=m.qpos0; d.qpos[qa]=q; d.qvel[:]=0; mujoco.mj_forward(m,d)
    t=d.qfrc_bias[da].copy()                                    # 로봇 자체 중력(손바닥 0.5 · 손가락 포함)
    if not with_fingers:                                        # 7주차 가정 = 손가락 없음 → 손가락 중력분 빼기
        for b in fing:
            jp=np.zeros((3,m.nv)); mujoco.mj_jacBodyCom(m,d,jp,None,b); t-= (jp.T@np.array([0,0,m.body_mass[b]*G]))[da]
    R=d.site_xmat[ee].reshape(3,3); p=d.site_xpos[ee]-(OFF*R[:,2] if pay_at_grasp else 0)
    jp=np.zeros((3,m.nv)); mujoco.mj_jac(m,d,jp,None,p,m.site_bodyid[ee]); t+=(jp.T@np.array([0,0,MPAY*G]))[da]
    return t
rng=np.random.default_rng(0); Q=lo+(hi-lo)*rng.random((3000,4))
A7=np.array([tau_static(q,False,False) for q in Q]); A8=np.array([tau_static(q,True,True) for q in Q])
# 손 검산: 앞으로 수평(pitch −90°) 자세, 어깨 pitch 중력 토크 증가분 ≈ 손가락 0.4 kg × 수평거리 + 3 kg × 0.045 m(축 방향 오프셋) × g
qh=np.array([-np.pi/2,0,0,0]); t7=tau_static(qh,False,False); t8=tau_static(qh,True,True)
d.qpos[:]=m.qpos0; d.qpos[qa]=qh; mujoco.mj_forward(m,d); sh=d.xpos[m.body("left_shoulder_pitch_link").id]; R=d.site_xmat[ee].reshape(3,3)
xf=np.mean([d.xipos[b] for b in fing],axis=0); hand=dict(fingers=0.4*G*abs(xf[0]-sh[0]),payload_shift=MPAY*G*abs(OFF*R[0,2]))
# B. 시나리오(3.0 kg 고정)
_st=P.sample_task
def task3(seed): t=_st(seed); t["mass"]=MPAY; return t
P.sample_task=task3; E.sample_task=task3
seeds=[1000,1001,1002,1003,1004,1005,1006,1007,1008,1009]; rows=[]; stage_max={}; worst={}
for s in seeds:
    rec={}; wl={}
    def hook(dd,act,ex):
        f=np.abs(dd.actuator_force[[m.actuator(f"left_{a}").id for a in ARM]]); st=ex.stage
        if st in("GRASP","LIFT","MOVE","PLACE"):
            r=rec.setdefault(st,np.zeros(4)); np.maximum(r,f,out=r)
            for i in range(4):
                if f[i]>wl.get(ARM[i],(0,))[0]: wl[ARM[i]]=(float(f[i]),s,st,[round(float(np.degrees(x)),1) for x in dd.qpos[qa]],round(float(np.degrees(dd.qpos[2])),1))
    r,_=E.run_episode(m,s,on_step=hook)
    if not r["success"]:          # 실패(발산 · 낙하) 에피소드의 토크는 물리값이 아니므로 예산에서 제외, 기록만
        rows.append(dict(seed=s,kind=r["kind"],success=False,fail=r["fail"],excluded=True)); continue
    for a,w in wl.items():
        if w[0]>worst.get(a,(0,))[0]: worst[a]=w
    rows.append(dict(seed=s,kind=r["kind"],success=r["success"],fail=r["fail"],**{st:[round(float(x),2) for x in v] for st,v in rec.items()}))
    for st,v in rec.items(): stage_max[st]=np.maximum(stage_max.get(st,np.zeros(4)),v)
P.sample_task=_st; E.sample_task=_st
# 예산표 (7주차 torque_budget_w07.csv 에 열 추가)
import csv
BT=json.load(open(os.environ.get("W07BT",os.path.join(os.path.dirname(HERE),"w07","results","base_torque.json"))))["est"]["design"]
W7=list(csv.DictReader(open(os.environ.get("W07CSV",os.path.join(os.path.dirname(HERE),"w07","results","torque_budget_w07.csv")))))
st7=np.abs(A7).max(0); st8=np.abs(A8).max(0); scen=np.max([stage_max[k] for k in stage_max],axis=0)
TB=[]
for r in W7:
    i=ARM.index(r["joint"]); base_add=float(BT[r["joint"]]["add_at_worst"])     # 7주차 설계 가속(1 m/s² · 1.57 rad/s²) 최악 자세의 추가분을 그대로 더함(자세가 달라도 보수적)
    st_new=float(st8[i]); need=max(float(r["need"]),st_new+base_add,float(scen[i])); rated=float(r["rated"])
    TB.append(dict(case=r["case"],joint=r["joint"],actuator=r["actuator"],rated=rated,peak=float(r["peak"]),need_w07=float(r["need"]),
                   static_w07_model=round(float(st7[i]),2),static_w08=round(st_new,2),base_add=round(base_add,2),scenario_3kg=round(float(scen[i]),2),
                   need_w08=round(need,2),margin=round((rated/need-1)*100,1),ok=rated>=1.2*need,pdr52=round((52/need-1)*100,1) if i<2 else None))
with open(os.path.join(OUT,"torque_budget_w08.csv"),"w",newline="") as f:
    w=csv.DictWriter(f,fieldnames=list(TB[0].keys())); w.writeheader(); w.writerows(TB)
J=dict(payload=MPAY,grasp_off=OFF,gripper_mass=dict(palm=0.5,fingers=0.4),static_w07_assump=dict(zip(ARM,st7.round(2).tolist())),static_w08=dict(zip(ARM,st8.round(2).tolist())),
       horizontal=dict(w07=t7.round(3).tolist(),w08=t8.round(3).tolist(),hand_check=hand),scenario=rows,stage_max={k:v.round(2).tolist() for k,v in stage_max.items()},worst=worst,budget=TB)
json.dump(J,open(os.path.join(OUT,"grasp_torque.json"),"w"),ensure_ascii=False,indent=1)
L=["# 파지 중 토크 예산 갱신 (week08, 함태훈) — 자동 생성 `w08_grasp_torque.py`","",
   f"가정값: 짐 {MPAY} kg 을 파지 중심(ee site − {OFF*1e3:.0f} mm, 그리퍼 축 방향)에, 그리퍼 0.9 kg(손바닥 0.5 = v1 모델 · 손가락 0.2 × 2 = 박종진 pick_env 임시값), 어깨 구동기 case 두 가지, 판정 = 정격 ≥ 필요 × 1.2","",
   "## A. 정적 최악 (왼팔 무작위 3,000 자세, 같은 자세 집합)","","| 관절 | 7주차 가정(3 kg @ ee, 손가락 없음) | 8주차 가정(3 kg @ ee − 45 mm + 손가락 0.4 kg) | 증가 |","|---|---|---|---|"]
for i,a in enumerate(ARM): L.append(f"| {a} | {st7[i]:.2f} | {st8[i]:.2f} | +{st8[i]-st7[i]:.2f} |")
L+=["",f"- 손 검산 (앞으로 수평, pitch): 시뮬 증가 {abs(t8[0])-abs(t7[0]):.2f} N·m ≈ 손가락 0.4 kg × g × 수평거리 {hand['fingers']:.2f} + 3 kg × g × 축 방향 오프셋 {hand['payload_shift']:.2f} = {hand['fingers']+hand['payload_shift']:.2f} N·m",
    "",f"## B. 옆면 접근 집기 실행 중 구동 토크 최대 (3.0 kg 고정, seed 1000 ~ 1009, 박종진 전문가 정책, dt {DTS*1e3:g} ms, 성공 에피소드만)","",
    "| 단계 | pitch | roll | yaw | elbow |","|---|---|---|---|---|"]
for st in("GRASP","LIFT","MOVE","PLACE"):
    if st in stage_max: v=stage_max[st]; L.append(f"| {st} | "+" | ".join(f"{x:.2f}" for x in v)+" |")
L+=["",f"- 성공 {sum(r['success'] for r in rows)}/{len(rows)} (3 kg 고정, 실패 {[ (r['seed'],r['fail']) for r in rows if not r['success']]} 는 제외). 관절별 최대 시점: "+"; ".join(f"{a} {w[0]:.1f} N·m (seed {w[1]} {w[2]}, 팔 {w[3]}°, 베이스 yaw {w[4]}°)" for a,w in worst.items()),
    "","## C. 토크 예산표 (7주차 표 + 8주차 열)","","필요 = max(7주차 필요, 8주차 정적 최악 + 7주차 베이스 설계 가속 추가분(최악 자세 값, 보수적), 집기 실행 최대)","",
    "| case | 관절 | 정격 | 7주차 필요 | 8주차 정적 | + 베이스 가속 | 집기 실행 | **8주차 필요** | 여유 % · 판정 | PDR 52 여유 % |","|---|---|---|---|---|---|---|---|---|---|"]
for r in TB: L.append(f"| {r['case']} | {r['joint']} | {r['actuator']} {r['rated']} | {r['need_w07']:.2f} | {r['static_w08']:.2f} | {r['base_add']:.2f} | {r['scenario_3kg']:.2f} | **{r['need_w08']:.2f}** | {r['margin']} · {'OK' if r['ok'] else 'NG'} | {r['pdr52'] if r['pdr52'] is not None else '—'} |")
open(os.path.join(OUT,"grasp_torque.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))
