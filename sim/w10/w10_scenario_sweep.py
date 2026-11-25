# 함태훈 week10 — 집기 시나리오(박종진 전문가 정책, 옆면 접근) 짐 질량 스윕: 0.2 · 1 · 2 · 3 kg × seed 1000 ~ 1009 (dt 0.5 ms)
#   잡기 ~ 놓기(GRASP · LIFT · MOVE · PLACE) 구간 왼팔 구동 토크 최대 + 성공 여부 → results/scenario_sweep.json
#   3 kg 행 = 8주차 w08_grasp_torque.py B(같은 seed 10개)와 같아야 함(pitch 9.83 · roll 8.69)
#   실행: python sim/w10/w10_scenario_sweep.py   (~8분, PJJ = 박종진 저장소)
import os,sys,json,numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
import w10_common as C
import pick_env as P, expert as E
P.OPTION='timestep="0.0005" cone="elliptic" impratio="10"'; E.NSUB=40
m=P.build(); A=[m.actuator(f"left_{a}").id for a in C.ARM]
_st=P.sample_task; out={}
for mass in (0.2,1.0,2.0,3.0):
    def task(seed,mass=mass): t=_st(seed); t["mass"]=mass; return t
    P.sample_task=task; E.sample_task=task; mx=np.zeros(4); ok=0; rows=[]
    for s in range(1000,1010):
        rec=np.zeros(4)
        def hook(dd,act,ex):
            if ex.stage in ("GRASP","LIFT","MOVE","PLACE"): np.maximum(rec,np.abs(dd.actuator_force[A]),out=rec)
        r,_=E.run_episode(m,s,on_step=hook)
        rows.append(dict(seed=s,success=bool(r["success"]),fail=r["fail"],tau=[round(float(x),2) for x in rec]))
        if r["success"]: ok+=1; np.maximum(mx,rec,out=mx)
        print(mass,s,r["success"],rec.round(2),flush=True)
    out[str(mass)]=dict(success=ok,n=10,tau_max=[round(float(x),2) for x in mx],rows=rows)
P.sample_task=_st
json.dump(out,open(os.path.join(HERE,"results","scenario_sweep.json"),"w"),indent=1)
