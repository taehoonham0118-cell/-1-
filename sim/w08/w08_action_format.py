# 함태훈 week08 — action 형식 비교: 「관절 목표」(박종진 수집 형식) vs 「끝점 목표」(같은 시연을 끝점 pose 로 바꿔 IK 로 재생)
#  끝점 목표 = 매 주기 관절 목표를 그 순간 베이스 pose 에서 FK → 로봇(베이스) 좌표 위치 3 + 그리퍼 축 ẑ 2 (4-DOF 라 위치 3 + 축 2 = 5조건 > 4 자유도, 최소제곱)
#  재생 IK = 팔 4관절 DLS(λ 0.01), 직전 해에서 시작, 최대 30회. 베이스 · 그리퍼 action 은 그대로
#  출력: results/action_format.{json,md}
import sys,os,json,time,numpy as np,mujoco
PJJ=os.environ.get("PJJ",os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","..","..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P, expert as E, collect as C
HERE=os.path.dirname(os.path.abspath(__file__)); OUT=os.path.join(HERE,"results"); os.makedirs(OUT,exist_ok=True)
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
m=P.build(cam=True); dk=mujoco.MjData(m)
qa=np.array([m.jnt_qposadr[m.joint(f"left_{a}_joint").id] for a in ARM]); da=np.array([m.jnt_dofadr[m.joint(f"left_{a}_joint").id] for a in ARM])
ee=m.site("left_ee").id; W_AX=0.3   # 축 오차 가중(m 대비 rad) — 위치 1 mm ↔ 축 0.19°
def base_T(q):
    c,s=np.cos(q[2]),np.sin(q[2]); return np.array([q[0],q[1],0.0]),np.array([[c,-s,0],[s,c,0],[0,0,1]])
def fk(qfull,qarm):
    dk.qpos[:]=qfull; dk.qpos[qa]=qarm; mujoco.mj_kinematics(m,dk); mujoco.mj_comPos(m,dk)
    return dk.site_xpos[ee].copy(),dk.site_xmat[ee].reshape(3,3)[:,2].copy()
def to_ee_target(qfull,qarm):   # 로봇 좌표 끝점 목표 (pos, z축)
    p,z=fk(qfull,qarm); t,R=base_T(qfull); return R.T@(p-t),R.T@z
def ik(qfull,q0,pb,zb,iters=30,lam=0.01):
    t,R=base_T(qfull); pw=t+R@pb; zw=R@zb; q=q0.copy(); lo=m.jnt_range[[m.joint(f"left_{a}_joint").id for a in ARM]]
    for _ in range(iters):
        p,z=fk(qfull,q); e=np.r_[pw-p,W_AX*(zw-z)]
        if np.linalg.norm(e[:3])<2e-4 and np.linalg.norm(zw-z)<2e-3: break
        jp=np.zeros((3,m.nv)); jr=np.zeros((3,m.nv)); mujoco.mj_jacSite(m,dk,jp,jr,ee)
        Jz=np.array([np.cross(jr[:,k],z) for k in range(m.nv)]).T   # dz/dq = ω_k × z
        J=np.vstack([jp[:,da],W_AX*Jz[:,da]]); dq=J.T@np.linalg.solve(J@J.T+lam**2*np.eye(6),e)
        q=np.clip(q+dq,lo[:,0],lo[:,1])
    p,z=fk(qfull,q); return q,np.linalg.norm(pw-p),np.degrees(np.arccos(np.clip(z@zw,-1,1)))
def replay(seed,A,mode):
    d=mujoco.MjData(m); task=P.sample_task(seed); I=P.reset(m,d,task); qprev=d.qpos[qa].copy(); Q=[]; res=[]; qt=[]
    for t in range(len(A)):
        a=A[t].copy()
        if mode in("ee","ee_cold"):
            pb,zb=to_ee_target(QREC[t],A[t,3:7])     # 기록 시점 베이스 pose 로 만든 끝점 목표(=데이터에 저장될 값)
            if mode=="ee": q,ep,ez=ik(d.qpos.copy(),qprev,pb,zb)
            else: q,ep,ez=ik(d.qpos.copy(),np.zeros(4),pb,zb,iters=150)   # 이력 없이(정책 출력 1개만 보고) 영점에서 시작
            a[3:7]=q; qprev=q; res.append((ep*1e3,ez))
        qt.append(a[3:7]); E.apply_action(m,d,a,I.act)
        for _ in range(E.NSUB): mujoco.mj_step(m,d)
        Q.append(d.qpos.copy())
    p=d.xpos[I.obj[task["kind"]]]; g=np.array([P.TABLE["edge_x"]+P.GOAL["depth"],P.GOAL["y"]]); err=np.linalg.norm(p[:2]-g)
    on=abs(p[2]-(P.TABLE["top"]+P.obj_half_h(task["kind"])))<0.01
    return dict(success=bool(err<=P.GOAL["tol"] and on),goal_err_mm=round(err*1e3,1),Q=np.array(Q),qt=np.array(qt),res=np.array(res) if res else None)
rows=[]; seeds=[int(x) for x in sys.argv[1].split(',')] if len(sys.argv)>1 else list(range(1000,1010))
for s in seeds:
    r,B,task=C.record(m,s); A=np.array(B["action"]); QREC=np.array(B["qpos"])   # 행동 직전 상태(= 데이터 obs.qpos[t])
    t0=time.time(); J=replay(s,A,"joint"); tj=time.time()-t0
    t0=time.time(); X=replay(s,A,"ee"); tx=time.time()-t0
    Xc=replay(s,A,"ee_cold")
    dj=np.abs(J["Q"][-1]-B["final_qpos"]).max()
    dq=np.degrees(np.abs(X["qt"]-A[:,3:7]).max(0)); ee_res=X["res"]
    # 같은 끝점 목표인데 관절 목표가 다른(가지 바뀜) 비율: 0.5° 넘게 다른 주기
    branch=float(np.mean(np.degrees(np.abs(X["qt"]-A[:,3:7]).max(1))>0.5))
    rows.append(dict(seed=s,kind=task["kind"],mass=round(task["mass"],2),expert=bool(r["success"]),joint_success=J["success"],joint_final_diff=float(dj),
                     ee_success=X["success"],ee_goal_err_mm=X["goal_err_mm"],ee_dq_max_deg=[round(float(x),2) for x in dq],ee_frac_dq_gt05=round(branch,3),
                     cold_success=Xc["success"],cold_frac_dq_gt05=round(float(np.mean(np.degrees(np.abs(Xc["qt"]-A[:,3:7]).max(1))>0.5)),3),
                     cold_dq_max_deg=round(float(np.degrees(np.abs(Xc["qt"]-A[:,3:7]).max())),1),cold_pos_res_mm_max=round(float(Xc["res"][:,0].max()),2),
                     ik_pos_res_mm_max=round(float(ee_res[:,0].max()),2),ik_axis_res_deg_max=round(float(ee_res[:,1].max()),2),
                     ms_per_step_joint=round(tj/len(A)*1e3,2),ms_per_step_ee=round(tx/len(A)*1e3,2)))
    print(rows[-1],flush=True)
json.dump(rows,open(os.path.join(OUT,"action_format.json"),"w"),ensure_ascii=False,indent=1)
n=len(rows); L=["# action 형식 비교 — 관절 목표 vs 끝점 목표 (week08, 함태훈) — 자동 생성 `w08_action_format.py`","",
 "같은 전문가 시연(박종진 expert, seed 1000 ~ 1009)을 ① 저장된 관절 목표로 재생 ② 끝점 목표(로봇 좌표 위치 + 그리퍼 축)로 바꿔 팔 IK(DLS)로 재생. 베이스 · 그리퍼 action 은 같음","",
 "| 항목 | 관절 목표 | 끝점 목표 + IK (직전 해에서 시작) | 끝점 목표 + IK (영점에서 시작 = 이력 없음) |","|---|---|---|---|",
 f"| 재생 성공 (전문가 {sum(r['expert'] for r in rows)}/{n}) | {sum(r['joint_success'] for r in rows)}/{n} | {sum(r['ee_success'] for r in rows)}/{n} | {sum(r['cold_success'] for r in rows)}/{n} |",
 f"| 최종 qpos 차 (원본 대비) | 최대 {max(r['joint_final_diff'] for r in rows):.1e} | — | — |",
 f"| 팔 관절 목표 차 최대 [°] | 0 | {max(max(r['ee_dq_max_deg']) for r in rows):.2f} | {max(r['cold_dq_max_deg'] for r in rows):.1f} |",
 f"| 0.5° 넘게 달라진 주기 비율 | 0 | 최대 {max(r['ee_frac_dq_gt05'] for r in rows)*100:.1f} % | 평균 {np.mean([r['cold_frac_dq_gt05'] for r in rows])*100:.1f} % |",
 f"| IK 잔차 위치 · 축 (최대) | — | {max(r['ik_pos_res_mm_max'] for r in rows):.2f} mm · {max(r['ik_axis_res_deg_max'] for r in rows):.2f}° | {max(r['cold_pos_res_mm_max'] for r in rows):.1f} mm |",
 f"| 재생 계산 시간 [ms/주기] | {np.mean([r['ms_per_step_joint'] for r in rows]):.2f} | {np.mean([r['ms_per_step_ee'] for r in rows]):.2f} | — |","",
 "| seed | 물체 · kg | 관절 목표 | 끝점 목표 (목표 오차 mm) | 관절 목표 차 최대 ° (pitch · roll · yaw · elbow) | IK 잔차 mm · ° | 이력 없음: 성공 · 0.5° 초과 주기 |","|---|---|---|---|---|---|---|"]
for r in rows: L.append(f"| {r['seed']} | {r['kind']} {r['mass']} | {'성공' if r['joint_success'] else '실패'} | {'성공' if r['ee_success'] else '실패'} ({r['ee_goal_err_mm']}) | {' · '.join(str(x) for x in r['ee_dq_max_deg'])} | {r['ik_pos_res_mm_max']} · {r['ik_axis_res_deg_max']} | {'성공' if r['cold_success'] else '실패'} · {r['cold_frac_dq_gt05']*100:.1f} % |")
open(os.path.join(OUT,"action_format.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))
