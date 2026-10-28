# DLS 역기구학 모듈 (week06 커리큘럼 화) — 박종진
#   인터페이스 = docs/ik_interface.md (함태훈 집기 시나리오와 합의, 10/18 초안 → week06 v1)
#   dq = Jᵀ (J Jᵀ + λ² I)⁻¹ e,  관절 한계 클리핑(매 반복), 스텝 크기 제한, 위치만 / 위치 + 그리퍼 축 / 위치 + 자세 전체
#   4-DOF 팔: 위치 3 + 그리퍼 축 2 = 5조건 > 4자유도 → 최소자승 타협 (rot_weight로 위치 우선)
# 사용 (모듈):  from ik import solve_ik, approach_quat, load_b1
#               m,d = load_b1(); res = solve_ik(m, d, "left", [0.40, 0.25, 1.00], approach_quat([1,0,0]))
# 사용 (스크립트): python ik.py   → 목표 10개 표 · λ 비교 · 수렴률 → results/ik_results.{json,md}, img/ik_targets.png
import os,sys,json,time
from dataclasses import dataclass,asdict
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE)
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]

def load_b1(scene=None):
    sys.path.insert(0,HERE); from v1_mjio import load_model
    m=load_model(scene or os.path.join(V1,"mjcf","scene.xml")); return m,mujoco.MjData(m)

def approach_quat(direction,up_hint=(0,0,1)):
    """그리퍼 접근 방향(ee site −z가 향할 방향, base 기준) → [w,x,y,z]. axis 모드에서는 z축만 쓰므로 up_hint는 full 모드용."""
    zc=-np.asarray(direction,float); zc/=np.linalg.norm(zc)
    h=np.asarray(up_hint,float); h=h-zc*(h@zc)
    if np.linalg.norm(h)<1e-6: h=np.array([1.0,0,0])-zc*zc[0]
    xc=h/np.linalg.norm(h); yc=np.cross(zc,xc); q=np.zeros(4); mujoco.mju_mat2Quat(q,np.column_stack([xc,yc,zc]).ravel()); return q

@dataclass
class IKResult:
    q: np.ndarray          # (4,) [pitch, roll, yaw, elbow] rad — 관절 범위 안 (그대로 position ctrl에 넣으면 됨)
    success: bool
    pos_err: float         # [m]
    rot_err: float         # [rad] (자세 미지정 시 0)
    iters: int
    status: str            # "converged" | "orientation_limit"(위치는 맞춤, 그리퍼 축은 최근접 — 4-DOF) | "max_iter" | "unreachable" | "joint_limit"
    restarts_used: int = 0
    def as_dict(self): r=asdict(self); r["q"]=[float(x) for x in self.q]; return r

_REACH={}
def _arm_info(model,side):
    jid=[model.joint(f"{side}_{a}_joint").id for a in ARM]
    return dict(qadr=np.array([model.jnt_qposadr[j] for j in jid]),dadr=np.array([model.jnt_dofadr[j] for j in jid]),
                lo=model.jnt_range[jid,0].copy(),hi=model.jnt_range[jid,1].copy(),site=model.site(f"{side}_ee").id,
                base=model.body("base_link").id,shoulder=model.body(f"{side}_shoulder_pitch_link").id)

def _rot_err(Rc,quat,mode):
    if mode=="axis":
        zc=Rc[:,2]; zt=np.zeros(9); mujoco.mju_quat2Mat(zt,np.asarray(quat,float)); zt=zt.reshape(3,3)[:,2]
        e=np.cross(zc,zt); s=np.linalg.norm(e); ang=np.arctan2(s,zc@zt)
        return (e/s*ang if s>1e-12 else (np.zeros(3) if zc@zt>0 else np.array([1.0,0,0])*np.pi)),ang,zc
    qc=np.zeros(4); mujoco.mju_mat2Quat(qc,Rc.ravel()); e=np.zeros(3); mujoco.mju_subQuat(e,np.asarray(quat,float),qc)
    # mju_subQuat: 국소 좌표 회전 벡터 → world
    e=Rc@e; return e,float(np.linalg.norm(e)),None

def _solve_once(model,dd,I,tp,tq,q0,rot_mode,rot_weight,damping,pos_tol,rot_tol,max_iter,step_max):
    q=np.clip(q0,I["lo"],I["hi"]); jacp=np.zeros((3,model.nv)); jacr=np.zeros((3,model.nv)); it=0; hist=[]
    for it in range(1,max_iter+1):
        dd.qpos[I["qadr"]]=q; mujoco.mj_kinematics(model,dd); mujoco.mj_comPos(model,dd)
        p=dd.site_xpos[I["site"]]; Rc=dd.site_xmat[I["site"]].reshape(3,3)
        ep=tp-p; pe=float(np.linalg.norm(ep)); re=0.0
        mujoco.mj_jacSite(model,dd,jacp,jacr,I["site"]); Jp=jacp[:,I["dadr"]]
        if tq is None: J,e=Jp,ep
        else:
            er,re,zc=_rot_err(Rc,tq,rot_mode); Jr=jacr[:,I["dadr"]]
            if rot_mode=="axis": P=np.eye(3)-np.outer(zc,zc); Jr=P@Jr; er=P@er   # z축 둘레 회전은 축 방향에 영향 없음 → 투영
            J=np.vstack([Jp,rot_weight*Jr]); e=np.concatenate([ep,rot_weight*er])
        if pe<=pos_tol and (tq is None or re<=rot_tol): hist.append((pe,re,0.0)); return q,pe,re,it,"converged",hist
        dq=J.T@np.linalg.solve(J@J.T+damping**2*np.eye(J.shape[0]),e)
        n=np.linalg.norm(dq); hist.append((pe,re,float(n)))
        if n>step_max: dq*=step_max/n
        q=np.clip(q+dq,I["lo"],I["hi"])
        if it>30 and abs(hist[-30][0]-pe)<1e-7 and (tq is None or abs(hist[-30][1]-re)<1e-6): break   # 정체
    return q,pe,re,it,"max_iter",hist

def solve_ik(model,data,side,target_pos,target_quat=None,*,frame="base",q_init=None,rot_mode="axis",rot_weight=0.3,
             damping=0.05,pos_tol=1e-3,rot_tol=1e-2,max_iter=200,step_max=0.3,restarts=4,seed=0,return_history=False):
    """docs/ik_interface.md 참고. data는 복사본으로 계산(호출 쪽 상태 그대로). 실패 시 최근접 해 + status."""
    I=_arm_info(model,side); dd=mujoco.MjData(model); dd.qpos[:]=data.qpos; mujoco.mj_kinematics(model,dd)
    tp=np.asarray(target_pos,float); tq=None if target_quat is None else np.asarray(target_quat,float)
    if frame=="base":   # base_link 기준 → world (베이스 이동과 무관하게 같은 목표)
        Rb=dd.xmat[I["base"]].reshape(3,3); pb=dd.xpos[I["base"]].copy(); tp=pb+Rb@tp
        if tq is not None: qb=dd.xquat[I["base"]].copy(); out=np.zeros(4); mujoco.mju_mulQuat(out,qb,tq); tq=out
    q0=data.qpos[I["qadr"]].copy() if q_init is None else np.asarray(q_init,float)
    rng=np.random.default_rng(seed); best=None
    for k in range(restarts+1):
        qs=q0 if k==0 else rng.uniform(I["lo"],I["hi"])
        r=_solve_once(model,dd,I,tp,tq,qs,rot_mode,rot_weight,damping,pos_tol,rot_tol,max_iter,step_max)
        score=r[1]+(0 if tq is None else 0.1*r[2])
        if best is None or score<best[0]: best=(score,r,k)
        if r[4]=="converged": break
    q,pe,re,it,st,hist=best[1]
    if st!="converged" and tq is not None:   # 위치 + 축 동시 불가(4-DOF) → 위치 우선 해: ① 가중해 근처에서 위치만 IK ② 위치 등식 제약 하 축 오차 최소(SLSQP, 관절 범위)
        rp=_solve_once(model,dd,I,tp,None,q,rot_mode,rot_weight,damping,pos_tol,rot_tol,max_iter,step_max)
        if rp[1]<=pos_tol:
            from scipy.optimize import minimize
            def fk(x): dd.qpos[I["qadr"]]=x; mujoco.mj_kinematics(model,dd); return dd.site_xpos[I["site"]].copy(),dd.site_xmat[I["site"]].reshape(3,3).copy()
            obj=lambda x: _rot_err(fk(x)[1],tq,rot_mode)[1]**2
            o=minimize(obj,rp[0],method="SLSQP",bounds=list(zip(I["lo"],I["hi"])),constraints=[{"type":"eq","fun":lambda x: fk(x)[0]-tp}],options=dict(maxiter=200,ftol=1e-12))
            qx=np.clip(o.x,I["lo"],I["hi"]); px,Rx=fk(qx); pe_x=float(np.linalg.norm(px-tp)); re_x=float(_rot_err(Rx,tq,rot_mode)[1])
            if pe_x>pos_tol: qx,pe_x,re_x=rp[0],rp[1],float(_rot_err(fk(rp[0])[1],tq,rot_mode)[1])
            q,pe,re=qx,pe_x,re_x; it+=rp[3]+int(o.nit); hist=hist+rp[5]
            st="converged" if re<=rot_tol else "orientation_limit"
    if st not in ("converged","orientation_limit"):   # 실패 원인 분류: 도달 반경 밖 → unreachable, 관절이 한계에 붙어 있으면 joint_limit, 그 외 max_iter
        dd.qpos[I["qadr"]]=0; mujoco.mj_kinematics(model,dd); sh=dd.xpos[I["shoulder"]].copy()
        rmax=_REACH.setdefault((id(model),side),_max_reach(model,I))
        if np.linalg.norm(tp-sh)>rmax+pos_tol: st="unreachable"
        elif np.any(np.isclose(q,I["lo"],atol=1e-4)|np.isclose(q,I["hi"],atol=1e-4)) and pe>pos_tol: st="joint_limit"
    res=IKResult(q=q,success=st=="converged",pos_err=pe,rot_err=re,iters=it,status=st,restarts_used=best[2])
    return (res,hist) if return_history else res

def _max_reach(model,I,n=4000):
    dd=mujoco.MjData(model); rng=np.random.default_rng(1); r=0.0
    dd.qpos[:]=0; mujoco.mj_kinematics(model,dd); sh=dd.xpos[I["shoulder"]].copy()
    for q in rng.uniform(I["lo"],I["hi"],size=(n,4)):
        dd.qpos[I["qadr"]]=q; mujoco.mj_kinematics(model,dd); r=max(r,float(np.linalg.norm(dd.site_xpos[I["site"]]-sh)))
    return r

# ---------------------------------------------------------------- 스크립트: 목표 10개 · λ 비교 · 수렴률
if __name__=="__main__":
    m,d=load_b1(); t0=time.time()
    W=json.load(open(os.path.join(V1,"results","workspace.json"),encoding="utf-8"))
    fwd=approach_quat([1,0,0]); down=approach_quat([0,0,-1])
    # 「아래로 접근」이 실현되는 집기점: 4-DOF(손목 없음)라 축을 고정하면 팔꿈치 위치가 정해짐 → 실현 가능한 점을 FK로 찾음
    #   (pitch, roll=0, yaw=0, elbow)에서 전완이 수직 아래가 되는 elbow = −pitch − π/2 … 여기서는 수치로: 상판 위 0.45 m(z 0.85) 근처에서 축 오차 최소 자세
    I_=_arm_info(m,"left"); dd_=mujoco.MjData(m); bestd=None
    from scipy.optimize import minimize_scalar
    def _fk(q_): dd_.qpos[:]=0; dd_.qpos[I_["qadr"]]=q_; mujoco.mj_kinematics(m,dd_); return dd_.site_xpos[I_["site"]].copy(),dd_.site_xmat[I_["site"]].reshape(3,3)[:,2].copy()
    for pt in np.deg2rad(np.arange(-90,0.1,0.5)):   # roll = yaw = 0 평면에서 pitch마다 전완이 수직 아래가 되는 elbow (1차원 최소화)
        o=minimize_scalar(lambda el: 1-_fk(np.array([pt,0,0,el]))[1][2],bounds=(I_["lo"][3],I_["hi"][3]),method="bounded",options=dict(xatol=1e-10))
        p_,z_=_fk(np.array([pt,0,0,o.x]))
        if z_[2]>1-1e-9 and p_[0]>0.15 and (bestd is None or abs(p_[2]-0.75)<abs(bestd[2]-0.75)): bestd=p_
    pick=[round(float(x),4) for x in bestd]
    p2_,_=_fk(np.deg2rad([-60,20,30,50])); q2=np.zeros(4); mujoco.mju_mat2Quat(q2,dd_.site_xmat[I_["site"]].copy()); p2=[round(float(x),4) for x in p2_]   # 위치 반올림 0.05 mm → 축 허용 오차 안
    # 목표 (왼팔, base 기준). 작업공간 안 8 (workspace.md 범위에서 고름) + 밖 2
    T=[("T1 정면 어깨 높이","left",[0.45,0.25,1.15],None,"in"),
       ("T2 가슴 높이 · 실현 가능 축 (FK 생성)","left",p2,q2,"in"),
       (f"T3 상판 위 집기점 · 아래로 접근","left",pick,down,"in"),
       ("T4 측면 바깥","left",[0.05,0.70,1.10],None,"in"),
       ("T5 머리 위","left",[0.20,0.25,1.65],None,"in"),
       ("T6 정중면 근처 (공유 작업공간)","left",[0.35,0.02,1.10],None,"in"),
       ("T7 거의 완전히 편 팔 (특이점 근처)","left",[0.590,0.28,1.20],None,"in"),
       ("T8 몸 뒤쪽 아래","left",[-0.30,0.35,0.85],None,"in"),
       ("T9 도달 반경 밖 (정면 0.80 m)","left",[0.80,0.25,1.20],None,"out"),
       ("T10 상판 윗면 (z 0.42 m)","left",[0.20,0.20,0.42],None,"out")]
    # 4-DOF 자세 한계 사례 (위치는 안, 축은 실현 불가) — 위치 우선 해로 대응
    TX=[("X0 정면 어깨 높이 · 앞으로 접근","left",[0.55,0.28,1.20],fwd,"in"),("X1 가슴 높이 · 앞으로 접근","left",[0.40,0.22,1.00],fwd,"in"),("X2 상판 위 0.45 m · 아래로 접근","left",[0.30,0.25,0.85],down,"in")]
    rows=[]
    for name,side,p,qt,kind in T+TX:
        d.qpos[:]=0; r=solve_ik(m,d,side,p,qt)
        rows.append(dict(target=name,kind=kind,pos=p,orient="그리퍼 축" if qt is not None else "위치만",**r.as_dict()))
    # λ 비교 (같은 10개, 재시작 없음 → λ 효과만) + 특이점 근처 거동(T7 · T9: 반복별 오차 · 관절 속도 크기)
    LAM=[0.005,0.02,0.05,0.1,0.3]; lam={}
    for l in LAM:
        rr=[]
        for name,side,p,qt,kind in T:
            d.qpos[:]=0; r,h=solve_ik(m,d,side,p,qt,damping=l,restarts=0,return_history=True)
            stp=[x[2] for x in h if len(x)>2]; tail=[x[0] for x in h[-20:]]
            rr.append(dict(t=name.split()[0],ok=r.success,it=r.iters,pe=r.pos_err,max_raw_step=float(max(stp) if stp else 0),tail_osc_mm=float((max(tail)-min(tail))*1000)))
        lam[str(l)]=rr
    # 수렴률: FK로 만든 도달 가능 목표 500개 (위치만 / 위치 + 그리퍼 축) — 자세 목표는 FK 자세의 z축 그대로 → 해가 반드시 존재
    rng=np.random.default_rng(66); I=_arm_info(m,"left"); conv={}
    for mode in ("pos","axis"):
        ok=0; its=[]; pes=[]; rst=0; dd=mujoco.MjData(m)
        for k in range(500):
            qt_=rng.uniform(I["lo"],I["hi"]); dd.qpos[:]=0; dd.qpos[I["qadr"]]=qt_; mujoco.mj_kinematics(m,dd)
            p=dd.site_xpos[I["site"]].copy(); qq=np.zeros(4); mujoco.mju_mat2Quat(qq,dd.site_xmat[I["site"]].copy())
            d.qpos[:]=0; r=solve_ik(m,d,"left",p,qq if mode=="axis" else None,seed=k)
            ok+=r.success; its.append(r.iters); pes.append(r.pos_err); rst+=r.restarts_used>0
        conv[mode]=dict(n=500,success_pct=ok/5,median_iters=float(np.median(its)),p95_pos_err_mm=float(np.percentile(pes,95)*1000),max_pos_err_mm=float(np.max(pes)*1000),needed_restart=rst)
    # 오른팔 거울 확인: 왼팔 T1~T8 거울 목표 → 같은 관절각(안 A 거울 = 같은 명령)
    mir=[]
    for name,side,p,qt,kind in T[:8]:
        d.qpos[:]=0; rl=solve_ik(m,d,"left",p,None,restarts=0); d.qpos[:]=0; rr_=solve_ik(m,d,"right",[p[0],-p[1],p[2]],None,restarts=0)
        mir.append(float(np.abs(rl.q-rr_.q).max()))
    R=dict(pick_point=pick,targets=rows,lambda_study=lam,convergence=conv,mirror_max_dq_rad=max(mir),defaults=dict(damping=0.05,rot_weight=0.3,pos_tol=1e-3,rot_tol=1e-2,max_iter=200,step_max=0.3,restarts=4),elapsed_s=round(time.time()-t0,1))
    json.dump(R,open(os.path.join(V1,"results","ik_results.json"),"w",encoding="utf-8"),indent=1,ensure_ascii=False,default=float)
    # md
    st_ko={"orientation_limit":"위치 수렴 · 축 한계","converged":"수렴","max_iter":"반복 한도","unreachable":"도달 불가(최근접 해)","joint_limit":"관절 한계"}
    L=["# IK 결과 (자동 생성 — ik.py, DLS λ = 0.05, 재시작 ≤ 4)","","왼팔, 목표 = base_link 기준 ee site(그리퍼 끝). 허용 오차 위치 1 mm · 그리퍼 축 0.01 rad. 「그리퍼 축」= ee site −z(접근 방향)를 지정 방향에 맞춤(4-DOF라 자세 전체 대신 축 2조건)","",
       "| 목표 | 구분 | 위치 [m] | 자세 | 결과 | 반복 | 재시작 | 위치 오차 [mm] | 축 오차 [°] | 해 q [°] |","|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        if r["target"].startswith("X0"): L+=["| *4-DOF 자세 한계 사례* | | | | | | | | | |"]
        L.append(f'| {r["target"]} | {"안" if r["kind"]=="in" else "**밖**"} | {r["pos"]} | {r["orient"]} | {st_ko[r["status"]]} | {r["iters"]} | {r["restarts_used"]} | {r["pos_err"]*1000:.3f} | {np.rad2deg(r["rot_err"]):.2f} | {[round(float(np.rad2deg(x)),1) for x in r["q"]]} |')
    R10=rows[:10]; nin=sum(r["kind"]=="in" for r in R10); okin=sum(r["success"] for r in R10 if r["kind"]=="in")
    L+=["",f"- 목표 10개: 작업공간 안 **{okin}/{nin} 수렴**, 밖 2개는 success = False · status = unreachable로 실패를 숨기지 않고 최근접 해 반환(T9 = 팔을 목표 쪽으로 완전히 폄, 남은 거리 = 목표 − 최대 도달 / T10 = 최저 도달 높이 z 0.606 m에서 멈춤)",
        f"- T3 집기점 {pick} = 「아래로 접근」이 실현되는 점을 FK로 찾은 것. 손목이 없어(4-DOF) 그리퍼 축을 고정하면 팔꿈치 위치가 정해지므로, 축 + 위치를 동시에 만족하는 점은 2차원 곡면뿐 → X0~X2처럼 임의 점 + 임의 축은 불가(앞으로 접근은 팔을 다 편 x ≈ 0.595 m에서만). 이때 status = orientation_limit: 위치는 1 mm 이내로 맞추고 축은 영공간(1차원)에서 최근접",
        "","## λ(감쇠) 비교 — 같은 목표, 재시작 없음","","| λ | "+" | ".join(x["t"] for x in lam[str(LAM[0])])+" | 안 수렴 | 원 스텝 최대 T7 / T9 [rad] | T9 진동 [mm] |","|---|"+"---|"*len(T)+"---|---|---|"]
    for l in LAM:
        L.append(f"| {l} | "+" | ".join((f'{x["it"]}' if x["ok"] else f'✗ {x["pe"]*1000:.0f} mm') for x in lam[str(l)])+f' | {sum(x["ok"] for x in lam[str(l)][:8])}/8 | {lam[str(l)][6]["max_raw_step"]:.2f} / {lam[str(l)][8]["max_raw_step"]:.2f} | {lam[str(l)][8]["tail_osc_mm"]:.2f} |')
    L+=["","- 칸 = 수렴까지 반복 횟수, ✗ = 미수렴(최종 위치 오차). 원 스텝 = 스텝 제한(0.3 rad) 전 ‖dq‖ 최대 — 특이점(팔을 다 편 T7 · T9)에서 λ가 작으면 커짐. T9 진동 = 마지막 20회 위치 오차 변동폭",
        "","## 수렴률 (FK로 만든 도달 가능 목표 500개, 무작위 시작 재시작 ≤ 4)","","| 모드 | 자코비안 | 수렴률 | 반복 중앙값 | 위치 오차 95 % / 최대 [mm] | 재시작 필요 |","|---|---|---|---|---|---|",
        f'| 위치만 | 3 × 4 (여유 1 → 해 무한, 영공간 1차원) | **{conv["pos"]["success_pct"]:.1f} %** | {conv["pos"]["median_iters"]:.0f} | {conv["pos"]["p95_pos_err_mm"]:.3f} / {conv["pos"]["max_pos_err_mm"]:.3f} | {conv["pos"]["needed_restart"]} |',
        f'| 위치 + 그리퍼 축 | 5 × 4 (과결정 → 일반적으로 해 0, 도달 가능 자세면 유한 개) | **{conv["axis"]["success_pct"]:.1f} %** | {conv["axis"]["median_iters"]:.0f} | {conv["axis"]["p95_pos_err_mm"]:.3f} / {conv["axis"]["max_pos_err_mm"]:.3f} | {conv["axis"]["needed_restart"]} |',
        "",f'- 오른팔: 왼팔 T1~T8의 y 반전 목표 → 관절각 차 최대 {R["mirror_max_dq_rad"]:.1e} rad (안 A 거울 = 같은 명령 확인)',f'- 실행 시간 {R["elapsed_s"]} s']
    open(os.path.join(V1,"results","ik_results.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))
