# 전신(베이스 3 + 팔 4 = 7자유도) 역기구학 모듈 (week07 커리큘럼 화) — 박종진
#   인터페이스 = docs/ik_interface.md §v2 (함태훈 w07 wb_jac.py 제안 시그니처 그대로 → 함태훈 제어기에서 import만 바꾸면 됨)
#     wb_jacobian(m, d, side, include_base=True, rot=False) -> (J[3×n] 또는 (Jp, Jr), dofs[n])   열 = base TX · TY · RZ, pitch · roll · yaw · elbow
#     wb_weights(include_base=True, w_xy=10, w_yaw=10)      -> W[n]   대각 가중치(클수록 덜 움직임), 팔 = 1
#     wdls(J, W, lam=0.03)                                  -> J#[n×m]  가중 DLS  J# = W⁻¹Jᵀ (J W⁻¹ Jᵀ + λ² I)⁻¹
#     limit_avoid(m, d, side, include_base=True, k=1.0)     -> q̇0[n]  영공간 2차 목표: 팔 관절 범위 중앙으로, 베이스 0
#     null_proj(J, W)                                       -> N[n×n]  정확한 가중 영공간 투영 I − J⁺_W J (λ 없음 → 1차 과제를 건드리지 않음)
#     solve_wb_ik(m, d, side, target_pos, target_axis=None, …) -> WBIKResult   반복 해법(위치 / 위치 + 그리퍼 축)
#   q̇ = J# ẋ + N q̇0,   q̇0 = 관절 한계 회피(팔) + 베이스 원위치 복귀(베이스 이동 최소)
#   MuJoCo base slide 축은 world 고정 → 베이스 열(TX, TY)은 world x · y 단위벡터, RZ 열 = ẑ × (p − p_RZ). 로봇 기준 속도 명령 = R(yaw)ᵀ [ẋ, ẏ]
import os,sys
from dataclasses import dataclass,asdict,field
import numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE)
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
BASE=["base_planar_joint_TX","base_planar_joint_TY","base_planar_joint_RZ"]
Z_BAND=(0.606,1.789)   # 팔 끝점 도달 높이(바닥 기준, workspace.md) — 베이스는 높이를 못 바꿈

def dofs(m,side,include_base=True):
    names=(BASE if include_base else [])+[f"{side}_{a}_joint" for a in ARM]
    return np.array([m.jnt_dofadr[m.joint(n).id] for n in names])

def qadr(m,side,include_base=True):
    names=(BASE if include_base else [])+[f"{side}_{a}_joint" for a in ARM]
    return np.array([m.jnt_qposadr[m.joint(n).id] for n in names])

def wb_jacobian(m,d,side,include_base=True,rot=False):
    """끝점 site 자코비안(world). rot=False → (Jp 3×n, dofs), rot=True → ((Jp, Jr), dofs). d는 mj_kinematics · mj_comPos 이후 상태"""
    jp=np.zeros((3,m.nv)); jr=np.zeros((3,m.nv)); mujoco.mj_jacSite(m,d,jp,jr,m.site(f"{side}_ee").id)
    dd=dofs(m,side,include_base); return ((jp[:,dd],jr[:,dd]) if rot else jp[:,dd]),dd

def wb_weights(include_base=True,w_xy=10.0,w_yaw=10.0):
    return np.array(([w_xy,w_xy,w_yaw] if include_base else [])+[1.0]*4,float)

def wdls(J,W,lam=0.03):
    Wi=np.diag(1.0/np.asarray(W,float))
    return Wi@J.T@np.linalg.inv(J@Wi@J.T+lam**2*np.eye(J.shape[0]))

def null_proj(J,W):
    Wi=np.diag(1.0/np.asarray(W,float)); Jp=Wi@J.T@np.linalg.pinv(J@Wi@J.T,rcond=1e-8)
    return np.eye(J.shape[1])-Jp@J

def limit_avoid(m,d,side,include_base=True,k=1.0):
    """H = ½ Σ ((q − q_mid)/(q_max − q_min))² 의 음의 기울기 → 팔을 범위 중앙 쪽으로 (함태훈 wb_jac.py와 같은 정의)"""
    jid=[m.joint(f"{side}_{a}_joint").id for a in ARM]
    q=d.qpos[[m.jnt_qposadr[j] for j in jid]]; lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
    g=-k*(q-(lo+hi)/2)/(hi-lo)**2
    return np.concatenate([np.zeros(3) if include_base else [],g])

@dataclass
class WBIKResult:
    q: np.ndarray            # (4,) 팔 [pitch, roll, yaw, elbow] rad
    base: np.ndarray         # (3,) 베이스 [x, y, yaw] (world, MuJoCo base_planar_joint TX · TY · RZ)
    success: bool
    pos_err: float           # [m]
    rot_err: float           # [rad] (축 미지정 0)
    iters: int
    status: str              # converged | orientation_limit | unreachable_height | base_limit | max_iter
    restarts_used: int = 0
    base_move: float = 0.0   # 시작 대비 베이스 평면 이동 [m]
    base_turn_deg: float = 0.0
    def as_dict(self):
        r=asdict(self); r["q"]=[float(x) for x in self.q]; r["base"]=[float(x) for x in self.base]; return r

def _axis_err(zc,zt):
    e=np.cross(zc,zt); s=np.linalg.norm(e); ang=float(np.arctan2(s,zc@zt))
    return (e/s*ang if s>1e-12 else (np.zeros(3) if zc@zt>0 else np.array([1.0,0,0])*np.pi)),ang

FOOT=(0.288,0.236)   # 상판 반치수 x · y [m] (assumptions_b1.json base.plate_xy 0.576 × 0.472) — 베이스 외곽 근사
def _clip_base(b,box,yaw_range,table_x=None):
    if yaw_range is not None: b[2]=np.clip(b[2],*yaw_range)
    if table_x is not None:   # 테이블(x > table_x, world)을 뚫지 않게: 회전한 상판 사각형의 가장 앞 x ≤ table_x
        b[0]=min(b[0],table_x-(FOOT[0]*abs(np.cos(b[2]))+FOOT[1]*abs(np.sin(b[2]))))
    if box is not None:
        (xl,xh),(yl,yh)=box; b[0]=np.clip(b[0],xl,xh); b[1]=np.clip(b[1],yl,yh)
    return b

def _once(m,dd,side,inc,tp,zt,x0,b0,W,lam,rot_weight,k_limit,k_base,pos_tol,rot_tol,max_iter,step_max,base_step,box,yaw_range,table_x):
    QA=qadr(m,side,inc); jid=[m.joint(f"{side}_{a}_joint").id for a in ARM]; lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
    x=x0.copy(); sid=m.site(f"{side}_ee").id; na=len(x); hist=[]
    for it in range(1,max_iter+1):
        dd.qpos[QA]=x; mujoco.mj_kinematics(m,dd); mujoco.mj_comPos(m,dd)
        p=dd.site_xpos[sid]; ep=tp-p; pe=float(np.linalg.norm(ep)); re=0.0
        (Jp,Jr),_=wb_jacobian(m,dd,side,inc,rot=True)
        if zt is None: J,e=Jp,ep
        else:
            zc=dd.site_xmat[sid].reshape(3,3)[:,2]; er,re=_axis_err(zc,zt); P=np.eye(3)-np.outer(zc,zc)
            J=np.vstack([Jp,rot_weight*(P@Jr)]); e=np.concatenate([ep,rot_weight*(P@er)])
        hist.append((pe,re))
        if pe<=pos_tol and (zt is None or re<=rot_tol): return x,pe,re,it,"converged",hist
        q0=limit_avoid(m,dd,side,inc,k_limit)
        if inc: q0[:3]=-k_base*(x[:3]-b0)   # 베이스 원위치 쪽으로(이동 최소)
        dx=wdls(J,W,lam)@e+null_proj(J,W)@q0
        sa=np.linalg.norm(dx[-4:])/step_max; sb=(np.linalg.norm(dx[:2])/base_step) if inc else 0.0
        sc=max(sa,sb,1.0); x=x+dx/sc
        x[-4:]=np.clip(x[-4:],lo,hi)
        if inc: x[:3]=_clip_base(x[:3],box,yaw_range,table_x)
        if it>40 and abs(hist[-40][0]-pe)<1e-7 and (zt is None or abs(hist[-40][1]-re)<1e-6): break
    return x,pe,re,it,"max_iter",hist

def solve_wb_ik(m,d,side,target_pos,target_axis=None,*,include_base=True,q_init=None,base_init=None,
                w_xy=10.0,w_yaw=10.0,lam=0.05,rot_weight=0.3,k_limit=0.5,k_base=0.05,pos_tol=1e-3,rot_tol=1e-2,
                max_iter=300,step_max=0.3,base_step=0.1,base_box=None,yaw_range=None,table_x=None,restarts=4,seed=0):
    """target_pos (3,) world [m]. target_axis (3,) = 그리퍼 접근 방향(ee site −z가 향할 방향, world), None이면 위치만.
    base_box = ((x_lo, x_hi), (y_lo, y_hi)) world 베이스 원점 허용 범위(주행 공간), yaw_range = (lo, hi) rad,
    table_x = 정면 테이블 앞 가장자리 world x(테이블은 x > table_x) → 상판 사각형(FOOT)이 테이블을 뚫지 않게 베이스 x 제한.
    include_base=False → 팔만(6주차 ik.py와 같은 문제를 같은 반복식으로 — 비교용). d는 복사해서 계산(호출 쪽 상태 그대로)."""
    dd=mujoco.MjData(m); dd.qpos[:]=d.qpos; inc=include_base
    tp=np.asarray(target_pos,float); zt=None
    if target_axis is not None: zt=-np.asarray(target_axis,float); zt/=np.linalg.norm(zt)
    QA=qadr(m,side,True); cur=d.qpos[QA].copy()
    b0=cur[:3].copy() if base_init is None else np.asarray(base_init,float)
    qa0=cur[3:] if q_init is None else np.asarray(q_init,float)
    if not inc: dd.qpos[QA[:3]]=b0
    W=wb_weights(inc,w_xy,w_yaw); rng=np.random.default_rng(seed); best=None
    jid=[m.joint(f"{side}_{a}_joint").id for a in ARM]; lo,hi=m.jnt_range[jid,0],m.jnt_range[jid,1]
    for k in range(restarts+1):
        SEEDS=(-np.pi/2,np.pi/2,-np.pi/4,np.pi/4,np.pi)   # 재시작 베이스 yaw 시작값(옆 · 대각 · 뒤로 붙이기 같은 먼 해 — 국소해 탈출)
        qa=qa0 if (k==0 or (inc and k<=len(SEEDS))) else rng.uniform(lo,hi)
        x0=np.concatenate([b0,qa]) if inc else qa.copy()
        if inc and k>0:
            x0[2]=b0[2]+SEEDS[k-1] if k<=len(SEEDS) else rng.uniform(-np.pi,np.pi)
            if yaw_range is not None: x0[2]=np.clip(x0[2],*yaw_range)
        if inc: x0[:3]=_clip_base(x0[:3].copy(),base_box,yaw_range,table_x)
        r=_once(m,dd,side,inc,tp,zt,x0,(b0 if k==0 else x0[:3].copy()) if inc else b0,W,lam,rot_weight,k_limit,k_base,pos_tol,rot_tol,max_iter,step_max,base_step,base_box,yaw_range,table_x)
        score=r[1]+(0 if zt is None else 0.1*r[2])
        if best is None or score<best[0]: best=(score,r,k)
        if r[4]=="converged": break
    x,pe,re,it,st,_=best[1]
    if st!="converged":
        if pe<=pos_tol and zt is not None: st="orientation_limit"
        elif not (Z_BAND[0]-pos_tol<=tp[2]<=Z_BAND[1]+pos_tol): st="unreachable_height"
        elif inc and (base_box is not None or table_x is not None):
            bx=_clip_base(x[:3].copy(),base_box,yaw_range,table_x); bb=x[:3].copy(); bb[0]+=1e-3; bb2=_clip_base(bb.copy(),base_box,yaw_range,table_x)
            if base_box is not None or abs(bb2[0]-bb[0])>1e-9: st="base_limit"
    base=x[:3] if inc else b0.copy(); q=x[-4:]
    return WBIKResult(q=q,base=base,success=st=="converged",pos_err=pe,rot_err=re,iters=it,status=st,restarts_used=best[2],
                      base_move=float(np.linalg.norm(base[:2]-b0[:2])),base_turn_deg=float(np.rad2deg(base[2]-b0[2])))

def apply(m,d,side,res):
    """해를 d.qpos에 넣고 forward (그대로 position ctrl · 베이스 목표로 사용)"""
    QA=qadr(m,side,True); d.qpos[QA[:3]]=res.base; d.qpos[QA[3:]]=res.q; mujoco.mj_forward(m,d)
