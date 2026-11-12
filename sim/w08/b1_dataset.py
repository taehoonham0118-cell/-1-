# 함태훈 week08 — B1 시연 데이터 형식 v1.0 (docs/dataset_spec.md) 검사 · 변환 도구
#   check(path)            → v1.0 규칙 위반 목록 (빈 목록 = 통과)
#   upgrade(src, dst)      → 박종진 초안 v0.1(b1-w08-draft-0.1) 파일을 v1.0 으로 변환 (action · obs 원본은 그대로 복사, 파생 필드만 추가)
#   body_vel(v_world, yaw) → 베이스 속도 world → 로봇 좌표
#   실행: python b1_dataset.py check <파일|폴더>    /    python b1_dataset.py upgrade <src폴더> <dst폴더>
import sys,os,glob,json
import numpy as np,h5py

SPEC="b1-dataset-1.0"
ACTION_NAMES=["base_vx","base_vy","base_wz","left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw","left_elbow","gripper_opening"]
ACTION_BODY_NAMES=["base_vx_body","base_vy_body","base_wz"]
# (경로, 열 수(None = 1차원), dtype 종류, 필수)
FIELDS=[("obs/qpos",27,"f8",True),("obs/qvel",25,"f4",True),("obs/base_pose",3,"f4",True),
        ("obs/ee_pos",3,"f4",True),("obs/ee_quat",4,"f4",True),("obs/ee_pos_base",3,"f4",True),
        ("obs/obj_pos",3,"f4",True),("obs/obj_quat",4,"f4",True),("obs/obj_pos_base",3,"f4",True),
        ("obs/gripper_width",None,"f4",True),("obs/stage",None,"i1",True),
        ("action",8,"f8",True),("action_base_body",3,"f4",True),("ctrl",13,"f8",True),("time",None,"f8",True)]
ATTRS=["spec","seed","kind","mass","obj_xy","obj_yaw","success","fail","fail_stage","goal_err_mm","t_total","control_hz","sim_dt",
       "model_hash","mujoco_version","action_names","action_body_names","stage_names","params","assumptions","events","source"]

def rotz(yaw):
    c,s=np.cos(yaw),np.sin(yaw); return np.array([[c,-s,0],[s,c,0],[0,0,1]])

def body_vel(v_world,yaw):
    """베이스 속도 [vx, vy, wz] world → 로봇 좌표 (wz 는 그대로). v_world (T,3), yaw (T,)"""
    v=np.asarray(v_world,float); c,s=np.cos(yaw),np.sin(yaw)
    return np.stack([c*v[:,0]+s*v[:,1],-s*v[:,0]+c*v[:,1],v[:,2]],1)

def to_base(p_world,base_pose):
    """world 점 (T,3) → 로봇 좌표 (베이스 x · y · yaw, 높이는 그대로)"""
    p=np.asarray(p_world,float).copy(); bp=np.asarray(base_pose,float)
    d=p[:,:2]-bp[:,:2]; c,s=np.cos(bp[:,2]),np.sin(bp[:,2])
    p[:,0]=c*d[:,0]+s*d[:,1]; p[:,1]=-s*d[:,0]+c*d[:,1]; return p

def check(path):
    bad=[]
    with h5py.File(path,"r") as f:
        T=len(f["action"]) if "action" in f else None
        for k,n,dt,req in FIELDS:
            if k not in f:
                if req: bad.append(f"없음: {k}")
                continue
            x=f[k]
            if x.shape[0]!=T: bad.append(f"길이 불일치: {k} {x.shape[0]} ≠ T {T}")
            if n is not None and (x.ndim!=2 or x.shape[1]!=n): bad.append(f"모양: {k} {x.shape} (열 {n} 이어야 함)")
            if n is None and x.ndim!=1: bad.append(f"모양: {k} {x.shape} (1차원이어야 함)")
            if x.dtype.str[1:]!=dt: bad.append(f"형: {k} {x.dtype} ({dt} 이어야 함)")
            if x.dtype.kind=="f" and not np.all(np.isfinite(x[:])): bad.append(f"NaN/Inf: {k}")
        if "final_qpos" not in f or f["final_qpos"].shape!=(27,): bad.append("없음/모양: final_qpos (27,)")
        if "obs/images/overview" in f:
            im=f["obs/images/overview"]
            if im.dtype!=np.uint8 or im.ndim!=4 or im.shape[0]!=T or im.shape[3]!=3: bad.append(f"모양: obs/images/overview {im.shape} {im.dtype}")
        for a in ATTRS:
            if a not in f.attrs: bad.append(f"속성 없음: {a}")
        if f.attrs.get("spec")!=SPEC: bad.append(f"spec = {f.attrs.get('spec')} (≠ {SPEC})")
        if "action_names" in f.attrs and json.loads(f.attrs["action_names"])!=ACTION_NAMES: bad.append("action_names 순서 다름")
        if bool(f.attrs.get("success",False)) and f.attrs.get("fail","")!="": bad.append("성공인데 fail 이 빈 문자열이 아님")
        if not bad:   # 값 규칙: 시간 간격 · 범위 · 파생 필드 일치
            t=f["time"][:]; dt=np.diff(t)
            if not np.allclose(dt,1/f.attrs["control_hz"],atol=1e-9): bad.append("time 간격 ≠ 1/control_hz")
            g=f["action"][:,7]
            if g.min()<-1e-9 or g.max()>0.055+1e-9: bad.append("gripper_opening 범위 밖 (0 ~ 0.055)")
            q=f["obs/qpos"][:]
            if np.abs(f["obs/base_pose"][:]-q[:,:3]).max()>1e-5: bad.append("base_pose ≠ qpos[:, 0:3]")
            ab=body_vel(f["action"][:,:3],q[:,2])
            if np.abs(f["action_base_body"][:]-ab).max()>1e-5: bad.append("action_base_body ≠ R(yaw)ᵀ action[:, 0:3]")
            if np.abs(f["obs/ee_pos_base"][:]-to_base(f["obs/ee_pos"][:],q[:,:3])).max()>1e-5: bad.append("ee_pos_base 불일치")
            nq=np.linalg.norm(f["obs/ee_quat"][:],axis=1)
            if np.abs(nq-1).max()>1e-4: bad.append("ee_quat 정규화 안 됨")
    return bad

def upgrade(src,dst,mujoco_version="3.15.0",source="PJJ0337/-1- week08 da55741 scenarios/collect.py"):
    """초안 v0.1 → v1.0. 원본 배열은 그대로(재생 결과 불변), 파생 필드 · 속성만 추가"""
    with h5py.File(src,"r") as s, h5py.File(dst,"w") as d:
        for k,v in s.attrs.items(): d.attrs[k]=v
        def cp(name,obj):
            if isinstance(obj,h5py.Dataset): s.copy(obj,d,name=name)
        s.visititems(cp)
        q=s["obs/qpos"][:]; bp=q[:,:3]; kw=dict(compression="gzip",compression_opts=4)
        d.create_dataset("obs/base_pose",data=bp.astype(np.float32),**kw)
        d.create_dataset("obs/ee_pos_base",data=to_base(s["obs/ee_pos"][:],bp).astype(np.float32),**kw)
        d.create_dataset("obs/obj_pos_base",data=to_base(s["obs/obj_pos"][:],bp).astype(np.float32),**kw)
        d.create_dataset("action_base_body",data=body_vel(s["action"][:,:3],bp[:,2]).astype(np.float32),**kw)
        d.attrs["spec"]=SPEC; d.attrs["mujoco_version"]=mujoco_version; d.attrs["source"]=source
        d.attrs["action_body_names"]=json.dumps(ACTION_BODY_NAMES)
        if bool(s.attrs["success"]): d.attrs["fail"]=""; d.attrs["fail_stage"]=""
        d.attrs["assumptions"]=json.dumps(["gripper","contact","table","obj","shoulder_actuator=a2_doc"])

if __name__=="__main__":
    cmd=sys.argv[1]
    if cmd=="check":
        P=sorted(glob.glob(os.path.join(sys.argv[2],"*.hdf5"))) if os.path.isdir(sys.argv[2]) else [sys.argv[2]]
        for p in P:
            b=check(p); print(os.path.basename(p),"통과" if not b else f"위반 {len(b)}개: "+"; ".join(b))
    elif cmd=="upgrade":
        os.makedirs(sys.argv[3],exist_ok=True)
        for p in sorted(glob.glob(os.path.join(sys.argv[2],"*.hdf5"))):
            q=os.path.join(sys.argv[3],os.path.basename(p)); upgrade(p,q); print(os.path.basename(p),"→",q)
