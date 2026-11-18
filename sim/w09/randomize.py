# 함태훈 week09 — 도메인 랜덤화 + 민감도 분석 (박종진 집기 시나리오 552614e 그대로: 전문가 정책 · 성공 판정 · dt 0.5 ms)
#   파라미터 = 가정값 표 v0.4 (S3 · S4 · S6 · S10 · S11 · S15 · S16 · S17) + 제어 지연 · 물체 위치 측정 오차 · 수치 설정(dt · solref · impratio)
#   ① 하나씩(OAT): 각 파라미터를 범위 양 끝으로, 나머지 공칭 → 같은 seed 로 성공률 비교 → 민감도 = 공칭 − min(하한, 상한)
#   ② 동시: 물리 파라미터 전부를 범위 안에서 에피소드마다 균등 추출 (좁게 ×0.5 · 기준 ×1 · 넓게 ×2)
#   실행:  python sim/w09/randomize.py oat  [seed수=40]        (~30 분, CPU 2개)
#          python sim/w09/randomize.py joint [seed수=100]       (~10 분)
#          python sim/w09/randomize.py curve name1,name2,…  [seed수=40]   (상위 파라미터 5단계)
#   PJJ 환경변수 = 박종진 저장소 경로(기본: 이 저장소 옆 ../PJJ0337, 552614e)
import sys,os,json,time,copy,numpy as np,mujoco
from multiprocessing import Pool
HERE=os.path.dirname(os.path.abspath(__file__))
PJJ=os.environ.get("PJJ",os.path.join(HERE,"..","..","..","PJJ0337")); sys.path.insert(0,os.path.join(PJJ,"scenarios"))
import pick_env as P, expert as E
TOP0=P.TABLE["top"]
ARM_B=[f"{s}_{b}" for s in ("left","right") for b in ("shoulder_pitch_link","shoulder_roll_link","shoulder_yaw_link","elbow_link","tool0","gripper_link")]
ARM_J=[f"{s}_{a}_joint" for s in ("left","right") for a in ("shoulder_pitch","shoulder_roll","shoulder_yaw","elbow")]
BASE_B=["base_link","base_plate_link","torso_link"]

# ---- 파라미터 · 범위 · 근거 (공칭, 하한, 상한, 묶음, 근거) ----
PARAMS={
 "mu_pad_obj":   (1.0, 0.5, 1.2, "접촉", "S16 손가락 · 물체 μ 1.0 가정(측정 전). 3 kg · 40 N 미끄럼 한계 μ 0.37(8주차)"),
 "grip_force":   (40.0,28.0,48.0,"그리퍼","S15 손가락당 40 N 가정(A팀 사양 미수령) −30 / +20 %"),
 "finger_mass":  (0.2, 0.1, 0.3, "그리퍼","S15 손가락 0.2 kg(0.05 kg 은 발산 → 0.2 로 정함)"),
 "obj_mass_x":   (1.0, 0.8, 1.2, "물체",  "S17 0.2 ~ 3 kg 의 ±20 % (상한 3.6 kg = 토크 예산 3 kg 초과 시험)"),
 "table_dz":     (0.0,-0.010,0.010,"환경","S17 테이블 0.70 m 가정 ±10 mm (정책은 0.70 m 로 믿고 하강)"),
 "obj_bias":     (0.0, 0.0, 0.010,"센서", "물체 위치 측정 오차(카메라 보정) 0 ~ 10 mm, 방향 무작위 — 지금은 참값 관측"),
 "latency":      (0.0, 0.0, 0.020,"제어", "명령 지연 0 ~ 20 ms(= 제어 1주기 50 Hz)"),
 "arm_mass_x":   (1.0, 0.9, 1.1, "질량",  "S6 팔 링크 CAD 밀도 가정 ±10 % (중력보상은 공칭 질량 기준)"),
 "base_mass_x":  (1.0, 0.8, 1.2, "질량",  "S10 · S11 하체 30 kg · 몸통 8 kg 역산값 ±20 %"),
 "armature_x":   (1.0, 0.8, 1.5, "구동기","S3 A2 로터만(어깨 0.219 · 팔꿈치 0.098), 기어열 미포함 → +50 %"),
 "damping_x":    (1.0, 0.5, 2.0, "구동기","S4 0.5 N·m·s/rad 안정화용 가정 ×0.5 ~ 2"),
 "frictionloss": (0.0, 0.0, 2.0, "구동기","S4 0 가정 → 0 ~ 2 N·m (RI85 정격 118.8 의 1.7 %)"),
 "kp_x":         (1.0, 0.9, 1.1, "구동기","토크 상수 · 전류 루프 오차 ±10 % (kp · kv 함께)"),
 "base_kv_x":    (1.0, 0.8, 1.2, "베이스","S12 베이스 kv 200 · 50, 바퀴 미끄럼 · 모터 ±20 %"),
 "dt":           (0.0005,0.00025,0.001,"수치","S16 dt 0.5 ms(8주차 1 ms 발산)"),
 "solref_tc":    (0.01,0.005,0.02,"수치",  "S16 solref 0.01 (8주차 스윕 0.005 ~ 0.04)"),
 "impratio":     (10.0,3.0,10.0,"수치",   "S16 impratio 10 (8주차 1 · 3 에서 발산)"),
}
CURVE={"table_dz":[0,0.004,0.008,0.010,0.012,0.014,0.016,0.020],          # 상위 3개 세부 단계(oat2 에서 깨지는 구간)
       "mu_pad_obj":[0.1,0.2,0.3,0.4,0.5,0.7,1.0],
       "obj_bias":[0,0.005,0.010,0.015,0.020,0.025,0.030]}
PHYS=[k for k,v in PARAMS.items() if v[3]!="수치"]
ONE_SIDED={"obj_bias","latency","frictionloss"}

def nominal(): return {k:v[0] for k,v in PARAMS.items()}

def scaled_range(k,f):
    """범위 배율 f(좁게 0.5 · 기준 1 · 넓게 2): 공칭 중심으로 폭을 f 배 (한쪽 범위는 0 에서 f 배), 물리 하한으로 자름"""
    n,lo,hi=PARAMS[k][:3]; lo2=n-(n-lo)*f; hi2=n+(hi-n)*f
    floor={"mu_pad_obj":0.1,"grip_force":5,"finger_mass":0.05,"obj_mass_x":0.2,"arm_mass_x":0.5,"base_mass_x":0.3,"armature_x":0.2,"damping_x":0.1,"kp_x":0.5,"base_kv_x":0.3,"dt":0.0002,"solref_tc":0.003,"impratio":1.0}
    if k in floor: lo2=max(lo2,floor[k])
    return lo2,hi2

def make_model(p):
    """p(dict) → 모델. 테이블 높이는 빌드 때(정책 하강 높이 E.GRASP_Z 는 공칭 0.70 m 기준 그대로), 나머지는 컴파일된 모델 값에 덮어씀"""
    P.TABLE["top"]=TOP0+p["table_dz"]
    m=P.build(); apply_model(m,p)
    return m

def apply_model(m,p):
    """컴파일된 모델에 p 의 모델 값(마찰 · 파지력 · 질량 · 구동기 · 접촉 · dt)을 덮어씀. 테이블 높이는 빌드 때만 바뀜(make_model)"""
    I=P.Ids(m); pads=I.pads; objg=list(I.objg.values())
    m.geom_friction[pads+objg,0]=p["mu_pad_obj"]            # MuJoCo 기본 결합 = 두 geom 중 max → 손가락 · 물체 둘 다 바꿔야 실효 μ 가 바뀜
    for n in ("l","r"):
        a=m.actuator(f"left_finger_{n}").id; m.actuator_forcerange[a]=[-p["grip_force"],p["grip_force"]]
        b=m.body(f"left_finger_{n}").id; s=p["finger_mass"]/m.body_mass[b]; m.body_mass[b]*=s; m.body_inertia[b]*=s
    for nm in ARM_B:   # 실제 질량 ×s, 중력보상은 공칭 질량만큼(gravcomp = 1/s)
        b=m.body(nm).id; m.body_mass[b]*=p["arm_mass_x"]; m.body_inertia[b]*=p["arm_mass_x"]; m.body_gravcomp[b]=1.0/p["arm_mass_x"]
    for nm in BASE_B:
        b=m.body(nm).id; m.body_mass[b]*=p["base_mass_x"]; m.body_inertia[b]*=p["base_mass_x"]
    for j in ARM_J:
        dof=m.jnt_dofadr[m.joint(j).id]
        m.dof_armature[dof]*=p["armature_x"]; m.dof_damping[dof]*=p["damping_x"]; m.dof_frictionloss[dof]=p["frictionloss"]
    for i in range(m.nu):
        nm=m.actuator(i).name
        if nm.startswith(("left_shoulder","left_elbow","right_shoulder","right_elbow")):
            m.actuator_gainprm[i,0]*=p["kp_x"]; m.actuator_biasprm[i,1:3]*=p["kp_x"]
        elif nm.startswith("base_"):
            m.actuator_gainprm[i,0]*=p["base_kv_x"]; m.actuator_biasprm[i,2]*=p["base_kv_x"]
    m.geom_solref[pads+objg+[I.table],0]=p["solref_tc"]
    m.opt.timestep=p["dt"]; m.opt.impratio=p["impratio"]
    mujoco.mj_setConst(m,mujoco.MjData(m))   # 질량 · armature 변경 후 상수(acc0 등) 재계산
    return m

class BiasedExpert(E.Expert):
    """물체 위치 측정에 고정 편향(에피소드마다 방향 무작위). 성공 판정은 참값"""
    bias=np.zeros(3); judging=False
    def obj_pose(s,d):
        p,yaw,tilt=super().obj_pose(d)
        if not s.judging: p=p+s.bias
        return p,yaw,tilt
    def _judge(s,d):
        s.judging=True; super()._judge(d)

def run(p,seed,policy=None,max_t=40.0):
    """한 에피소드. policy=None 이면 전문가, 아니면 policy(obs dict) → action(8) (BC 평가용)"""
    m=make_model(p); d=mujoco.MjData(m)
    task=P.sample_task(seed); task["mass"]*=p["obj_mass_x"]
    I=P.reset(m,d,task)
    rng=np.random.default_rng(seed+77); ang=rng.uniform(0,2*np.pi)
    BiasedExpert.bias=np.array([np.cos(ang),np.sin(ang),0.0])*p["obj_bias"]
    ex=BiasedExpert(m,d,task,I)
    nsub=int(round(E.DT/p["dt"])); k=int(round(p["latency"]/p["dt"])); buf=[d.ctrl.copy()]*(k+1)
    n=0; tau_max=np.zeros(4); qa=[m.jnt_dofadr[m.joint(f"left_{a}_joint").id] for a in ("shoulder_pitch","shoulder_roll","shoulder_yaw","elbow")]
    while not ex.done and n*E.DT<max_t:
        if policy is None: act=ex.step(d)
        else:
            act=policy(d,ex); E.apply_action(m,d,act,I.act); ex_step_judge(ex,d)
        u=d.ctrl.copy()
        for _ in range(nsub):
            buf.append(u); buf.pop(0); d.ctrl[:]=buf[0]
            mujoco.mj_step(m,d)
        if ex.stage in ("LIFT","MOVE","PLACE") and not ex.fail: tau_max=np.maximum(tau_max,np.abs(d.qfrc_actuator[qa]))
        n+=1
    if not ex.done: ex._fail("approach_timeout",d); ex._judge(d)
    P.TABLE["top"]=TOP0
    r=ex.result
    return dict(seed=seed,kind=r["kind"],mass=r["mass"],success=bool(r["success"]),fail=r["fail"],fail_stage=r.get("fail_stage"),
                goal_err_mm=r["goal_err_mm"],t=r["t_total"],tau_pitch=round(float(tau_max[0]),2),tau_roll=round(float(tau_max[1]),2))

def ex_step_judge(ex,d): pass

def apply(m,seed,f=1.0):
    """박종진 policies/rand_dyn.apply 와 같은 모양: (m 복사본, seed) → 표본 dict. 동시 랜덤화(배율 f) 표본 중 모델 값만 m 에 적용.
       테이블 높이 · 측정 편향 · 지연 · 물체 질량 배율은 모델 값이 아님 → sim/w09/bc_eval_pjj.py 의 감싸기(sample_task · live_features · mj_step)로 적용"""
    p=sample_joint(seed,f); apply_model(m,p); return p   # BC 평가는 sim/w09/bc_eval.py 에서 따로

def _job(a):
    tag,p,seed=a
    try: r=run(p,seed)
    except Exception as e: r=dict(seed=seed,success=False,fail="error:"+type(e).__name__)
    r["tag"]=tag; return r

def sample_joint(seed,f):
    """동시 랜덤화: 물리 파라미터 전부를 (배율 f) 범위에서 균등 추출, 수치 설정은 공칭"""
    r=np.random.default_rng(10_000+seed); p=nominal()
    for k in PHYS: lo,hi=scaled_range(k,f); p[k]=float(r.uniform(lo,hi))
    return p

if __name__=="__main__":
    mode=sys.argv[1] if len(sys.argv)>1 else "oat"; out=os.path.join(HERE,"results")
    jobs=[]
    if mode=="oat":
        N=int(sys.argv[2]) if len(sys.argv)>2 else 40; seeds=range(1000,1000+N)
        jobs+=[("nominal",nominal(),s) for s in seeds]
        for k,(n,lo,hi,*_) in PARAMS.items():
            for lab,v in (("lo",lo),("hi",hi)):
                if v==n: continue
                p=nominal(); p[k]=v; jobs+=[(f"{k}:{lab}:{v:g}",p,s) for s in seeds]
    elif mode=="oat2":   # 스트레스: 범위 폭 ×2 양 끝 (기준 범위에서 성공률이 포화될 때 어디서 깨지는지)
        N=int(sys.argv[2]) if len(sys.argv)>2 else 40; seeds=range(1000,1000+N)
        for k in PARAMS:
            lo,hi=scaled_range(k,2.0)
            for lab,v in (("lo2",lo),("hi2",hi)):
                if abs(v-PARAMS[k][0])<1e-12: continue
                p=nominal(); p[k]=v; jobs+=[(f"{k}:{lab}:{v:g}",p,s) for s in seeds]
    elif mode=="joint":
        N=int(sys.argv[2]) if len(sys.argv)>2 else 100; seeds=range(1000,1000+N)
        for f,lab in ((0.0,"없음"),(0.5,"좁게x0.5"),(1.0,"기준x1"),(2.0,"넓게x2")):
            jobs+=[(f"joint:{lab}",sample_joint(s,f) if f>0 else nominal(),s) for s in seeds]
    elif mode=="curve":
        names=sys.argv[2].split(","); N=int(sys.argv[3]) if len(sys.argv)>3 else 40; seeds=range(1000,1000+N)
        for k in names:
            lo,hi=scaled_range(k,2.0)
            for v in CURVE.get(k,np.linspace(lo,hi,7)):
                p=nominal(); p[k]=float(v); jobs+=[(f"{k}:{v:.6g}",p,s) for s in seeds]
    t0=time.time()
    with Pool(2) as pool: R=pool.map(_job,jobs,chunksize=1)
    fn=os.path.join(out,f"randomize_{mode}.json" if mode!="curve" else f"randomize_curve_{sys.argv[2].replace(',','_')}.json")
    json.dump(dict(mode=mode,params={k:list(v) for k,v in PARAMS.items()},mujoco=mujoco.__version__,pjj="552614e",sec=round(time.time()-t0),results=R),open(fn,"w"),ensure_ascii=False,indent=0)
    import collections; C=collections.defaultdict(list)
    for r in R: C[r["tag"]].append(r["success"])
    for t,v in C.items(): print(f"{t:40s} {sum(v)}/{len(v)}")
    print("sec",round(time.time()-t0))
