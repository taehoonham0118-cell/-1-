# 함태훈 week09 — 랜덤화 환경 평가용 「임시 기준 BC」 + BC 롤아웃 판정기
#   목적: 박종진 BC(policies/) 가 올라오기 전에 「랜덤화 환경에서 BC 정책 평가 → 실패를 민감 파라미터와 연결」 파이프라인을 먼저 검증.
#   박종진 정책이 오면 policy(obs) 만 바꿔 끼우면 같은 판정 · 같은 랜덤화로 다시 돈다 (evaluate()).
#   torch 미설치 환경(클라우드 프록시가 download.pytorch.org 차단) → scikit-learn MLPRegressor(CPU) 로 같은 구조(MLP 256×256, ReLU, Adam)
#   실행:  python sim/w09/bc_baseline.py collect 100        (전문가 seed 1000 ~ 1099 → results/bc_data.npz, 데이터 명세 v1.0 의 obs 부분집합)
#          python sim/w09/bc_baseline.py train                (→ results/bc_mlp.pkl, 학습 / 검증 = seed 끝자리 0 · 1 = 검증 20 %)
#          python sim/w09/bc_baseline.py eval [N=40]          (전문가 vs BC × 랜덤화 없음 / 기준 동시 / Top 3 각각 → results/bc_eval.json)
import sys,os,json,time,pickle,collections,numpy as np,mujoco
from multiprocessing import Pool
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE)
import randomize as R
P,E=R.P,R.E
OUT=os.path.join(HERE,"results")

def obs_vec(m,d,I,kind):
    """관측 27: 베이스 x · y · sin · cos(yaw), 팔 4, 그리퍼 폭, 베이스 속도 3, 팔 속도 4, 끝점 3, 물체 3, 물체 − 끝점 3, 물체 − 목표 2, 물체 yaw sin · cos(4θ, 상자 대칭)"""
    qa=E.qadr(m,"left",True); q=d.qpos[qa]; dv=[m.jnt_dofadr[m.joint(j).id] for j in ("base_planar_joint_TX","base_planar_joint_TY","base_planar_joint_RZ")]+[m.jnt_dofadr[m.joint(f"left_{a}_joint").id] for a in E.ARM]
    ee=d.site_xpos[I.ee]; ob=I.obj[kind]; po=d.xpos[ob]; Rm=d.xmat[ob].reshape(3,3); yaw=np.arctan2(Rm[1,0],Rm[0,0])
    g=np.array([P.TABLE["edge_x"]+P.GOAL["depth"],P.GOAL["y"]])
    return np.concatenate([q[:2],[np.sin(q[2]),np.cos(q[2])],q[3:],[2*np.mean(d.qpos[I.fq])],d.qvel[dv],ee,po,po-ee,po[:2]-g,[np.sin(4*yaw),np.cos(4*yaw)]]).astype(np.float32)

def act_to_label(m,d,act):
    """학습 라벨: 베이스 속도 3, 팔 목표 − 현재 4(델타), 그리퍼 목표 1"""
    qa=E.qadr(m,"left",True); return np.concatenate([act[:3],act[3:7]-d.qpos[qa[3:]],act[7:8]]).astype(np.float32)
def label_to_act(m,d,y):
    qa=E.qadr(m,"left",True); return np.concatenate([y[:3],y[3:7]+d.qpos[qa[3:]],np.clip(y[7:8],0,P.GRIPPER["stroke"])])

NOISE=dict(arm=0.0,base=0.0)   # DART(잡음 주입 수집): 실행 action 에만 가우스 잡음, 라벨 = 전문가 깨끗한 action → 벗어난 상태에서의 복귀 라벨이 생김
def _collect(seed):
    rng=np.random.default_rng(seed+5); p=R.nominal(); m=R.make_model(p); d=mujoco.MjData(m); task=P.sample_task(seed); I=P.reset(m,d,task); ex=E.Expert(m,d,task,I)
    X=[];Y=[];n=0
    while not ex.done and n*E.DT<40:
        o=obs_vec(m,d,I,task["kind"]); a=ex.step(d); X.append(o); Y.append(act_to_label(m,d,a))
        if NOISE["arm"]>0:
            an=a.copy(); an[:3]+=rng.normal(0,NOISE["base"],3); an[3:7]+=rng.normal(0,NOISE["arm"],4); E.apply_action(m,d,an,I.act)
        for _ in range(E.NSUB): mujoco.mj_step(m,d)
        n+=1
    if not ex.done: ex._fail("approach_timeout",d); ex._judge(d)
    return seed,ex.result["success"],np.array(X),np.array(Y)

class Judge:
    """BC 롤아웃 판정(전문가 상태기계 없이, 측정만): 성공 = 목표 원 안 · 테이블 위 · 기울기 < 15° · 들어 올린 적 있음.
       실패 분류: table_collision(손가락 외 로봇 ↔ 테이블) · sim_unstable · grasp_miss(양 손가락 접촉 없음) · slip/drop(잡았으나 들기 전 · 후 놓침) · place_miss · tipped"""
    def __init__(s,m,I,kind): s.m,s.I,s.kind=m,I,kind; s.ob=I.obj[kind]; s.grasped=False; s.lifted=False; s.coll=False; s.unstable=False; s.zmax=0
    def step(s,d):
        C=P.contacts(s.m,d,s.I); W=mujoco.mjtWarning
        if any(d.warning[w].number>0 for w in (W.mjWARN_BADQACC,W.mjWARN_BADQVEL)): s.unstable=True
        if any(not g.startswith("left_pad") for g in C["robot_table"]): s.coll=True
        if len(C["pad_obj"])==2: s.grasped=True
        z=d.xpos[s.ob][2]-(P.TABLE["top"]+P.obj_half_h(s.kind)); s.zmax=max(s.zmax,z)
        if z>0.03 and len(C["pad_obj"])==2: s.lifted=True   # 들기 높이 pre_h 는 계획마다 다름(≥ 0.05 m) → 30 mm 넘으면 들었음
    def result(s,d):
        p=d.xpos[s.ob]; R_=d.xmat[s.ob].reshape(3,3); tilt=np.degrees(np.arccos(np.clip(R_[2,2],-1,1)))
        g=np.array([P.TABLE["edge_x"]+P.GOAL["depth"],P.GOAL["y"]]); e=float(np.linalg.norm(p[:2]-g)); on=abs(p[2]-(P.TABLE["top"]+P.obj_half_h(s.kind)))<0.01
        if s.unstable: f="sim_unstable"
        elif s.coll: f="table_collision"
        elif not s.grasped: f="grasp_miss"
        elif not s.lifted: f="slip"
        elif not on and p[2]<P.TABLE["top"]-0.05: f="drop"
        elif tilt>15: f="tipped"
        elif e>P.GOAL["tol"] or not on: f="place_miss"
        else: f=None
        return dict(success=f is None,fail=f,goal_err_mm=round(e*1e3,1),zmax_mm=round(s.zmax*1e3,1))

_MODEL=None
def bc_policy_factory():
    global _MODEL
    if _MODEL is None: _MODEL=pickle.load(open(os.path.join(OUT,os.environ.get("BC_MODEL","bc_mlp")+".pkl"),"rb"))
    mu,sd,ym,ys,net=_MODEL["mu"],_MODEL["sd"],_MODEL["ym"],_MODEL["ys"],_MODEL["net"]
    def pol(m,d,I,kind):
        x=(obs_vec(m,d,I,kind)-mu)/sd; y=net.predict(x[None])[0]*ys+ym; return label_to_act(m,d,y)
    return pol

def rollout(args):
    """args = (정책 'expert'|'bc', 파라미터 dict, seed, tag) → 같은 랜덤화 적용(make_model · 질량 배율 · 측정 편향 · 지연)"""
    who,p,seed,tag=args
    m=R.make_model(p); d=mujoco.MjData(m); task=P.sample_task(seed); task["mass"]*=p["obj_mass_x"]; I=P.reset(m,d,task)
    rng=np.random.default_rng(seed+77); ang=rng.uniform(0,2*np.pi); bias=np.array([np.cos(ang),np.sin(ang),0.0])*p["obj_bias"]
    J=Judge(m,I,task["kind"]); nsub=int(round(E.DT/p["dt"])); k=int(round(p["latency"]/p["dt"])); buf=[d.ctrl.copy()]*(k+1)
    if who=="expert": R.BiasedExpert.bias=bias; ex=R.BiasedExpert(m,d,task,I); pol=None
    else: pol=bc_policy_factory(); ob=I.obj[task["kind"]]
    T=32.0 if who=="bc" else 40.0; n=0
    while n*E.DT<T:
        if who=="expert":
            if ex.done: break
            ex.step(d)
        else:
            d.xpos[ob]+=bias; a=pol(m,d,I,task["kind"]); d.xpos[ob]-=bias   # 관측에만 편향
            E.apply_action(m,d,a,I.act)
        u=d.ctrl.copy()
        for _ in range(nsub):
            buf.append(u); buf.pop(0); d.ctrl[:]=buf[0]; mujoco.mj_step(m,d)
        J.step(d); n+=1
    r=J.result(d)
    if who=="expert" and ex.done: r["expert_fail"]=ex.result["fail"]
    P.TABLE["top"]=R.TOP0
    r.update(who=who,seed=seed,tag=tag,kind=task["kind"],mass=round(task["mass"],3)); return r

if __name__=="__main__":
    mode=sys.argv[1]
    if mode=="collect":
        N=int(sys.argv[2]) if len(sys.argv)>2 else 100; tag=sys.argv[3] if len(sys.argv)>3 else ""
        if tag=="dart": NOISE.update(arm=0.03,base=0.05)
        with Pool(2) as pool: Z=pool.map(_collect,range(1000,1000+N))
        np.savez_compressed(os.path.join(OUT,f"bc_data{'_'+tag if tag else ''}.npz"),seeds=np.concatenate([[s]*len(X) for s,_,X,_ in Z]),
                            X=np.concatenate([X for *_,X,_ in Z]),Y=np.concatenate([Y for *_,Y in Z]),ok=np.array([o for _,o,_,_ in Z]))
        print(N,"eps",sum(o for _,o,_,_ in Z),"success", sum(len(X) for *_,X,_ in Z),"steps")
    elif mode=="train":
        from sklearn.neural_network import MLPRegressor
        names=sys.argv[2].split(",") if len(sys.argv)>2 else ["bc_data"]; out=sys.argv[3] if len(sys.argv)>3 else "bc_mlp"
        Ds=[np.load(os.path.join(OUT,n+".npz")) for n in names]; X=np.concatenate([D["X"] for D in Ds]); Y=np.concatenate([D["Y"] for D in Ds]); S=np.concatenate([D["seeds"] for D in Ds])
        keep=np.concatenate([np.isin(D["seeds"],1000+np.where(D["ok"])[0]) for D in Ds])   # 성공 에피소드만(실패 · 발산 에피소드 라벨은 버림)
        X,Y,S=X[keep],Y[keep],S[keep]; va=(S%10)<2
        mu=X[~va].mean(0); sd=X[~va].std(0)+1e-6; ym=Y[~va].mean(0); ys=Y[~va].std(0)+1e-6
        net=MLPRegressor(hidden_layer_sizes=(256,256),activation="relu",learning_rate_init=1e-3,batch_size=256,max_iter=60,random_state=0,early_stopping=False,verbose=False)
        t0=time.time(); net.fit((X[~va]-mu)/sd,(Y[~va]-ym)/ys)
        mse=lambda a,b: float(np.mean((net.predict((a-mu)/sd)-(b-ym)/ys)**2))
        info=dict(train_steps=int((~va).sum()),val_steps=int(va.sum()),train_mse=round(mse(X[~va],Y[~va]),4),val_mse=round(mse(X[va],Y[va]),4),
                  val_seeds=sorted(set(int(s) for s in S[va])),epochs=net.n_iter_,sec=round(time.time()-t0,1),loss_curve=[round(l,5) for l in net.loss_curve_])
        info["data"]=names
        pickle.dump(dict(mu=mu,sd=sd,ym=ym,ys=ys,net=net),open(os.path.join(OUT,out+".pkl"),"wb"))
        json.dump(info,open(os.path.join(OUT,out+"_train.json"),"w"),indent=0); print({k:v for k,v in info.items() if k!="loss_curve"})
    elif mode=="eval":
        N=int(sys.argv[2]) if len(sys.argv)>2 else 40; top=sys.argv[3].split(",") if len(sys.argv)>3 else []
        seeds=[s for s in range(1000,1100) if s%10<2][:N]   # 검증 seed(학습에 안 쓴 것) 우선
        seeds+= [s for s in range(1100,1100+max(0,N-len(seeds)))]
        jobs=[]
        for who in ("expert","bc"):
            jobs+=[(who,R.nominal(),s,"없음") for s in seeds]
            jobs+=[(who,R.sample_joint(s,1.0),s,"동시 기준x1") for s in seeds]
            for k in top:
                _,lo,hi,*_=R.PARAMS[k]; v=hi if abs(hi-R.PARAMS[k][0])>=abs(lo-R.PARAMS[k][0]) else lo
                p=R.nominal(); p[k]=v; jobs+=[(who,p,s,f"{k}={v:g}") for s in seeds]
        with Pool(2) as pool: Rr=pool.map(rollout,jobs,chunksize=1)
        json.dump(dict(model=os.environ.get("BC_MODEL","bc_mlp"),seeds=seeds,results=Rr),open(os.path.join(OUT,"bc_eval.json"),"w"),ensure_ascii=False,indent=0)
        C=collections.defaultdict(list)
        for r in Rr: C[(r["who"],r["tag"])].append(r)
        for (w,t),v in C.items(): print(f"{w:7s} {t:22s} {sum(r['success'] for r in v)}/{len(v)}",collections.Counter(r["fail"] for r in v if not r["success"]))
