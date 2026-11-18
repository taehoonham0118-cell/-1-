# 함태훈 week09 — 팀 연계: 박종진 BC(99d60da policies/ckpt/bc_joint_dart_c25.pt)를 함태훈 randomize.py 랜덤화로 평가 → 실패 ↔ 민감 파라미터
#   롤아웃 · 판정 = 박종진 eval_bc.run() 그대로(T_MAX 34 s, 같은 판정 · 실패 분류), 평가 seed 2000 ~ 2049 (박종진 결과와 바로 비교)
#   환경 = randomize.make_model(p) (테이블 높이 · 마찰 · 질량 · 구동기 …), 거기에 eval_bc 가 못 하는 3개를 덧붙임:
#     물체 질량 배율(sample_task 감쌈) · 물체 위치 측정 편향(관측 live_features 에만) · 명령 지연(시뮬 스텝 버퍼)
#   정책 = pt_numpy.NumpyPolicy(PyTorch 없이 같은 MLP 추론) — 노트북에서는 train_bc.Policy 로 바꿔도 됨
#   실행: python sim/w09/bc_eval_pjj.py [n=50] [정책=joint_dart_c25]          (~13 분, CPU 2개) → results/bc_eval_pjj.json
#         python sim/w09/bc_eval_pjj.py 25 joint_dart_c25 oat                   (~15 분) → results/bc_eval_pjj_oat.json (BC 하나씩 민감도)
import sys,os,json,time,collections,numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__)); sys.argv_saved=list(sys.argv); sys.argv=[sys.argv[0]]; sys.path.insert(0,HERE)
import randomize as R
sys.argv=sys.argv_saved
sys.path.insert(0,os.path.join(R.PJJ,"policies"))
import eval_bc as EB, bc_common as B, pt_numpy
P,E=R.P,R.E
_orig_sample=P.sample_task; _orig_feat=B.live_features; _real_step=mujoco.mj_step
CUR=dict(p=None,bias=np.zeros(3),buf=None,k=0)
def _sample(seed):
    t=_orig_sample(seed); t["mass"]*=CUR["p"]["obj_mass_x"]; return t
def _feat(m,d,I,kind):
    ob=I.obj[kind]; d.xpos[ob]+=CUR["bias"]; x=_orig_feat(m,d,I,kind); d.xpos[ob]-=CUR["bias"]; return x
class _MJ:   # eval_bc 안의 mujoco.mj_step 만 지연 버퍼로 감쌈(나머지는 그대로)
    def __getattr__(s,k): return getattr(mujoco,k)
    def mj_step(s,m,d):
        if CUR["k"]>0:
            if CUR["buf"] is None: CUR["buf"]=[d.ctrl.copy()]*(CUR["k"]+1)
            CUR["buf"].append(d.ctrl.copy()); CUR["buf"].pop(0); d.ctrl[:]=CUR["buf"][0]
        _real_step(m,d)
EB.P.sample_task=_sample; EB.B.live_features=_feat; EB.mujoco=_MJ()

def one(args):
    who,tag,p,seed=args
    CUR.update(p=p,buf=None,k=int(round(p["latency"]/p["dt"])))
    ang=np.random.default_rng(seed+77).uniform(0,2*np.pi); CUR["bias"]=np.array([np.cos(ang),np.sin(ang),0.0])*p["obj_bias"]
    if who=="expert":
        r=R.run(p,seed); r=dict(seed=seed,kind=r["kind"],mass=r["mass"],success=r["success"],fail=r["fail"],goal_err_mm=r["goal_err_mm"])
    else:
        m0=R.make_model(p); pol=pt_numpy.NumpyPolicy(os.path.join(R.PJJ,"policies","ckpt",f"bc_{who}.pt"))
        r=EB.run(m0,pol,seed,False); r.pop("rand",None)
    P.TABLE["top"]=R.TOP0
    r.update(who=who,tag=tag); return r

def conds():
    nom=R.nominal(); C=[("없음",lambda s:nom),("동시 ×1",lambda s:R.sample_joint(s,1.0)),("동시 ×2",lambda s:R.sample_joint(s,2.0))]
    for k,v,lab in (("table_dz",0.010,"테이블 +10 mm"),("table_dz",-0.010,"테이블 −10 mm"),("mu_pad_obj",0.5,"μ 0.5"),("obj_bias",0.010,"편향 10 mm")):
        p=dict(nom); p[k]=v; C.append((lab,(lambda pp: lambda s:pp)(p)))
    return C

if __name__=="__main__":
    from multiprocessing import Pool
    N=int(sys.argv[1]) if len(sys.argv)>1 else 50; pol=sys.argv[2] if len(sys.argv)>2 else "joint_dart_c25"; seeds=range(2000,2000+N)
    mode=sys.argv[3] if len(sys.argv)>3 else "conds"
    if mode=="oat":   # BC 의 민감도(하나씩, ×1 범위 양 끝) — 전문가 민감도 순위와 비교용
        jobs=[(pol,"없음",R.nominal(),s) for s in seeds]
        for k in R.PHYS:
            n0,lo,hi,*_=R.PARAMS[k]
            for v in (lo,hi):
                if v==n0: continue
                p=R.nominal(); p[k]=v; jobs+=[(pol,f"{k}={v:g}",p,s) for s in seeds]
    else: jobs=[(w,lab,f(s),s) for w in ("expert",pol) for lab,f in conds() for s in seeds]
    t0=time.time()
    with Pool(2) as pool: rows=pool.map(one,jobs,chunksize=1)
    json.dump(dict(policy=pol,pjj="99d60da",seeds=[2000,2000+N-1],mujoco=mujoco.__version__,sec=round(time.time()-t0),rows=rows),
              open(os.path.join(HERE,"results","bc_eval_pjj.json" if mode!="oat" else "bc_eval_pjj_oat.json"),"w"),ensure_ascii=False,indent=0,default=float)
    G=collections.defaultdict(list)
    for r in rows: G[(r["who"],r["tag"])].append(r)
    for (w,t),v in G.items(): print(f"{w:16s} {t:14s} {sum(r['success'] for r in v)}/{len(v)}",dict(collections.Counter(r['fail'] for r in v if not r['success'])))
    print("sec",round(time.time()-t0))
