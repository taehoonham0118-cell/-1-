# 함태훈 week08 — 접촉 파라미터(solref · solimp · impratio · condim · dt) 스윕 — 박종진 집기 시나리오(week08 da55741) 그대로, CONTACT · OPTION 만 바꿈
#  실행: python sim/w08/w08_contact_sweep.py <seed,seed,…> [all|설정이름|…] [출력 json]   (무거운 물체 38건 × 15설정 ~6 분, CPU 2개)
#  PJJ 환경변수 = 박종진 저장소 경로(기본 ../PJJ0337 클론)
import sys,os,json,time,itertools,numpy as np,mujoco
from multiprocessing import Pool
PJJ=os.environ.get("PJJ",os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","..","..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P, expert as E
BASE=dict(solref=(0.01,1.0),solimp=(0.95,0.99,0.001),condim=4,impratio=10,dt=0.001,finger_mass=0.2)
CFG=[("기준(박종진 임시값)",{}),
     ("solref 0.005",dict(solref=(0.005,1.0))),("solref 0.02",dict(solref=(0.02,1.0))),("solref 0.04",dict(solref=(0.04,1.0))),
     ("dampratio 0.7",dict(solref=(0.01,0.7))),("dampratio 2",dict(solref=(0.01,2.0))),
     ("solimp width 5 mm",dict(solimp=(0.95,0.99,0.005))),("solimp 0.9·0.95",dict(solimp=(0.9,0.95,0.001))),("solimp 0.99·0.999",dict(solimp=(0.99,0.999,0.001))),
     ("impratio 1",dict(impratio=1)),("impratio 3",dict(impratio=3)),
     ("condim 3",dict(condim=3)),("condim 6",dict(condim=6)),
     ("dt 0.5 ms",dict(dt=0.0005)),
     ("solref 0.02 + width 5 mm",dict(solref=(0.02,1.0),solimp=(0.95,0.99,0.005)))]
def setup(c):
    p={**BASE,**c}
    P.CONTACT.update(solref=p["solref"],solimp=p["solimp"],condim=p["condim"])
    P.GRIPPER["finger_mass"]=p["finger_mass"]
    P.OPTION=f'timestep="{p["dt"]}" cone="elliptic" impratio="{p["impratio"]}"'
    E.NSUB=int(round(0.02/p["dt"]))
    return P.build()
def one(args):
    name,c,seed=args; m=setup(c); st=dict(pen=0.0,f=0.0,nflip=0,last=None,lift_wmax=0.0)
    def hook(d,act,ex):
        I=ex.I; og=set(I.objg.values()); pen=0; f=0; n=0
        for i in range(d.ncon):
            cc=d.contact[i]
            if (cc.geom1 in I.pads and cc.geom2 in og) or (cc.geom2 in I.pads and cc.geom1 in og):
                n+=1; pen=max(pen,-cc.dist); fr=np.zeros(6); mujoco.mj_contactForce(m,d,i,fr); f=max(f,fr[0])
        if ex.stage in("LIFT","MOVE","PLACE"):
            st["pen"]=max(st["pen"],pen); st["f"]=max(st["f"],f)
            k=ex.task["kind"]; w=np.abs(d.qvel[I.objv[k]+3:I.objv[k]+6]).max(); st["lift_wmax"]=max(st["lift_wmax"],w)
            if st["last"] is not None and n!=st["last"]: st["nflip"]+=1
            st["last"]=n
    t0=time.time(); r,d=E.run_episode(m,seed,on_step=hook)
    return dict(cfg=name,seed=seed,kind=r["kind"],mass=r["mass"],success=r["success"],fail=r["fail"],t=r["t_total"],
                pen_mm=round(st["pen"]*1e3,3),fmax_N=round(st["f"],1),contact_flips=st["nflip"],w_max=round(st["lift_wmax"],3),sec=round(time.time()-t0,1))
if __name__=="__main__":
    seeds=[int(s) for s in sys.argv[1].split(",")] if len(sys.argv)>1 else [1080]
    cfgs=CFG if len(sys.argv)<3 or sys.argv[2] in ("","all") else [c for c in CFG if c[0] in sys.argv[2].split("|")]
    jobs=[(n,c,s) for n,c in cfgs for s in seeds]
    with Pool(2) as pool: R=pool.map(one,jobs,chunksize=1)
    out=sys.argv[3] if len(sys.argv)>3 else os.path.join(os.path.dirname(os.path.abspath(__file__)),"results","contact_sweep.json"); json.dump(R,open(out,"w"),ensure_ascii=False,indent=1)
    for r in R: print(r)
