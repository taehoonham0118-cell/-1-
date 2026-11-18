# 함태훈 week09 — 테이블 높이 민감도 근거: 공칭에서 GRASP 진입 순간 손끝 ↔ 테이블 · 손바닥(ee) ↔ 물체 윗면 여유 실측
#   실행: python sim/w09/w09_clearance.py   (~10 s) → results/clearance.json
import sys,os,json;sys.argv=[sys.argv[0]];HERE=os.path.dirname(os.path.abspath(__file__));sys.path.insert(0,HERE)
import randomize as R,mujoco,numpy as np
P,E=R.P,R.E; out=[]
for seed in range(1000,1010):
    m=R.make_model(R.nominal());d=mujoco.MjData(m);task=P.sample_task(seed);I=P.reset(m,d,task);ex=E.Expert(m,d,task,I);got=None
    while not ex.done and got is None:
        ex.step(d)
        for _ in range(E.NSUB): mujoco.mj_step(m,d)
        if ex.stage=="GRASP":
            tip=min(min((d.geom_xpos[g]+d.geom_xmat[g].reshape(3,3)@np.array([sx,sy,sz])*m.geom_size[g])[2] for sx in(-1,1) for sy in(-1,1) for sz in(-1,1)) for g in I.pads)
            top=d.xpos[I.obj[task["kind"]]][2]+P.obj_half_h(task["kind"])
            got=dict(seed=seed,kind=task["kind"],tip_table_mm=round((tip-P.TABLE["top"])*1e3,1),palm_objtop_mm=round((d.site_xpos[I.ee][2]-top)*1e3,1))
    if got: out.append(got); print(got)
json.dump(out,open(os.path.join(HERE,"results","clearance.json"),"w"),indent=0)
for k in ("box","cyl"):
    v=[o for o in out if o["kind"]==k]; print(k,"tip",min(o["tip_table_mm"] for o in v),"palm",min(o["palm_objtop_mm"] for o in v))
