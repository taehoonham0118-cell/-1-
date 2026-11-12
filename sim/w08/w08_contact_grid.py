# 함태훈 week08 — solref · solimp · dt 격자: 손가락 ↔ 물체 관통 깊이 · 미끄러짐 (박종진 gripper_slip.trial 장치 그대로, 3 kg 원기둥 · μ 1.0 · F 40 N)
#  이론: 정상상태 관통 ≈ 접촉 「스프링」 k = d(r)/(d_w² · tc² · dmax... ) → tc 가 클수록(부드러울수록) · dmin 이 작을수록 깊어짐
import sys,os,json,itertools,numpy as np
PJJ=os.environ.get("PJJ",os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","..","..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P, gripper_slip as GS
OUT=os.path.join(os.path.dirname(os.path.abspath(__file__)),"results")
rows=[]
for tc,dr,si,dt in itertools.product((0.005,0.01,0.02,0.04),(0.7,1.0,2.0),((0.9,0.95,0.001),(0.95,0.99,0.001),(0.99,0.999,0.001),(0.95,0.99,0.005)),(0.001,0.0005)):
    P.CONTACT["solref"]=(tc,dr); P.CONTACT["solimp"]=si
    GS.CONTACT=P.CONTACT
    opt=f'timestep="{dt}" cone="elliptic" impratio="10"'
    try: r=GS.trial(1.0,40.0,3.0,"cyl",opt=opt); r["unstable"]=False
    except Exception as e: r=dict(unstable=True)
    import mujoco
    r.update(tc=tc,dr=dr,solimp=list(si),dt=dt); rows.append(r)
json.dump(rows,open(os.path.join(OUT,"contact_grid.json"),"w"),ensure_ascii=False,indent=1)
for r in rows: print(r.get("tc"),r.get("dr"),r.get("solimp"),r.get("dt"),r.get("pen_mm"),r.get("slip_lift_mm"),r.get("slip_hold_mm"),r.get("held"))
