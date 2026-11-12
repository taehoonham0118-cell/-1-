# 함태훈 week08 — seed 1080(원기둥 2.98 kg) 발산 진단: 제어 주기마다 손가락 ↔ 물체 접촉 수 · 관통 · 법선력 · 물체 각속도 기록 (dt 1 ms 기준 vs 0.5 ms)
import sys,os,csv,numpy as np,mujoco
HERE=os.path.dirname(os.path.abspath(__file__))
PJJ=os.environ.get("PJJ",os.path.join(HERE,"..","..","..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P, expert as E
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
plt.rcParams["font.family"]=["Malgun Gothic","NanumGothic","Noto Sans CJK KR","DejaVu Sans"]; plt.rcParams["axes.unicode_minus"]=False
def run(dt):
    P.OPTION=f'timestep="{dt}" cone="elliptic" impratio="10"'; E.NSUB=int(round(0.02/dt)); m=P.build(); L=[]
    def hook(d,act,ex):
        I=ex.I; og=set(I.objg.values()); pen=0; f=0; n=0
        for i in range(d.ncon):
            c=d.contact[i]
            if (c.geom1 in I.pads and c.geom2 in og) or (c.geom2 in I.pads and c.geom1 in og):
                n+=1; pen=max(pen,-c.dist); fr=np.zeros(6); mujoco.mj_contactForce(m,d,i,fr); f=max(f,fr[0])
        k=ex.task["kind"]; v=d.qvel[I.objv[k]:I.objv[k]+6]
        L.append(dict(t=round(d.time,3),stage=ex.stage,pad_contacts=n,pen_mm=round(pen*1e3,4),fn_max_N=round(f,2),obj_w=round(float(np.abs(v[3:]).max()),4),obj_v=round(float(np.abs(v[:3]).max()),4)))
    r,_=E.run_episode(m,1080,on_step=hook); return r,L
out={}
for dt in (0.001,0.0005):
    r,L=run(dt); out[dt]=(r,L)
    with open(os.path.join(HERE,"results",f"seed1080_dt{dt*1e3:g}ms.csv"),"w",newline="") as f:
        w=csv.DictWriter(f,fieldnames=list(L[0].keys())); w.writeheader(); w.writerows(L)
    print(dt,r["success"],r["fail"],r["t_total"])
fig,ax=plt.subplots(3,1,figsize=(8,6.4),sharex=True)
for dt,c in ((0.001,"#c0392b"),(0.0005,"#2c7fb8")):
    L=[x for x in out[dt][1] if 9.8<=x["t"]<=12.5]; t=[x["t"] for x in L]; lab=f"dt {dt*1e3:g} ms ({'발산' if not out[dt][0]['success'] else '성공'})"
    ax[0].plot(t,[x["obj_w"] for x in L],c=c,label=lab); ax[1].plot(t,[x["pen_mm"] for x in L],c=c); ax[2].step(t,[x["pad_contacts"] for x in L],c=c,where="post")
ax[0].set_yscale("log"); ax[0].set_ylabel("물체 각속도 [rad/s]"); ax[1].set_ylabel("손가락 관통 [mm]"); ax[2].set_ylabel("손가락↔물체 접촉 수"); ax[2].set_xlabel("시간 [s] (GRASP 10.1 s → LIFT 10.66 s)")
ax[0].legend(); ax[0].set_title("seed 1080 (원기둥 2.98 kg) 들기 순간 — dt 1 ms 는 각속도가 지수적으로 커져 발산")
fig.tight_layout(); os.makedirs(os.path.join(HERE,"..","..","assets","screenshots","week08"),exist_ok=True)
fig.savefig(os.path.join(HERE,"..","..","assets","screenshots","week08","week08_seed1080_diag.png"),dpi=130)
