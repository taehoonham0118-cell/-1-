# 함태훈 week08 — seed 1080(원기둥 2.98 kg) 집기 GIF: 왼쪽 dt 1 ms(들기 순간 수치 발산) / 오른쪽 dt 0.5 ms(성공), 같은 시각 나란히
#   박종진 PJJ0337/-1- da55741 scenarios/ 그대로(OPTION · NSUB 만 바꿈). 9.6 ~ 14 s 구간 0.1 s 간격 + 오른쪽 끝까지 0.4 s 간격
#   출력: assets/screenshots/week08/week08_seed1080_dt.gif     실행: python sim/w08/w08_seed1080_gif.py  (리눅스 무화면이면 MUJOCO_GL=osmesa)
import os,sys,numpy as np,mujoco
from PIL import Image,ImageDraw,ImageFont
HERE=os.path.dirname(os.path.abspath(__file__))
PJJ=os.environ.get("PJJ",os.path.join(HERE,"..","..","..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios")
import pick_env as P, expert as E
W,H=400,320; AZ=float(os.environ.get("AZ","160")); HD=70
try: F=ImageFont.truetype(next(x for x in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc","C:/Windows/Fonts/malgun.ttf") if os.path.exists(x)),15)
except StopIteration: F=None
def run(dt):
    P.OPTION=f'timestep="{dt}" cone="elliptic" impratio="10"'; E.NSUB=int(round(0.02/dt))
    m=P.build(cam=True); r=mujoco.Renderer(m,H,W)
    cam=mujoco.MjvCamera(); cam.lookat[:]=[0.62,-0.02,0.80]; cam.distance=1.5; cam.azimuth=AZ; cam.elevation=-28
    fr={}; info=dict(t=0)
    def cb(d,a,ex):
        t=round(ex.t_total,2); info["t"]=t; info["stage"]=ex.stage
        if ex.fail: return          # 발산 판정 뒤에는 MuJoCo 가 상태를 리셋하므로 기록하지 않음(마지막 장면 고정)
        if (9.6<=t<=14.0 and abs(t*10-round(t*10))<1e-6) or (t>14.0 and abs(t*2.5-round(t*2.5))<1e-6):
            r.update_scene(d,cam); k=ex.task["kind"]; w=float(np.abs(d.qvel[ex.I.objv[k]+3:ex.I.objv[k]+6]).max())
            fr[t]=(r.render().copy(),ex.stage,w)
    res,d=E.run_episode(m,1080,on_step=cb); return res,fr
R1,F1=run(0.001); R2,F2=run(0.0005)
ts=sorted(set(F1)|set(F2)); frames=[]; last1=None
for t in ts:
    can=Image.new("RGB",(2*W,H+HD),(255,255,255)); dr=ImageDraw.Draw(can)
    for j,(Fx,R,lab) in enumerate(((F1,R1,"dt 1 ms"),(F2,R2,"dt 0.5 ms (B1 v1.0)"))):
        if t in Fx: im,st,w=Fx[t]; 
        elif j==0 and last1 is not None: im,st,w=last1; st="발산(11.48 s) → 종료"
        else: continue
        if j==0 and t in Fx: last1=(im,st,w)
        can.paste(Image.fromarray(im),(j*W,HD))
        col=(180,30,30) if (j==0 and (t not in Fx or w>0.5)) else (30,60,160)
        dr.text((j*W+8,3),lab,fill=(20,20,20),font=F); dr.text((j*W+8,25),f"t {t:5.1f} s · {st}",fill=col,font=F); dr.text((j*W+8,46),f"물체 각속도 {w:.2f} rad/s",fill=col,font=F)
    frames.append(can.convert("P",palette=Image.ADAPTIVE,colors=128))
out=os.path.join(HERE,"..","..","assets","screenshots","week08"); os.makedirs(out,exist_ok=True); p=os.path.join(out,"week08_seed1080_dt.gif")
frames[0].save(p,save_all=True,append_images=frames[1:]+[frames[-1]]*10,duration=120,loop=0,optimize=True)
print(R1["success"],R1["fail"],R1["t_total"],"|",R2["success"],R2["fail"],R2["goal_err_mm"],"|",len(frames),"frames",os.path.getsize(p)//1024,"KB")
