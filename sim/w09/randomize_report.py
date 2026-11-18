# 함태훈 week09 — randomize_{oat,oat2,joint,curve}.json → 민감도 표 · Top 3 · 그림 2장 (results/randomize.md, assets/screenshots/week09/*.png)
#   민감도 순위 = ① ×2(스트레스) 성공률 하락 ② ×1 성공률 하락 ③ ×1 목표 오차 변화(같은 seed 짝 비교) — 물리 파라미터만(수치 설정 dt · solref · impratio 는 따로)
import os,sys,json,glob,collections,numpy as np
import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
HERE=os.path.dirname(os.path.abspath(__file__)); RES=os.path.join(HERE,"results"); IMG=os.path.join(HERE,"..","..","assets","screenshots","week09")
sys.argv=[sys.argv[0]]; sys.path.insert(0,HERE); import randomize as R
for f in fm.findSystemFonts():
    if "NotoSansCJK-Regular" in f or "malgun" in f.lower(): fm.fontManager.addfont(f); plt.rcParams["font.family"]=fm.FontProperties(fname=f).get_name(); break
plt.rcParams["axes.unicode_minus"]=False
C1,C2,INK,MUTE="#2a78d6","#eb6834","#0b0b0b","#52514e"
L=lambda n: json.load(open(os.path.join(RES,n)))["results"]
def group(rs):
    G=collections.defaultdict(list)
    for r in rs: G[r["tag"]].append(r)
    return G
O1=group(L("randomize_oat.json")); O2=group(L("randomize_oat2.json")); J=group(L("randomize_joint.json"))
CV=group(L(os.path.basename(glob.glob(os.path.join(RES,"randomize_curve_*.json"))[0])))
nom={r["seed"]:r for r in O1["nominal"]}; N=len(nom); n_ok=sum(r["success"] for r in nom.values())
def rate(v): return sum(r["success"] for r in v)/len(v)
def tags(G,k): return {t.split(":")[1]:(t,v) for t,v in G.items() if t.split(":")[0]==k}
rows=[]
for k,(n,lo,hi,grp,why) in R.PARAMS.items():
    a=tags(O1,k); b=tags(O2,k)
    s1=min(rate(v) for _,v in a.values()); s2=min(rate(v) for _,v in b.values())
    dge=max(abs(np.mean([r["goal_err_mm"]-nom[r["seed"]]["goal_err_mm"] for r in v if r["success"] and nom[r["seed"]]["success"]])) for _,v in a.values())
    tp=max(max(r.get("tau_pitch",0) for r in v) for _,v in a.values())
    worst2=min(b.values(),key=lambda tv: rate(tv[1])); fails=collections.Counter(r["fail"] for r in worst2[1] if not r["success"])
    lo2,hi2=R.scaled_range(k,2.0)
    rows.append(dict(k=k,grp=grp,nom=n,lo=lo,hi=hi,lo2=lo2,hi2=hi2,s1=s1,s2=s2,d1=n_ok/N-s1,d2=n_ok/N-s2,dge=dge,tp=tp,worst2=worst2[0],fails=dict(fails),why=why))
phys=[r for r in rows if r["grp"]!="수치"]; num=[r for r in rows if r["grp"]=="수치"]
phys.sort(key=lambda r:(-r["d2"],-r["d1"],-r["dge"])); TOP=[r["k"] for r in phys[:3]]
fmt=lambda x: f"{x:g}"
md=[f"# 도메인 랜덤화 · 민감도 결과 (week09, 함태훈)\n",
    f"- 시나리오 = 박종진 552614e 집기(전문가 정책 · 성공 판정 그대로, dt 0.5 ms), seed {min(nom)} ~ {max(nom)} ({N}개), 공칭 성공 **{n_ok}/{N}**. mujoco 3.15.0",
    "- 하나씩(OAT): 각 파라미터를 범위 양 끝으로(나머지 공칭). ×1 = 가정값 표 근거 범위, ×2 = 폭 2배(스트레스, 물리 하한에서 자름)",
    "- 민감도 순위 = ×2 성공률 하락 → ×1 성공률 하락 → ×1 목표 오차 변화(같은 seed 짝). 수치 설정(dt · solref · impratio)은 순위에서 뺌(물리량이 아님)\n",
    "## 1. 민감도 표 (물리 14개, 순위순)\n",
    "| 순위 | 파라미터 | 묶음 | 공칭 | ×1 범위 | ×1 최저 성공 | ×2 범위 | ×2 최저 성공 (실패 분류) | ×1 목표 오차 변화 | ×1 최대 pitch 토크 | 범위 근거 |","|---|---|---|---|---|---|---|---|---|---|---|"]
for i,r in enumerate(phys,1):
    b="**" if i<=3 else ""
    md.append(f"| {b}{i}{b} | {b}`{r['k']}`{b} | {r['grp']} | {fmt(r['nom'])} | {fmt(r['lo'])} ~ {fmt(r['hi'])} | {round(r['s1']*N)}/{N} | {r['lo2']:.4g} ~ {r['hi2']:.4g} | {b}{round(r['s2']*N)}/{N}{b} {r['fails'] or ''} | {r['dge']:+.2f} mm | {r['tp']:.1f} N·m | {r['why']} |")
md+=["\n수치 설정 (순위 밖):\n","| 파라미터 | 공칭 | ×1 범위 → 최저 | ×2 범위 → 최저 (실패) |","|---|---|---|---|"]
for r in num: md.append(f"| `{r['k']}` | {fmt(r['nom'])} | {fmt(r['lo'])} ~ {fmt(r['hi'])} → {round(r['s1']*N)}/{N} | {r['lo2']:.4g} ~ {r['hi2']:.4g} → {round(r['s2']*N)}/{N} {r['fails'] or ''} |")
md+=[f"\n**민감 파라미터 Top 3 = {', '.join(TOP)}**\n","## 2. Top 3 세부 단계 (40 seed) · 이론 검산\n","| 파라미터 | 값 → 성공 |","|---|---|"]
CVs={}
for k in TOP:
    vs=sorted(((float(t.split(':')[1]),v) for t,v in CV.items() if t.split(':')[0]==k),key=lambda x:x[0]); CVs[k]=vs
    md.append(f"| `{k}` | "+" · ".join(f"{x:g} → {sum(r['success'] for r in v)}/{len(v)}" for x,v in vs)+" |")
# 이론 검산
mu=[(x,v) for x,v in CVs.get("mu_pad_obj",[])]; agree=tot=0
for x,v in mu:
    for r in v:
        if r["fail"]=="sim_unstable": continue
        tot+=1; agree+=(r["mass"]<=2*x*40/9.81)==r["success"]
md+=["",f"- 마찰: 미끄럼 조건 m g ≤ 2 μ F (손가락 2개 · F = 40 N) → 들 수 있는 최대 질량 m_max = 2μF/g (μ 0.1 → 0.815 kg, 0.3 → 2.45 kg). seed 별 예측 vs 결과 일치 **{agree}/{tot}**(sim_unstable 제외)",
     "- 테이블 높이: 정책은 공칭 0.70 m 로 믿고 파지 높이까지 하강. 공칭 파지 순간 실측(`w09_clearance.py`) 손끝 ↔ 테이블 **≈ 16 mm**, 손바닥 ↔ 물체 윗면 상자 ≈ 16 mm · **원기둥 ≈ 11 mm**(높이 65 mm). +10 mm 까지 40/40, +12 mm 에서 원기둥 24개 전부 손바닥이 윗면에 닿아 grasp_miss(상자 15/16 성공), +14 mm 부터 계획 단계(손가락 여유 5 mm 점검)에서 손가락 ↔ 테이블 충돌 0/40. 깨지는 곳 = 원기둥 여유 11 mm · 손끝 여유 16 mm − 점검 여유 5 mm 와 일치",
     "- 위치 측정 편향: 그리퍼 벌림 110 mm − 상자 60 mm = 한쪽 여유 **25 mm**(원기둥 27 mm) → 20 mm 까지 38/40(실패 2 = 도달 한계 seed 의 ik_fail), 25 mm 에서 31/40 · 30 mm 26/40(drop · tipped · grasp_miss). 깨지는 곳 = 여유와 일치\n",
     "## 3. 동시 랜덤화 (물리 14개 전부, 에피소드마다 균등 추출, seed 1000 ~ 1099)\n","| 범위 | 성공 | 실패 분류 |","|---|---|---|"]
for t in ("joint:없음","joint:좁게x0.5","joint:기준x1","joint:넓게x2"):
    v=J[t]; md.append(f"| {t.split(':')[1]} | **{sum(r['success'] for r in v)}/{len(v)}** | {dict(collections.Counter(r['fail'] for r in v if not r['success'])) or '–'} |")
# ×2 실패 ↔ 파라미터 (정규화 위치 0 ~ 1 평균 차)
v=J["joint:넓게x2"]; ok=np.array([r["success"] for r in v]); corr=[]
for k in R.PHYS:
    lo,hi=R.scaled_range(k,2.0); x=np.array([(R.sample_joint(r["seed"],2.0)[k]-lo)/(hi-lo) for r in v]); corr.append((x[~ok].mean()-x[ok].mean(),k))
corr.sort(key=lambda c:-abs(c[0]))
md+=["",f"- ×2 실패 {int((~ok).sum())}건과 성공 사이 파라미터 위치(범위 안 0 ~ 1) 평균 차 상위: "+", ".join(f"`{k}` {c:+.2f}" for c,k in corr[:4])+" → 하나씩 결과의 Top 1(테이블 높이)과 같은 결론",
     "- 범위를 넓힐수록 하락: 좁게 99 → 기준 98 → 넓게 72 %. ×1 은 가정값 근거 범위 안에서 전문가 정책이 견딤을, ×2 는 테이블 높이 · 마찰이 근거 없이 넓어지면 「풀 수 없는 과제」가 섞임을 보임\n",
     "## 4. 그림\n","- `assets/screenshots/week09/week09_sensitivity.png` — 파라미터별 ×1 · ×2 최저 성공률","- `assets/screenshots/week09/week09_top3_curves.png` — Top 3 세부 단계 성공률 + 이론 경계"]
open(os.path.join(RES,"randomize.md"),"w").write("\n".join(md)+"\n"); json.dump(dict(top3=TOP,rows=rows),open(os.path.join(RES,"sensitivity.json"),"w"),ensure_ascii=False,indent=1,default=float)
# 그림 1: 민감도 막대 (가로, 순위순)
fig,ax=plt.subplots(figsize=(9,6.6),dpi=150); ys=np.arange(len(phys))[::-1]; h=0.36
ax.barh(ys+h/2+0.02,[r["s1"]*100 for r in phys],h,color=C1,label="×1 범위 (가정값 근거)")
ax.barh(ys-h/2-0.02,[r["s2"]*100 for r in phys],h,color=C2,label="×2 범위 (스트레스)")
for y,r in zip(ys,phys):
    if r["s2"]<0.99: ax.text(r["s2"]*100+1,y-h/2-0.02,f"{round(r['s2']*N)}/{N}",va="center",fontsize=8,color=MUTE)
ax.set_yticks(ys); ax.set_yticklabels([("★ " if r["k"] in TOP else "")+r["k"] for r in phys],fontsize=9)
ax.set_xlim(0,108); ax.set_xlabel("최저 성공률 [%] (범위 양 끝 중 낮은 쪽, seed 40개)"); ax.axvline(n_ok/N*100,color=MUTE,lw=0.8,ls="--")
ax.set_title(f"민감도 — 하나씩 랜덤화 (공칭 {n_ok}/{N}), ★ = Top 3",fontsize=11,loc="left")
for s in ("top","right"): ax.spines[s].set_visible(False)
ax.grid(axis="x",color="#e6e5e0",lw=0.6); ax.set_axisbelow(True); ax.legend(loc="upper center",bbox_to_anchor=(0.5,-0.09),ncol=2,frameon=False,fontsize=9)
fig.tight_layout(); fig.savefig(os.path.join(IMG,"week09_sensitivity.png")); plt.close(fig)
# 그림 2: Top 3 곡선
lab={"table_dz":("테이블 높이 오차 [mm]",1e3,11.2,"원기둥 손바닥 여유 11 mm"),"mu_pad_obj":("손가락 · 물체 마찰 μ",1,None,""),"obj_bias":("물체 위치 측정 편향 [mm]",1e3,25,"벌림 여유 25 mm")}
fig,axs=plt.subplots(1,3,figsize=(12,3.6),dpi=150)
for ax,k in zip(axs,TOP):
    t,sc,thr,tl=lab[k]; xs=[x*sc for x,_ in CVs[k]]; ss=[100*sum(r["success"] for r in v)/len(v) for _,v in CVs[k]]
    ax.plot(xs,ss,color=C1,lw=2,marker="o",ms=5); ax.set_xlabel(t,fontsize=9); ax.set_ylim(-5,105); ax.set_ylabel("성공률 [%]",fontsize=9)
    n0,lo,hi=R.PARAMS[k][:3]; ax.axvspan(lo*sc,hi*sc,color=C1,alpha=0.08,lw=0); ax.text(lo*sc,3,"×1 범위",fontsize=8,color=MUTE)
    if thr: ax.axvline(thr,color=C2,lw=1,ls="--"); ax.text(thr,(22 if k=="table_dz" else 50),f" {tl}",fontsize=8,color=INK)
    if k=="table_dz": ax.axvline(16.1,color=MUTE,lw=1,ls=":"); ax.text(16.1,75,f" 손끝 여유 16 mm",fontsize=8,color=INK)
    if k=="mu_pad_obj":
        mus=np.linspace(0.1,1.0,50); ms=[r["mass"] for r in CVs[k][0][1]]
        pred=[100*np.mean([m<=2*u*40/9.81 for m in ms]) for u in mus]; ax.plot(mus,pred,color=C2,lw=1,ls="--",label="이론 m ≤ 2μF/g"); ax.legend(frameon=False,fontsize=8,loc="lower right")
    for s in ("top","right"): ax.spines[s].set_visible(False)
    ax.grid(color="#e6e5e0",lw=0.6); ax.set_title(k,fontsize=10,loc="left")
fig.suptitle("Top 3 세부 단계 (seed 40개, 나머지 공칭)",fontsize=11,x=0.01,ha="left"); fig.tight_layout(); fig.savefig(os.path.join(IMG,"week09_top3_curves.png")); plt.close(fig)
print("TOP3",TOP); print(open(os.path.join(RES,"randomize.md")).read()[:6000])
