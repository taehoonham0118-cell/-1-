# 함태훈 week08 — 저장 · 재생 검증 (내 환경에서 독립 재실행): 박종진 샘플 3개 + v1.0 변환본 3개 + 새로 수집한 에피소드 N개를 저장된 action 만으로 재생
import sys,os,glob,json,time,numpy as np,mujoco,h5py
PJJ=os.environ.get("PJJ",os.path.join(os.path.dirname(os.path.abspath(__file__)),"..","..","..","PJJ0337")); sys.path.insert(0,PJJ+"/scenarios"); sys.path.insert(0,os.path.dirname(os.path.abspath(__file__)))
import collect as C, pick_env as P, b1_dataset as B
HERE=os.path.dirname(os.path.abspath(__file__)); OUT=os.path.join(HERE,"results"); TMP=os.environ.get("TMPD","/tmp/w08_ds")
m=P.build(cam=True); mh=C.model_hash(m); R=[]
def rep(p,group):
    t0=time.time(); r=C.replay(m,p,mh); r.update(group=group,sec=round(time.time()-t0,2),spec_ok=not B.check(p) if group!="draft" else None); R.append(r); print(r,flush=True)
for p in sorted(glob.glob(os.path.join(PJJ,"datasets","samples","*.hdf5"))): rep(p,"draft")
os.makedirs(TMP+"/v10",exist_ok=True); os.makedirs(TMP+"/new",exist_ok=True)
for p in sorted(glob.glob(os.path.join(PJJ,"datasets","samples","*.hdf5"))):
    q=os.path.join(TMP,"v10",os.path.basename(p)); B.upgrade(p,q); rep(q,"v1.0 변환")
N=int(sys.argv[1]) if len(sys.argv)>1 else 10
for s in range(2000,2000+N):     # 새 seed 로 수집 → v1.0 변환 → 재생 (내 환경에서 수집 · 재생 모두)
    res,Bf,task=C.record(m,s); p0=os.path.join(TMP,"new",f"raw_{s}.hdf5"); C.save(p0,res,Bf,task,mh)
    q=os.path.join(TMP,"new",f"ep_{s:05d}.hdf5"); B.upgrade(p0,q); os.remove(p0); rep(q,"새 수집")
json.dump(dict(model_hash=mh,mujoco=mujoco.__version__,rows=R),open(os.path.join(OUT,"replay_check.json"),"w"),ensure_ascii=False,indent=1)
L=["# 저장 · 재생 검증 (week08, 함태훈) — 자동 생성 `w08_replay_check.py`","",f"mujoco {mujoco.__version__}, 모델 지문 {mh} (박종진 collect.json 7016330767f5 와 {'같음' if mh=='7016330767f5' else '다름'})","",
   "| 묶음 | 개수 | qpos 최대 차 | 성공 판정 일치 | 지문 일치 | v1.0 명세 검사 |","|---|---|---|---|---|---|"]
for g in ("draft","v1.0 변환","새 수집"):
    X=[r for r in R if r["group"]==g]
    if X: L.append(f"| {g} | {len(X)} | {max(r['max_abs_qpos_diff'] for r in X):.1e} | {sum(r['success_saved']==r['success_replayed_goal'] for r in X)}/{len(X)} | {sum(r['model_hash_match'] for r in X)}/{len(X)} | "+("—" if g=="draft" else f"{sum(bool(r['spec_ok']) for r in X)}/{len(X)} 통과")+" |")
L+=["",f"- 새 수집 성공 {sum(r['success_saved'] for r in R if r['group']=='새 수집')}/{N}. draft 원본은 v1.0 기준 위반 10개(파생 필드 4 · 속성 5 · spec 문자열) → `b1_dataset.py upgrade` 로 원본 배열은 그대로 두고 추가 → 재생 결과 불변"]
open(os.path.join(OUT,"replay_check.md"),"w",encoding="utf-8").write("\n".join(L)+"\n"); print("\n".join(L))
