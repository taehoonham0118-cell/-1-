# 함태훈 week09 — bc_eval_pjj{,_oat}.json → 팀 연계 결과표 results/bc_eval_pjj.md (박종진 BC 실패 ↔ 민감 파라미터)
import os,json,collections,numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); RES=os.path.join(HERE,"results")
A=json.load(open(os.path.join(RES,"bc_eval_pjj.json"))); O=json.load(open(os.path.join(RES,"bc_eval_pjj_oat.json")))
FN={"grasp_miss":"못 집음","table_collision":"테이블 충돌","sim_unstable":"수치 발산","timeout_holding":"들고 시간 초과","place_miss":"목표 밖","drop":"떨어뜨림","tipped":"넘어짐","ik_fail":"계획 실패(IK)","slip":"미끄럼"}
def g(rows,key): 
    G=collections.defaultdict(list)
    for r in rows: G[key(r)].append(r)
    return G
fs=lambda v: " · ".join(f"{FN.get(k,k)} {n}" for k,n in collections.Counter(r["fail"] for r in v if not r["success"]).most_common()) or "–"
lift=lambda v: sum(r.get("rise_max_mm",100)>=30 for r in v)
G=g(A["rows"],lambda r:(r["who"],r["tag"])); pol=A["policy"]; N=len(G[("expert","없음")])
nomB={r["seed"]:r["success"] for r in G[(pol,"없음")]}
md=[f"# 팀 연계: 박종진 BC × 함태훈 랜덤화 — 실패 ↔ 민감 파라미터 (week09)\n",
f"- 정책 = 박종진 **{A['pjj']}** `policies/ckpt/bc_{pol}.pt` (DART + 액션 청킹 25, 관측 39 · 로봇 좌표), 롤아웃 · 판정 = 박종진 `eval_bc.run()` 그대로(34 s, 같은 실패 분류)",
f"- 환경 = 함태훈 `randomize.py` (+ 물체 질량 배율 · 측정 편향 · 명령 지연을 `bc_eval_pjj.py` 가 감싸서 적용), 평가 seed {A['seeds'][0]} ~ {A['seeds'][1]} (박종진과 같음), mujoco {A['mujoco']}",
"- 정책 추론 = PyTorch 없이 같은 MLP 를 numpy 로(`pt_numpy.py`). 확인: 랜덤화 없음 BC 18/50 = 박종진 eval.md 18/50 (36 %) 과 같은 성공률(mujoco 3.14 ↔ 3.15 라 seed 별 결과는 일부 다름)\n",
"## 1. 전문가 vs BC — 동시 랜덤화 · Top 3 (seed 50개)\n","| 조건 | 전문가 | BC | BC 들어 올린 수 | BC 실패 분류 | BC 공칭 대비 (잃음 / 얻음) |","|---|---|---|---|---|---|"]
for t in ("없음","동시 ×1","동시 ×2","테이블 +10 mm","테이블 −10 mm","μ 0.5","편향 10 mm"):
    e,b=G[("expert",t)],G[(pol,t)]; lost=sum(nomB[r["seed"]] and not r["success"] for r in b); gain=sum((not nomB[r["seed"]]) and r["success"] for r in b)
    md.append(f"| {t} | {sum(r['success'] for r in e)}/{N} | **{sum(r['success'] for r in b)}/{N}** | {lift(b)} | {fs(b)} | {'–' if t=='없음' else f'{lost} / {gain}'} |")
import sys; sys.argv=[sys.argv[0]]; sys.path.insert(0,HERE); import randomize as R
vj=G[(pol,"동시 ×1")]; lat=np.array([R.sample_joint(r["seed"],1.0)["latency"]*1e3 for r in vj]); lf=np.array([r["rise_max_mm"]>=30 for r in vj])
lat_txt=" · ".join(f"{a:g} ~ {min(b,20):g} ms {int(lf[(lat>=a)&(lat<b)].sum())}/{int(((lat>=a)&(lat<b)).sum())}" for a,b in ((0,5),(5,10),(10,20.01)))
GO=g(O["rows"],lambda r:r["tag"]); n0=len(GO["없음"]); s0=sum(r["success"] for r in GO["없음"])
rows=[]
for t,v in GO.items():
    if t=="없음": continue
    k,val=t.split("="); rows.append((k,val,sum(r["success"] for r in v),v))
by=collections.defaultdict(list)
for k,val,s,v in rows: by[k].append((val,s,v))
rank=sorted(by.items(),key=lambda kv:min(s for _,s,_ in kv[1]))
md+=["",f"## 2. BC 의 민감도 (하나씩, ×1 범위 양 끝, seed {O['seeds'][0]} ~ {O['seeds'][1]}) — 공칭 {s0}/{n0}\n","| BC 순위 | 파라미터 | 하한 → 성공 | 상한 → 성공 | 최저 쪽 실패 분류 · 들어 올린 수 | 전문가 순위(randomize.md) |","|---|---|---|---|---|---|"]
ex_rank={k:i+1 for i,k in enumerate(json.load(open(os.path.join(RES,"sensitivity.json")))["rows"] and [r["k"] for r in sorted([r for r in json.load(open(os.path.join(RES,"sensitivity.json")))["rows"] if r["grp"]!="수치"],key=lambda r:(-r["d2"],-r["d1"],-r["dge"]))])}
for i,(k,vs) in enumerate(rank,1):
    vs=sorted(vs,key=lambda x:float(x[0])); worst=min(vs,key=lambda x:x[1]); b="**" if i<=3 else ""
    lo=vs[0]; hi=vs[-1] if len(vs)>1 else None
    md.append(f"| {b}{i}{b} | {b}`{k}`{b} | {lo[0]} → {lo[1]}/{n0} | {(hi[0]+' → '+str(hi[1])+'/'+str(n0)) if hi else '–'} | {fs(worst[2])} · 들어 올림 {lift(worst[2])} | {ex_rank.get(k,'–')} |")
md+=["",
"## 3. 실패 ↔ 민감 파라미터 (결론)\n",
"| BC 실패 | 원인 파라미터 | 근거 (수치) | 전문가는 왜 견디나 | 대책 (박종진 BC · 데이터) |","|---|---|---|---|---|",
"| 팔 자세 바꾸는 경유점에서 멈춤 → 못 집음 | **명령 지연 20 ms (제어 1주기)** | 0/25 · 들어 올림 0. seed 2001 궤적: 끝점 높이 ≈ 0.70 m 에서 멈추고 베이스 명령 ±0.04 m/s 로 진동(지연 없으면 0.88 m 까지 올리고 테이블로 진행) | 전문가는 내부 기준 궤적 + 측정 피드백 → 지연 40 ms 도 40/40 | 시연 수집 · DART 에 지연 0 ~ 20 ms 랜덤화, 관측에 직전 명령 추가 |",
"| 들고 시간 초과 · 목표 밖 · 못 집음 | **테이블 높이 −10 mm** (낮아짐) | 2/50 (공칭 18, 잃음 16 · 얻음 0), oat 0/25 | 전문가는 PLACE 에서 물체 ↔ 테이블 접촉을 보고 놓음, −10 mm 49/50 | 시연을 테이블 높이 ±10 mm 랜덤으로 다시 수집(학습 데이터 높이 1개 → 관측 obj_z 가 분포 밖), 접촉 · 손가락 힘 관측 추가 |",
"| 〃 | 테이블 +10 mm | 14/50 (들고 시간 초과 13) | 여유 11 · 16 mm 안 | 〃 |",
"| 떨어뜨림 · 들고 시간 초과 · 테이블 충돌 | 팔 링크 질량 −10 %, 베이스 kv +20 % / 하체 질량 −20 % | 4 · 4 · 5/25 | 전문가 40/40 (피드백) | 질량 · 베이스 게인 랜덤화 시연 |",
"| 변화 없음 | μ 0.5 · 편향 10 mm · 파지력 · 손가락 질량 · armature · 감쇠 · kp | 17 ~ 19/50, oat 7 ~ 10/25 (공칭 8) | — | 전문가 Top 2 · 3(μ · 편향)은 BC 실패 원인이 아님 — BC 는 그 전에 다른 이유로 실패 |",
"",
"- 정리: 전문가 Top 3(테이블 높이 · μ · 편향)와 BC Top 3(**지연 · 테이블 높이(특히 낮을 때) · 팔 질량 / 베이스 게인**)가 다름. 겹치는 건 테이블 높이 하나. BC 는 피드백이 아니라 「시연에서 본 상태 → 명령」을 외워서, **학습 데이터에 없던 동역학 차이(지연 · 질량)와 관측 분포 이동(테이블 높이)** 에 약함 = 공변량 이동이 랜덤화로 드러난 것",
f"- 동시 ×1 에서 BC {sum(r['success'] for r in G[(pol,'동시 ×1')])}/{N} (들어 올림 {lift(G[(pol,'동시 ×1')])}) — 표본의 지연별 들어 올린 수: "+lat_txt+" → 지연 10 ms 이상이면 하나도 못 듦. 박종진 임시 rand_dyn(지연 · 테이블 · 팔 질량 없음)에서 24 % 였던 차이의 주원인",
"- 제안: ① 시연 · DART 수집을 랜덤화 환경(`randomize.apply` + 테이블 · 지연)에서 ② 관측에 직전 명령 · 손가락 접촉 추가 ③ 평가는 이 표 조건 그대로(`bc_eval_pjj.py`)",
]
open(os.path.join(RES,"bc_eval_pjj.md"),"w").write("\n".join(md)+"\n"); print("\n".join(md))
