# week06 | 토크 예산표(A팀 피드백용, 노션 양식) + Isaac Lab 액추에이터 파라미터표 actuator_params.md (함태훈, B1)
#  입력: results/pick_scenario.json (집기 · 최악 자세 뻗기 · 가감속 민감도), results/kpkv_tune.json (게인),
#        b1_snapshot/v1/results/workspace_max_torque.json (박종진, 정적 최악 = 5주차 ③), inputs/a2_actuators.json (A2 사양)
#  여유율 = 정격 / |τ|max − 1 (5주차 ③ 과 같은 정의).  판정: OK (≥ 20 %) / 주의 (0 ~ 20 %) / 초과 (< 0)
#  출력: results/torque_budget.{md,csv,json}, docs/actuator_params.md
import csv, json, os
import numpy as np
from w06_common import *

P = json.load(open(os.path.join(OUT, "pick_scenario.json"), encoding="utf-8"))
K = json.load(open(os.path.join(OUT, "kpkv_tune.json"), encoding="utf-8"))
W = json.load(open(os.path.join(V1, "results", "workspace_max_torque.json"), encoding="utf-8"))["results"]
PDR = 52.0
PL = [("0kg", "p0", 0.0), ("0.5kg", "p05", 0.5), ("1kg", "p1", 1.0), ("3kg", "p3", 3.0)]


def verdict(mg): return "OK" if mg >= 20 else ("주의" if mg >= 0 else "초과")


rows = []
for case in ("a2_doc", "cad_ri85"):
    pre = "" if case == "a2_doc" else "ri85_"
    for j in ARM:
        act, rt, pk = rated(case, j)
        st = {k: W[k][j]["tau_abs_max"] for k, _, _ in PL}
        pick = {k: P["runs"][pre + r]["tau_max"][j] for k, r, _ in PL}
        reach = {k: max([P["reach"][f"{jw}|{k}|{case}"]["tau_max"][j] for jw in ("shoulder_pitch", "shoulder_roll")]) for k, _, _ in PL}
        worst = {k: max(st[k], pick[k], reach[k]) for k, _, _ in PL}
        mg = {k: (rt / worst[k] - 1) * 100 for k in worst}
        mpdr = {k: (PDR / worst[k] - 1) * 100 for k in worst}
        rows.append(dict(case=case, joint=j, actuator=act, rated=rt, peak=pk, static=st, pick=pick, reach=reach, worst=worst,
                         margin=mg, margin_pdr52=mpdr, verdict={k: verdict(v) for k, v in mg.items()}, verdict_pdr52={k: verdict(v) for k, v in mpdr.items()}))
sens = P["sens"]
s10 = [x for x in sens if x["T"] == 1.0][0]

L = ["# 토크 예산표 (week06, 함태훈 · A팀 피드백용) — 노션 6주차 양식", "",
     "- 모델: B1 v1 (박종진 d6d6304 빌드), 중력보상 B on · week06 게인(`kpkv_tune.md`). 왼팔 기준(오른팔 = 거울, 크기 동일)",
     "- 정적 최악 = 박종진 `workspace_max_torque.json`(= 5주차 ③, 함태훈 `v1_validate_123.py` 와 일치). 집기 = `w06_pick_scenario.py` 집기 동작(테이블 높이 pick_point) 중 |τ| 최대(중력보상분 포함 구동 토크). 뻗기 = 정적 최악 자세로 1.5 s 5차 궤적 뻗기 · 유지 · 복귀 동작 중 최대",
     "- 여유율 = 정격 / max(정적, 집기, 뻗기) − 1 (5주차 ③ 과 같은 정의). 판정 OK ≥ 20 % / 주의 0 ~ 20 % / 초과 < 0. 짐 = 끝점 질점, 중력보상 대상 아님(모르는 짐)", ""]
for case, title in (("a2_doc", "case a2_doc — 어깨 TQ ILM-E85x30 (A2 사양서 10/19, 기본)"), ("cad_ri85", "case cad_ri85 — 8개 모두 RI85 (CAD 대로)")):
    L += [f"## {title}", "",
          "| 관절 | 정격 / 피크 (N·m) | 정적 최악 0 / 0.5 / 1 / 3 kg | 집기 동작 최대 0 / 0.5 / 1.0 / 3 kg | 뻗기 동작 최대 0 / 0.5 / 1 / 3 kg | 여유율 1.0 kg · **3 kg** (정격 대비 %) | 판정 | PDR 52 기준 여유 3 kg |",
          "|---|---|---|---|---|---|---|---|"]
    for r in [x for x in rows if x["case"] == case]:
        f = lambda dct: " / ".join(f"{dct[k]:.2f}" for k, _, _ in PL)
        L.append(f"| {SIDES[0]}_{r['joint']} ({r['actuator']}) | {r['rated']:g} / {r['peak']:g} | {f(r['static'])} | {f(r['pick'])} | {f(r['reach'])} | "
                 f"{r['margin']['1kg']:.0f} · **{r['margin']['3kg']:.0f}** | {r['verdict']['3kg']} | {r['margin_pdr52']['3kg']:.1f} % {r['verdict_pdr52']['3kg']} |")
    L.append("")
mn = min(rows, key=lambda r: r["margin"]["3kg"] if r["case"] == "a2_doc" else 1e9)
mn2 = min(rows, key=lambda r: r["margin"]["3kg"] if r["case"] == "cad_ri85" else 1e9)
L += ["## 요약 · A팀 피드백", "",
      f"- **최소 여유율 (3 kg, 정격 대비)**: a2_doc {mn['joint']} {mn['margin']['3kg']:.0f} % (τ {mn['worst']['3kg']:.2f} / 정격 {mn['rated']:g}), cad_ri85 {mn2['joint']} {mn2['margin']['3kg']:.0f} % (/ {mn2['rated']:g}) → 두 case 모두 OK",
      f"- 1.0 kg(커리큘럼 페이로드) 최소 여유율: a2_doc {min(r['margin']['1kg'] for r in rows if r['case']=='a2_doc'):.0f} %, cad_ri85 {min(r['margin']['1kg'] for r in rows if r['case']=='cad_ri85'):.0f} %",
      f"- 집기 동작(테이블 높이 pick_point) 토크는 정적 최악의 {100*P['runs']['p3']['tau_max']['shoulder_pitch']/W['3kg']['shoulder_pitch']['tau_abs_max']:.0f} %(pitch, 3 kg) — 팔을 내려 쓰는 자세라 작음. 예산을 정하는 것은 팔을 앞으로 · 옆으로 뻗은 정적 최악 자세",
      f"- 뻗기 동작(1.5 s) 동적 최대 = 정적 최악 + {100*(rows[0]['reach']['3kg']/rows[0]['static']['3kg']-1):.1f} % (pitch 3 kg {rows[0]['reach']['3kg']:.2f} N·m). 관성 토크는 1/T² 로 커짐(아래 표)",
      "- **PDR 52 기준(정격 ≥ 최악 × 1.2) 은 정적 기준으로 통과**(5주차 ③ 21.2 %). 동적 뻗기까지 넣으면 pitch 3 kg 여유 "
      f"{rows[0]['margin_pdr52']['3kg']:.1f} %(주의), 관절 속도 한계(3.0 rad/s) 까지 빠르게 뻗으면 {s10['tau_pitch']:.2f} N·m → {100*(PDR/s10['tau_pitch']-1):.1f} %(주의). 실제 정격(TQ 118.8 · RI85 72.6) 대비는 모두 OK",
      "", "### 가감속 민감도 (pitch 최악 자세 · 3 kg, 뻗기 시간 T)", "", "| T (s) | q̇_ref 최고 (rad/s) | 속도 한계 3.0 | |τ_pitch| 최대 (N·m) | 정적 대비 | PDR 52 여유 | TQ 118.8 여유 | RI85 72.6 여유 |", "|---|---|---|---|---|---|---|---|"]
for x in sens:
    L.append(f"| {x['T']:.2f} | {x['qd_ref_max']:.2f} | {'안' if x['within_vel_limit'] else '**밖**'} | {x['tau_pitch']:.2f} | +{100*(x['tau_pitch']/x['static']-1):.1f} % | {100*(PDR/x['tau_pitch']-1):.1f} % | {100*(118.8/x['tau_pitch']-1):.0f} % | {100*(72.6/x['tau_pitch']-1):.0f} % |")
L += ["", "- 정적 최악 자세(팔 수평)에서 끝나는 5차 궤적은 끝점에서 가속도가 0이라, 큰 가속도는 팔이 아래쪽(중력 토크 작음)일 때 생김 → 관성분이 정적 최악에 그대로 더해지지 않음. 그래도 짧게 할수록 커지므로 동작 시간 · 속도 제한을 함께 정해야 함",
      "- 시뮬레이션에 없는 것(구술 ②): 마찰(frictionloss 0) · 백래시(A2 0.002 rad) · 열(연속 정격은 RMS 토크로 판단) · 모터 전류 한계 · 실제 가감속 프로파일 → 위 여유율은 낙관적 추정(실제 여유는 이보다 작아질 수 있음)",
      "", "재생성: `python sim/w06/w06_kpkv_tune.py` → `python sim/w06/w06_pick_scenario.py` → `python sim/w06/w06_torque_budget.py`"]
save_md("torque_budget.md", "\n".join(L) + "\n")
save_json("torque_budget.json", dict(rows=rows, sens=sens, pdr=PDR))
with open(os.path.join(OUT, "torque_budget.csv"), "w", newline="", encoding="utf-8-sig") as fh:
    w = csv.writer(fh); w.writerow(["case", "joint", "actuator", "rated", "peak"] + [f"{t}_{k}" for t in ("static", "pick", "reach", "worst", "margin_pct") for k, _, _ in PL] + ["verdict_3kg", "margin_pdr52_3kg"])
    for r in rows:
        w.writerow([r["case"], r["joint"], r["actuator"], r["rated"], r["peak"]] + [f"{r[t][k]:.3f}" for t in ("static", "pick", "reach", "worst", "margin") for k, _, _ in PL] + [r["verdict"]["3kg"], f"{r['margin_pdr52']['3kg']:.2f}"])

# ---- Isaac Lab 액추에이터 파라미터표 ----
g = K["gains"]; vel = P["vel_limit"]
D = ["# actuator_params.md — B1 v1 팔 관절 ↔ Isaac Lab `ImplicitActuatorCfg` (week06, 함태훈)", "",
     "실습 PC 배정 첫날 `b1/models/v1/isaac/b1_v1_cfg.py`(박종진 초안)의 팔 게인을 이 표 값으로 바꾼다. 값 근거 = `sim/w06/results/kpkv_tune.md` · `torque_budget.md`, A2 사양 = `inputs/a2_actuators.json`.", "",
     "## 1. 관절별 값 (case a2_doc 기본, 오른팔 = 같은 값)", "",
     "| 관절 (`{left,right}_…_joint`) | 기종 | stiffness = kp (N·m/rad) | damping = kv (N·m·s/rad) | effort_limit_sim (N·m) | velocity_limit_sim (rad/s) | armature (kg·m²) | friction | 근거 |",
     "|---|---|---|---|---|---|---|---|---|"]
for j in ARM:
    act, rt, pk = rated("a2_doc", j); arm_ = A2["cases"]["a2_doc"]["classes"][CLS[j]]["armature"]
    D.append(f"| {j} | {act} | {g[j]['kp']:.0f} | {g[j]['kv']:.1f} | {pk:g} (피크) | {vel:.1f} | {arm_:g} | 0.0 | R1 1 kg 처짐 ≤ 1° · R3 계단 OS ≤ 5 % · 정착 ≤ 0.3 s |")
D += ["", "case cad_ri85(8개 RI85)로 확정되면: 어깨 3관절 effort_limit_sim 181.5 · armature 0.098, **kp · kv 는 그대로**(kv 탐색에 두 case 의 Mᵢᵢ 범위를 모두 넣었음 → `kpkv_tune.md` 3절).", "",
      "## 2. MuJoCo ↔ Isaac Lab 대응 · 주의", "",
      "| MuJoCo (b1.xml) | Isaac Lab | 비고 |", "|---|---|---|",
      "| `<position kp>` | `stiffness` | 같은 단위(N·m/rad). Isaac Lab 은 rad 기준 값을 PhysX 로 변환 |",
      "| `<position kv>` (week06 고정값) | `damping` | week05 는 dampratio 1(컴파일 시 qpos0 Mᵢᵢ 로 계산) → Isaac 에는 그런 옵션이 없어 고정 kv 로 튜닝함 |",
      "| joint `damping 0.5` (수동 감쇠) | `damping` 에 더하거나 joint 속성 | ⚠ PhysX 관절에는 드라이브와 별도의 점성 감쇠가 없음 → 0.5 를 damping 에 더해 쓰는 것이 같은 효과(실습 PC 에서 확인) |",
      "| `forcerange` · `actuatorfrcrange` (피크) | `effort_limit_sim` | 정격(TQ 118.8 · RI85 72.6)은 토크 예산 판정용, 시뮬 상한은 피크 |",
      "| URDF `velocity 3.0` | `velocity_limit_sim` | TQ 무부하 star-serial 3.5 rad/s 보다 작음(A2). 집기 · 뻗기 동작 최고 1.96 rad/s |",
      "| joint `armature` | `armature` | = J_rotor × i² (로터만, 기어열 관성 TBD) |",
      "| `frictionloss 0` | `friction` | A2 마찰값 받으면 갱신 |",
      "| 중력보상 B (body `gravcomp=1` + `actuatorgravcomp`) | **없음** → 피드포워드 | ⚠ Isaac Lab 에는 actuatorgravcomp 이 없음. `disable_gravity=True` 는 토크 예산에 중력이 빠짐 → 관절 중력 항을 effort 피드포워드로 넣는 방식 권장(`b1_v1_cfg.py` 도 disable_gravity=False). 실습 PC 에서 API 확인 |",
      "| position 목표 + 속도 피드포워드(`qfrc_applied = kv·q̇_ref`) | joint position target + **velocity target** | 속도 목표를 같이 주면 kv 지연(kv·q̇/kp)이 사라짐: 집기 1 kg 끝점 오차 15.4 → 5.8 mm(짐 FF 까지 2.0 mm) |",
      "", "## 3. 베이스 (참고, 박종진 값 유지)", "", "| 관절 | stiffness | damping | effort_limit_sim |", "|---|---|---|---|",
      "| base_x · base_y | 0 | 200 | 2000 |", "| base_yaw | 0 | 50 | 500 |", ""]
os.makedirs(os.path.join(HERE, "docs"), exist_ok=True)
open(os.path.join(HERE, "docs", "actuator_params.md"), "w", encoding="utf-8").write("\n".join(D) + "\n")
print("\n".join(L))
