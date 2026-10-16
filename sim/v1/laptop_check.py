# week05 | 노트북 재현 확인: 클라우드 결과(results/)와 노트북에서 다시 계산한 값(results_laptop/)을 비교 (함태훈, B1)
# 실행 (5주차_작업 폴더 = 저장소 루트에서):  py sim/v1/laptop_check.py
#   1) v1_validate_123.py · v1_actuator_table.py · v1_mirror_actuator_check.py 를 출력 폴더만 바꿔(results_laptop/) 다시 실행 — 클라우드 결과는 덮어쓰지 않음
#   2) 핵심 수치를 나란히 놓고 차이를 표로 → results_laptop/laptop_compare.md (보고서 · 로그에 「노트북 재현」 근거)
import csv, json, os, platform, subprocess, sys
import mujoco, numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REF = os.path.join(HERE, "results"); NEW = os.path.join(HERE, "results_laptop"); os.makedirs(NEW, exist_ok=True)
env = dict(os.environ, V1_OUT=NEW, V1_IMG=NEW, PYTHONIOENCODING="utf-8")
for s in ("v1_actuator_table.py", "v1_validate_123.py", "v1_mirror_actuator_check.py"):
    print(f"[laptop_check] {s} 실행 중 …", flush=True)
    r = subprocess.run([sys.executable, os.path.join(HERE, s)], env=env, capture_output=True, text=True, encoding="utf-8")
    open(os.path.join(NEW, s.replace(".py", "_run_log.txt")), "w", encoding="utf-8").write(r.stdout + r.stderr)
    if r.returncode: print(r.stderr[-2000:]); sys.exit(f"{s} 실패 — results_laptop/ 로그 확인")

a, b = json.load(open(os.path.join(REF, "validate_123.json"), encoding="utf-8")), json.load(open(os.path.join(NEW, "validate_123.json"), encoding="utf-8"))
rows = [("총질량 [kg]", a["mass"]["total"], b["mass"]["total"]),
        ("손계산 앞 수평 0 kg [N·m]", a["hand_horizontal"]["tau0"], b["hand_horizontal"]["tau0"]),
        ("MuJoCo 앞 수평 0 kg [N·m]", a["hand_horizontal"]["mj0"], b["hand_horizontal"]["mj0"]),
        ("② 앞 수평 보상 없음 [°]", a["hold"]["앞으로 수평|없음"]["final_err_deg"], b["hold"]["앞으로 수평|없음"]["final_err_deg"]),
        ("② roll 상한 오프셋(A) [°]", a["hold"]["옆으로 벌림(roll 상한)|오프셋(A)"]["final_err_deg"], b["hold"]["옆으로 벌림(roll 상한)|오프셋(A)"]["final_err_deg"]),
        ("② roll 상한 gravcomp(B) [°]", a["hold"]["옆으로 벌림(roll 상한)|gravcomp(B)"]["final_err_deg"], b["hold"]["옆으로 벌림(roll 상한)|gravcomp(B)"]["final_err_deg"])]
for j in ("shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"):
    rows.append((f"③ {j} 3 kg 최악 [N·m]", a["gravity"]["results"]["3kg@ee"][j]["tau"], b["gravity"]["results"]["3kg@ee"][j]["tau"]))
ka = {r["joint"]: r for r in csv.DictReader(open(os.path.join(REF, "actuator_table.csv"), encoding="utf-8-sig"))}
kb = {r["joint"]: r for r in csv.DictReader(open(os.path.join(NEW, "actuator_table.csv"), encoding="utf-8-sig"))}
for j in ka:
    rows.append((f"kv {j.split('_', 1)[1].replace('_joint', '')}", float(ka[j]["kv_compiled"]), float(kb[j]["kv_compiled"])))
mir = open(os.path.join(NEW, "mirror_actuator_check.md"), encoding="utf-8").read()
ok_mir = "전부 OK" in mir

L = ["# 노트북 재현 확인 (자동 생성 — sim/v1/laptop_check.py)", "",
     f"- 노트북: {platform.system()} {platform.release()} · Python {platform.python_version()} · MuJoCo {mujoco.__version__} · numpy {np.__version__}",
     f"- 기준: 클라우드 results/ (Linux · Python 3.13 · MuJoCo {a['model']['mujoco']})", "",
     "| 항목 | 클라우드 | 노트북 | 차 |", "|---|---|---|---|"]
worst = 0.0
for n, x, y in rows:
    dlt = abs(float(x) - float(y)); worst = max(worst, dlt / max(abs(float(x)), 1e-9))
    L.append(f"| {n} | {x} | {y} | {dlt:.2e} |")
L += ["", f"- 미러 점검(노트북): {'17/17 전부 OK' if ok_mir else 'NG 있음 — mirror_actuator_check.md 확인'}",
      f"- 최대 상대 차: {worst:.2e} → {'**재현 일치**' if worst < 1e-3 and ok_mir else '**차이 있음 — 확인 필요**'}"]
open(os.path.join(NEW, "laptop_compare.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
print("\n".join(L))
