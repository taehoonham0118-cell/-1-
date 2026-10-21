# week05 | A2 구동기 사양서(A2_Actuator_Specs_for_B1.pdf, A2 10/19 작성 · 팀장 10/21 단톡 전달) 영향 분석 (함태훈, B1)
#  공용 모델 값(actuator_params.json · assumptions_b1.json)은 바꾸지 않는다 — 박종진과 합의 후 6주차에 반영 (사양 일부 Partial/TBD)
#  사양 요약 (사양서 §1 · §2 · §3 「Suggested preliminary values」):
#    어깨 = TQ ILM-E85x30 + Wolfrom 38.5:1 → 정격 118.8 · 피크 381.7 N·m, armature(로터만) 0.219 kg·m², 무부하 3.50 rad/s(star-serial) / 7.00(star-parallel)
#    팔꿈치 = RI85-PH KV85 + Wolfrom 38.79:1 → 정격 72.6 · 피크 181.5 N·m, armature(로터만) 0.098 kg·m², 정격 8.21 · 무부하 11.02 rad/s
#  분석: (1) ③ 최악 토크 vs 실제 정격/피크 (2) armature 를 메모리에서 넣었을 때 kv · 실효 감쇠비 · 계단 응답 (3) 모델 가정값(effort 100 · velocity 3.0)과 차이
#  출력: results/a2_spec_impact.md · .json
#  실행: python sim/v1/v1_a2_spec_impact.py
import json, os, re
import numpy as np, mujoco
import v1_common as c

SPEC = {"shoulder": dict(model="TQ ILM-E85x30 + Wolfrom 38.5:1", arm=0.219, rated=118.8, peak=381.7, vel="3.50 (star-serial) / 7.00 (star-parallel), 무부하"),
        "elbow": dict(model="CubeMars RI85-PH KV85 + Wolfrom 38.79:1", arm=0.098, rated=72.6, peak=181.5, vel="8.21 정격 / 11.02 무부하")}
cls = lambda a: "elbow" if a == "elbow" else "shoulder"
A = c.assumptions(); RATED_CRIT = A.get("pdr_torque", {}).get("rated_criterion_Nm", 46.0)
G = json.load(open(os.path.join(c.OUT, "validate_123.json"), encoding="utf-8"))["gravity"]["results"]


def edit(x):
    for cl, s in SPEC.items():
        x = re.sub(rf'(<default class="{cl}">\s*<joint armature=")[^"]*"', rf'\g<1>{s["arm"]}"', x)
    return x


def step_stats(m, a):
    d = mujoco.MjData(m); mujoco.mj_forward(m, d); acts = {m.actuator(k).name: k for k in range(3, m.nu)}
    q0 = d.qpos.copy(); tgt = {n: q0[c.qa(m, f"{n}_joint")] for n in acts}; tgt[f"left_{a}"] += 0.2; y = []
    for k in range(int(1.5 / m.opt.timestep)):
        for n, i in acts.items(): d.ctrl[i] = tgt[n] + d.qfrc_bias[c.da(m, f"{n}_joint")] / m.actuator_gainprm[i][0]
        mujoco.mj_step(m, d); y.append((d.qpos[c.qa(m, f"left_{a}_joint")] - q0[c.qa(m, f"left_{a}_joint")]) / 0.2)
    y = np.array(y); t = np.arange(1, len(y) + 1) * m.opt.timestep; out = np.where(np.abs(y - 1) > 0.02)[0]
    return max(0.0, (y.max() - 1) * 100), float(t[out[-1]]) if len(out) else 0.0


R = {"torque": {}, "dyn": {}}
for a in c.ARM:
    s = SPEC[cls(a)]; tau = G["3kg@ee"][a]["tau"]
    R["torque"][a] = dict(actuator=s["model"], tau3=tau, need=round(1.2 * tau, 2), crit=RATED_CRIT, rated=s["rated"], peak=s["peak"],
                          margin_rated_pct=round((s["rated"] - tau) / tau * 100, 1), crit_ok=bool(1.2 * tau <= RATED_CRIT), rated_ok=bool(RATED_CRIT <= s["rated"]))
for tag, ed in (("armature 0 (현재 모델)", None), ("A2 armature", edit)):
    m = c.load(xml_edit=ed); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    M = np.zeros((m.nv, m.nv)); mujoco.mj_fullM(m, d, M); R["dyn"][tag] = {}
    for a in c.ARM:
        i = m.actuator(f"left_{a}").id; dof = c.da(m, f"left_{a}_joint"); kv = -m.actuator_biasprm[i][2]; I = M[dof, dof]
        os_, ts = step_stats(m, a)
        R["dyn"][tag][a] = dict(armature=float(m.dof_armature[dof]), I=round(I, 4), kv=round(kv, 2), wn=round(float(np.sqrt(300 / I)), 1),
                                zeta=round(float(kv / (2 * np.sqrt(300 * I))), 3), os=round(os_, 1), settle=round(ts, 3))
json.dump(R, open(os.path.join(c.OUT, "a2_spec_impact.json"), "w", encoding="utf-8"), indent=1, ensure_ascii=False)

d0, d1 = R["dyn"]["armature 0 (현재 모델)"], R["dyn"]["A2 armature"]
L = ["# A2 구동기 사양서 영향 분석 (자동 생성 — sim/v1/v1_a2_spec_impact.py)", "",
     "- 출처: `A2_Actuator_Specs_for_B1.pdf` (A2 → B1, 2026-10-19 작성, 팀장 10/21 단톡 전달). 값 상태: Confirmed / Partial / TBD 혼재 — armature 는 **로터만**(기어열 관성 TBD, 실제보다 작음)",
     "- **공용 모델은 아직 그대로**(armature 0 · effort 100 · velocity 3.0). 아래는 메모리에서만 값을 넣은 영향 — 반영은 박종진과 합의 후 6주차", "",
     "## 1. 검증 ③ 최악 중력 토크 vs 기준 · 실제 정격", "",
     f"| 관절 | 구동기 (사양서) | 3 kg 최악 τ [N·m] | × 1.2 | PDR 기준 {RATED_CRIT:g} (팀장 10/19) | 실제 정격 / 피크 [N·m] | 실제 정격 대비 여유 % | 기준 ≤ 실제 정격 |", "|---|---|---|---|---|---|---|---|"]
for a, r in R["torque"].items():
    L.append(f'| {a} | {r["actuator"]} | {r["tau3"]} | {r["need"]} | {"통과" if r["crit_ok"] else "미통과"} | {r["rated"]} / {r["peak"]} | {r["margin_rated_pct"]} | {"OK" if r["rated_ok"] else "NG"} |')
L += ["", "- PDR 판정(기준 52 N·m)은 통과, 그리고 사양서 실제 정격(어깨 118.8 · 팔꿈치 72.6)이 기준 52 를 넘음 → 3 kg 페이로드 정적 유지는 정격 안 (여유 pitch 177 %)",
      "- 단 **CAD(STEP)에는 8개 구동기가 모두 「RI85 제작건」** 으로 그려져 있음 — 사양서는 어깨 = TQ(모터 822 g, RI85 411 g 의 2배). 어깨 링크 질량 · COM 이 바뀌면 τ 도 바뀜 → 팀장 · A1 확인 필요 (어깨 3축 모두 TQ 인지 포함)", "",
      "## 2. armature 를 넣으면 (kp 300 · dampratio 1 그대로, 왼팔 0.2 rad 계단, 다른 관절 유지 · 중력보상)", "",
      "| 관절 | armature [kg·m²] | Mᵢᵢ + armature | kv (dampratio 1 재계산) | ω_n [rad/s] | 실효 감쇠비 ζ_hold | 오버슈트 % | 정착 2 % [s] |", "|---|---|---|---|---|---|---|---|"]
for a in c.ARM:
    x0, x1 = d0[a], d1[a]
    L.append(f'| {a} | 0 → {x1["armature"]:g} | {x0["I"]} → {x1["I"]} | {x0["kv"]} → {x1["kv"]} | {x0["wn"]} → {x1["wn"]} | {x0["zeta"]} → {x1["zeta"]} | {x0["os"]} → {x1["os"]} | {x0["settle"]} → {x1["settle"]} |')
L += ["", "- 가벼운 관절일수록 영향이 큼: shoulder_yaw 는 armature 0.219 가 링크 관성 0.0153 의 **14배** → ω_n 140 → 35.8 rad/s, 정착 0.04 → 0.16 s. 감속기 관절은 armature 가 동특성을 지배",
      "- dampratio 라 kv 가 자동으로 커져(pitch 20.5 → 29.8) 오버슈트는 오히려 줄어듦 — kv 를 고정값으로 쓰면 재튜닝 필요 (armature_study.md 와 같은 결론). 6주차 Isaac `ImplicitActuatorCfg` 로 옮길 kv 도 armature 반영 후 값이어야 함",
      "- 로터만이라 실제 armature 는 더 큼(기어열 TBD) → 감쇠비 · 정착시간은 이보다 더 느린 쪽", "",
      "## 3. 공용 가정값과 사양서 차이 (6주차 반영 후보)", "",
      "| 항목 | 공용 가정값 (C4) | 사양서 | 비고 |", "|---|---|---|---|",
      "| armature | 0 | 어깨 0.219 · 팔꿈치 0.098 (로터만) | 사양서 §3 권장 시뮬 값 |",
      "| effort (= 피크, forcerange) | 100 | 어깨 381.7 · 팔꿈치 181.5 (피크) / 118.8 · 72.6 (정격) | 피크를 쓰면 포화가 거의 안 걸림 → 정격 · 피크 중 무엇을 forcerange 로 쓸지 합의 필요 |",
      "| velocity | 3.0 rad/s | 어깨 3.50 (star-serial, 무부하) / 7.00 · 팔꿈치 8.21 정격 | 어깨는 권선 미정 — 3.0 은 보수적으로 유지 가능 |",
      "| frictionloss · damping | 0 · 0.5 | TBD (다이나모 시험 후) | 그대로 |",
      "| 백래시 | – | 팔꿈치 2.2e-3 (최악 2.9e-3) · 어깨 2.0e-3 rad | MuJoCo 기본 모델엔 없음 — 6주차 구술 Q2(실제 모터 문제) 근거 |"]
open(os.path.join(c.OUT, "a2_spec_impact.md"), "w", encoding="utf-8").write("\n".join(L) + "\n")
print("\n".join(L))
