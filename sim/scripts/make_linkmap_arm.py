# week04 | 「양팔로봇 최종버전.stp」 링크 분할표(linkmap) 생성 (함태훈, B1)
# 근거: find_joint_axes.py 결과(RI85 출력부 원통 축 · 접촉 부품). 실행: python sim/scripts/make_linkmap_arm.py
#   → sim/robot/linkmap_arm.json  (step_to_links.py --linkmap 입력)
# 규칙
#   - 링크 = 서로 상대 운동이 없는 부품 묶음. 이름 = ICD {side}_{부위}_{운동}_link
#   - RI85 구동기는 둘로 나눔: 출력과 같이 도는 부품(output-v1 · ring2 기어 · cover plate1 = 접촉 확인) → 자식 링크,
#     나머지(하우징 · 모터 · 1단 링기어 · 캐리어 · 유성기어 · 베어링) → 부모 링크. 캐리어 · 유성기어는 중간 속도로 돌지만 v0 에서는 부모에 둠
#     (모터 · 기어의 회전 관성은 armature = J_rotor·N² 로 따로 넣는 값 — RI85 사양 요청 중)
#   - 링크 프레임: 원점 = 그 링크를 움직이는 관절의 축 위 점(RI85 출력 원통 R52 의 축 기준점), 자세 = 로봇 좌표계(REP-103: x 전방 · y 좌 · z 상)
#     CAD → 로봇 축 가정: 로봇 x = CAD +z, 로봇 y = CAD +x, 로봇 z = CAD +y (Inventor Y-up, 팔이 -y 로 늘어진 자세) → rpy = (-π/2, -π/2, 0)
#     ※ 전방(±z) · 좌우는 가정 — 박종진 ICD · 팀장 확인 대상
import json, math, os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RPY = [-math.pi / 2, -math.pi / 2, 0.0]
STEEL, PCB = 7850, 1850
OUT_GRP = r"(output-v1|ring2-InternalInvoluteGear|cover plate1:)"
act_out = lambda i: rf"/RI85 제작건:{i}/{OUT_GRP}"
act_stator = lambda i: rf"/RI85 제작건:{i}/(?!{OUT_GRP})"
top = lambda name, k: rf"^[^/]+/{name}:{k}/"          # 최상위 인스턴스만 (RI85 내부 같은 이름 부품 제외)

# CAD 관절점 [mm] (find_joint_axes.py, 10/13)
J = {"left": {"J1": [29.46, 0, 0], "J2": [159.46, -20.0, 19.0], "J3": [159.47, -180.15, -1.66], "J4": [159.57, -295.61, 16.09]},
     "right": {"J1": [-265.74, 0, 0], "J2": [-395.74, -20.0, 19.0], "J3": [-388.62, -179.66, -1.64], "J4": [-388.54, -295.12, 16.11]}}
# 측 별 인스턴스 번호 (CAD x>0 = left 가정)
S = {"left":  dict(a=[1, 2, 3, 4], sb="어깨 브래킷1", sb2="어깨 브래킷2", k=1, tube=2, drv=[1, 3, 5], brg=[1, 2]),
     "right": dict(a=[6, 5, 7, 8], sb="어깨 브래킷1\\(반대\\)", sb2="어깨 브래킷2\\(반대\\)", k=2, tube=1, drv=[2, 4, 6], brg=[3, 4])}

links = [{"name": "torso_link", "parts": [act_stator(1), act_stator(6)],
          "origin_mm": [(J["left"]["J1"][0] + J["right"]["J1"][0]) / 2, 0, 0], "rpy": RPY,
          "note": "STEP 에 몸통 없음 — 어깨 RI85 두 개의 고정부만. 몸통 본체는 URDF 에서 임시 box"}]
for side, s in S.items():
    a1, a2, a3, a4 = s["a"]; k = s["k"]
    links += [
        {"name": f"{side}_shoulder_pitch_link", "origin_mm": J[side]["J1"], "rpy": RPY,
         "parts": [top(s["sb"], 1), top(s["sb2"], 1), top("모터드라이버 예상", s["drv"][0]),
                   top("JIS B 1521 6810 50x65x7", s["brg"][0]), act_out(a1), act_stator(a2)]},
        {"name": f"{side}_shoulder_roll_link", "origin_mm": J[side]["J2"], "rpy": RPY,
         "parts": [top("C브래킷1\\(수정본\\)", k), top("C브래킷2\\(수정본\\)", k), top("상완원통 일체형", s["tube"]),
                   top("모터드라이버 예상", s["drv"][1]), act_out(a2), act_stator(a3)]},
        {"name": f"{side}_shoulder_yaw_link", "origin_mm": J[side]["J3"], "rpy": RPY,
         "parts": [top("상완 칼라1", k), top("상완 칼라2", k),
                   top("JIS B 1521 6810 50x65x7", s["brg"][1]), act_out(a3), act_stator(a4)]},
        {"name": f"{side}_elbow_link", "origin_mm": J[side]["J4"], "rpy": RPY,
         "parts": [top("전완요크1", k), top("전완요크2", k), top("그리퍼 플랜지", k),
                   top("모터드라이버 예상", s["drv"][2]), act_out(a4)]},   # 드라이버 5·6 = 전완요크2 와 0 mm 접촉 (10/14 박종진 분할표와 대조 후 수정)
    ]
for L in links:
    L["density_by_part"] = {r"InvoluteGear|planet shaft|/key:|JIS B 1521|ISO 3245|50305040210|MB039|Prirobnica": STEEL,
                            r"encoder plate": PCB}
    L["mesh_exclude"] = [r"/RI85 제작건:\d+/(?!housing|RI85-PH|output-v1|encoder cover|cover plate1:)", r"^[^/]+/JIS B 1521 6810"]
    L["bom_mass_kg"] = None
lm = {"step": "양팔로봇 최종버전.stp (팀장 10/13 카톡, 원본은 저장소 밖)",
      "density_default": 2700,
      "density_note": "가정: 브래킷 · 하우징 · 출력부 = 알루미늄 2700, 기어 · 축 · 키 · 베어링 · 모터 = 강 7850, 엔코더 기판 1850. BOM 수령 시 교체",
      "links": links}
os.makedirs(os.path.join(ROOT, "robot"), exist_ok=True)
p = os.path.join(ROOT, "robot", "linkmap_arm.json")
json.dump(lm, open(p, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
print("저장:", p, f"링크 {len(links)}개")
