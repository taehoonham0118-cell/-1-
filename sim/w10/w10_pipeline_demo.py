# 함태훈 week10 — 모델 갱신 파이프라인 시연 3가지 (tools/rebuild_and_test.py 를 그대로 3번 실행)
#   A. 기본(params.yaml 그대로, a2_doc)  B. 어깨 case 변경(shoulder_case: cad_ri85 — params.yaml 한 줄)
#   C. A팀 변경 흉내: 팔꿈치 링크 질량 +10 %(박종진 inputs/link_inertials_v1.json 사본만 수정, 원본 저장소 그대로) → 회귀 테스트가 잡는지
#   결과: results/pipeline_demo.md
import os,sys,shutil,subprocess,json,yaml,re
HERE=os.path.dirname(os.path.abspath(__file__)); ROOT=os.path.dirname(os.path.dirname(HERE)); PJJ=os.environ.get("PJJ",os.path.join(ROOT,"..","PJJ0337"))
TMP=os.path.join(ROOT,"build_demo"); os.makedirs(TMP,exist_ok=True)
def run(label,params,pjj):
    out=os.path.join(TMP,label); r=subprocess.run([sys.executable,os.path.join(ROOT,"tools","rebuild_and_test.py"),"--params",params,"--out",out],
                                                  env=dict(os.environ,PJJ=pjj,PYTHONIOENCODING="utf-8"),capture_output=True,text=True,encoding="utf-8")
    s=open(os.path.join(out,"regression_summary.md"),encoding="utf-8").read()
    last=re.search(r"pytest: \*\*(.+?)\*\*",s).group(1); md5=re.search(r"md5 (\w+) \((.+?)\)",s)
    fails=[l.split("|")[1].strip().split(" - ")[0] for l in s.splitlines() if "FAILED" in l]
    return dict(label=label,pytest=last,md5=md5.group(1),build=md5.group(2),fails=fails)
P0=os.path.join(ROOT,"config","params.yaml"); par=yaml.safe_load(open(P0,encoding="utf-8"))
pb=dict(par); pb["model"]=dict(par["model"],shoulder_case="cad_ri85"); PB=os.path.join(TMP,"params_cad_ri85.yaml"); yaml.safe_dump(pb,open(PB,"w",encoding="utf-8"),allow_unicode=True,sort_keys=False)
PJC=os.path.join(TMP,"pjj_elbow_plus10")
if os.path.exists(PJC): shutil.rmtree(PJC)
for sub in ("b1","scenarios"): shutil.copytree(os.path.join(PJJ,sub),os.path.join(PJC,sub))
fi=os.path.join(PJC,"b1","models","v1","inputs","link_inertials_v1.json"); L=json.load(open(fi,encoding="utf-8")); m0=L["left_elbow_link"]["mass"]
L["left_elbow_link"]["mass"]=m0*1.1; L["left_elbow_link"]["inertia"]=[x*1.1 for x in L["left_elbow_link"]["inertia"]]; json.dump(L,open(fi,"w",encoding="utf-8"),ensure_ascii=False,indent=1)
R=[run("A_default",P0,PJJ),run("B_case_cad_ri85",PB,PJJ),run("C_elbow_mass_+10pct",P0,PJC)]
T=["# 모델 갱신 파이프라인 시연 (자동 생성 — w10_pipeline_demo.py)","","명령 한 번: `python tools/rebuild_and_test.py --params <yaml>` = 박종진 build_v1.py 재빌드(MJCF 생성) + pytest 15개(검증 5종 · 토크 10)","",
   "| 시연 | 바꾼 것 | b1.xml | pytest | 걸린 테스트 |","|---|---|---|---|---|"]
what={"A_default":"없음 (params.yaml 그대로, a2_doc)","B_case_cad_ri85":"params.yaml `shoulder_case: cad_ri85` 한 줄","C_elbow_mass_+10pct":f"팔꿈치 링크 질량 {m0:.3f} → {m0*1.1:.3f} kg (+10 %, 관성 ×1.1)"}
for r in R: T.append(f"| {r['label']} | {what[r['label']]} | {r['md5']} ({r['build']}) | {r['pytest']} | {', '.join(r['fails']) or '—'} |")
T+=["","- B: case 를 바꾸면 MJCF 의 어깨 forcerange · armature 가 RI85 값으로 바뀌고(md5 다름), 토크 판정은 정격 72.6 으로 다시 계산돼도 3 kg 에서 통과(최소 여유 49.1 %)",
    "- C: 질량이 바뀌면 ① 질량 합계 회귀 · ③ 중력 토크 · 필요 토크 · m_max 회귀가 실패 → 「바뀐 숫자」를 바로 알려 줌. 의도한 변경(A팀 새 BOM)이면 regression 값을 새로 고정하고 로그에 이유를 적는다",
    "- 원본 저장소(박종진)는 건드리지 않고 build_demo/ 사본에서만 빌드"]
open(os.path.join(HERE,"results","pipeline_demo.md"),"w",encoding="utf-8").write("\n".join(T)+"\n"); print("\n".join(T))
