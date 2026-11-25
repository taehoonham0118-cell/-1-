# 모델 갱신 파이프라인 (week10, 함태훈): config/params.yaml → MJCF 재생성 → pytest → 요약
#   A팀 변경(질량 · 관절 원점 · 구동기 case)이 들어오면: ① params.yaml(또는 박종진 inputs/*.json) 값 · 출처 수정 ② 이 스크립트 한 번 실행
#   ③ 실패한 테스트 = 바뀐 숫자 → 의도한 변경이면 regression 값 갱신 + 로그에 이유 기록
#   하는 일: 박종진 저장소(PJJ)의 b1/ · scenarios/ 를 build/ 로 복사 → build_v1.py 를 B1_A2_CASE=<shoulder_case> 로 실행(원본 저장소는 안 건드림)
#            → pytest tests/ (B1_V1 = 새 빌드) → build/regression_summary.md
#   실행: python tools/rebuild_and_test.py [--params config/params.yaml] [--keep]
import os,sys,shutil,subprocess,argparse,hashlib,datetime
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0,os.path.join(ROOT,"config")); import params as PRM
ap=argparse.ArgumentParser(); ap.add_argument("--params",default=os.path.join(ROOT,"config","params.yaml")); ap.add_argument("--out",default=os.path.join(ROOT,"build")); a=ap.parse_args()
par=PRM.load(a.params); case=par["model"]["shoulder_case"]
PJJ=os.environ.get("PJJ",os.path.join(ROOT,"..","PJJ0337")); B=a.out
if os.path.exists(B): shutil.rmtree(B)
for sub in ("b1","scenarios"): shutil.copytree(os.path.join(PJJ,sub),os.path.join(B,sub))
V1=os.path.join(B,"b1","models","v1"); env=dict(os.environ,B1_A2_CASE=case,PYTHONIOENCODING="utf-8")
r=subprocess.run([sys.executable,os.path.join(V1,"scripts","build_v1.py")],env=env,capture_output=True,text=True,encoding="utf-8")
print(r.stdout[-400:]); assert r.returncode==0,r.stderr[-2000:]
h=lambda p: hashlib.md5(open(p,"rb").read()).hexdigest()[:10]
same=h(os.path.join(V1,"mjcf","b1.xml"))==h(os.path.join(PJJ,"b1","models","v1","mjcf","b1.xml"))
env2=dict(env,B1_V1=V1,PJJ=B,B1_PARAMS=os.path.abspath(a.params))
t=subprocess.run([sys.executable,"-m","pytest",os.path.join(ROOT,"tests"),"-q","-rA","-p","no:cacheprovider"],env=env2,capture_output=True,text=True,encoding="utf-8",cwd=ROOT)
lines=[l for l in t.stdout.splitlines() if l.startswith(("PASSED","FAILED","ERROR"))]; last=t.stdout.strip().splitlines()[-1]
S=["# 회귀 요약 (자동 생성 — tools/rebuild_and_test.py)","",f"- 일시 {datetime.datetime.now():%Y-%m-%d %H:%M} · params `{os.path.relpath(a.params,ROOT)}` · shoulder_case **{case}** · payload_limit {par['limits']['payload_limit']['enable']} · reach_limit {par['limits']['reach_limit']['enable']}",
   f"- 빌드: 박종진 build_v1.py (B1_A2_CASE={case}) · b1.xml md5 {h(os.path.join(V1,'mjcf','b1.xml'))} ({'저장소 원본과 같음' if same else '저장소 원본과 다름 — case/입력 변경'})",f"- pytest: **{last}**","","| 테스트 | 결과 |","|---|---|"]
S+=[f"| {l.split(' ',1)[1].replace('tests/','')} | {l.split(' ')[0]} |" for l in lines]
open(os.path.join(B,"regression_summary.md"),"w",encoding="utf-8").write("\n".join(S)+"\n"); print("\n".join(S))
sys.exit(t.returncode)
