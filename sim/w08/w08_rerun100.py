# 함태훈 week08 — # 박종진 수집 seed 1000 ~ 1099 를 권장 설정(dt 0.5 ms)으로 다시 실행 → 성공 · 실패 분류 비교 (데이터 저장 없이 판정만)
import sys,os,json,collections
sys.argv=[sys.argv[0]]; import w08_contact_sweep as S
from multiprocessing import Pool
if __name__=="__main__":
    jobs=[("dt 0.5 ms",dict(dt=0.0005),s) for s in range(1000,1100)]
    with Pool(2) as p: R=p.map(S.one,jobs,chunksize=2)
    json.dump(R,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"results","rerun100_dt05.json"),"w"),ensure_ascii=False,indent=1)
    print(sum(r["success"] for r in R),collections.Counter(r["fail"] for r in R if not r["success"]),[r for r in R if r["seed"]==1080][0])
