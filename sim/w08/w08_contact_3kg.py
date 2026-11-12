# 함태훈 week08 — # 짐 3.0 kg 고정(최악 질량) 집기 20회: 접촉 설정 후보별 성공률 · 실패 분류 · 계산 시간
import sys,os,json
sys.argv=[sys.argv[0]]; import w08_contact_sweep as S
import pick_env as P, expert as E
from multiprocessing import Pool
_st=P.sample_task
def t3(seed): t=_st(seed); t["mass"]=3.0; return t
P.sample_task=t3; E.sample_task=t3
C={k:v for k,v in S.CFG}
CF=[("기준(박종진 임시값)",C["기준(박종진 임시값)"]),("dt 0.5 ms",C["dt 0.5 ms"]),("dampratio 0.7",C["dampratio 0.7"]),("dt 0.5 ms + dampratio 0.7",dict(dt=0.0005,solref=(0.01,0.7)))]
if __name__=="__main__":
    jobs=[(n,c,s) for n,c in CF for s in range(1000,1020)]
    with Pool(2) as p: R=p.map(S.one,jobs,chunksize=1)
    json.dump(R,open(os.path.join(os.path.dirname(os.path.abspath(__file__)),"results","contact_3kg.json"),"w"),ensure_ascii=False,indent=1)
