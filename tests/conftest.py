# pytest 공통 (week10, 함태훈) — 모델 = B1_V1(빌드 폴더의 b1/models/v1) 또는 박종진 저장소 원본, 설정 = config/params.yaml(B1_PARAMS)
#   실행: python tools/rebuild_and_test.py   (재빌드 + 테스트 한 번에)  또는  pytest tests/ -q   (원본 모델 그대로)
import os,sys,pytest
ROOT=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PJJ=os.environ.get("PJJ",os.path.join(ROOT,"..","PJJ0337")); os.environ["PJJ"]=PJJ
V1=os.environ.get("B1_V1",os.path.join(PJJ,"b1","models","v1"))
for p in (os.path.join(ROOT,"config"),os.path.join(ROOT,"sim","w10"),os.path.join(V1,"scripts")): sys.path.insert(0,p)
import params as PRM
@pytest.fixture(scope="session")
def par(): return PRM.load()
@pytest.fixture(scope="session")
def scene(): return os.path.join(V1,"mjcf","scene.xml")
@pytest.fixture(scope="session")
def arm():
    import w10_common as C; A=C.Arm(); Q=A.sample(3000,0); T=[A.terms(q) for q in Q]
    import numpy as np
    return dict(C=C,A=A,Q=Q,TG=np.array([t[0] for t in T]),SZ=np.array([t[1] for t in T]),RHO=np.array([t[4] for t in T]))
