# 함태훈 week10 팀 연계 — 박종진 「세 방안 비교표」의 최소 여유율 열 검산 도구
#   입력: 시나리오 기록(npz/h5 의 left 팔 qpos 시계열 또는 구동 토크) + 짐 질량 + case → 관절별 필요(정적 + 베이스 설계 가속) 최대 · 최소 여유율
#   사용: python sim/w10/w10_check_margin.py <파일.h5|npz> --mass 3 --case cad_ri85   (h5 = dataset_spec v1.x 형식, obs/qpos 의 팔 4열)
#         인자 없이 실행하면 자기 점검: 앞으로 수평 자세 1개 → cad_ri85 pitch 여유율 출력
import os,sys,argparse,numpy as np
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE); import w10_common as C
def check(qs,mass,case,use_add=True):
    A=C.Arm(); best={}
    for q in qs:
        tg,sz,ar,ap,rho,_=A.terms(q); n=np.abs(tg+mass*sz)+(np.array([C.ADD_W07[a] for a in C.ARM]) if use_add else 0)
        for i,a in enumerate(C.ARM): best[a]=max(best.get(a,0),float(n[i]))
    return {a:dict(need=round(v,2),rated=C.RATED[C.CASES[case][a]],margin=round(C.margin(C.RATED[C.CASES[case][a]],v),1)) for a,v in best.items()}
if __name__=="__main__":
    ap_=argparse.ArgumentParser(); ap_.add_argument("file",nargs="?"); ap_.add_argument("--mass",type=float,default=3.0); ap_.add_argument("--case",default="a2_doc"); ap_.add_argument("--cols",default="3,4,5,6")
    a=ap_.parse_args(); cols=[int(x) for x in a.cols.split(",")]
    if not a.file: qs=[np.array([-np.pi/2,0,0,0])]
    elif a.file.endswith(".npz"): qs=np.load(a.file)["qpos"][:,cols]
    else:
        import h5py; qs=h5py.File(a.file,"r")["obs/qpos"][:][:,cols]
    for k,v in check(qs,a.mass,a.case).items(): print(k,v)
