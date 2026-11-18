# 함태훈 week09 — PyTorch 없이 박종진 BC 체크포인트(.pt, torch.save zip 형식)를 numpy 로 읽어 같은 추론을 하는 대체 Policy
#   이유: 클라우드 계산 환경은 PyTorch 설치가 막힘(download.pytorch.org 차단). 노트북(torch CPU)에서는 박종진 train_bc.Policy 를 그대로 쓰면 됨
#   구조 = train_bc.MLP: (Linear → ReLU) × layers → Linear, 입력 · 출력 정규화(xm · xs · ym · ys) — 같은 수식, float32
import zipfile,pickle,collections,numpy as np
class _Stor:
    def __init__(s,dtype): s.dtype=dtype
_DT={"FloatStorage":np.float32,"DoubleStorage":np.float64,"LongStorage":np.int64,"IntStorage":np.int32,"HalfStorage":np.float16,"BoolStorage":np.bool_}
def load_pt(path):
    z=zipfile.ZipFile(path); root=[n for n in z.namelist() if n.endswith("/data.pkl")][0][:-len("data.pkl")]
    class U(pickle.Unpickler):
        def find_class(s,mod,name):
            if mod=="torch" and name in _DT: return _DT[name]
            if mod=="torch._utils" and name=="_rebuild_tensor_v2":
                def rb(st,off,size,stride,*a,**k):
                    item=np.dtype(st[1]).itemsize
                    return np.lib.stride_tricks.as_strided(st[0][off:],shape=tuple(size),strides=tuple(x*item for x in stride)).copy()
                return rb
            if mod=="torch._utils" and name=="_rebuild_parameter": return lambda t,*a,**k: t
            return super().find_class(mod,name)
        def persistent_load(s,pid):
            _,dt,key,_loc,n=pid; raw=z.read(root+"data/"+str(key)); return (np.frombuffer(raw,dtype=dt)[:n],dt)
    return U(z.open(root+"data.pkl")).load()

class NumpyPolicy:
    """train_bc.Policy 와 같은 속성 · act() — 박종진 eval_bc.run() 에 그대로 넣을 수 있음"""
    def __init__(s,path):
        ck=load_pt(path); s.cfg=ck["cfg"]; st=ck["state"]
        ks=sorted({k.rsplit(".",1)[0] for k in st},key=lambda k:int(k.split(".")[-1]))
        s.W=[(np.asarray(st[k+".weight"],np.float32),np.asarray(st[k+".bias"],np.float32)) for k in ks]
        s.xm,s.xs,s.ym,s.ys=[np.asarray(ck[k],np.float32) for k in ("xm","xs","ym","ys")]
        s.out=s.cfg["out"]; s.hist=s.cfg.get("hist",""); s.chunk=s.cfg.get("chunk",1)
    def act(s,x):
        h=((np.asarray(x,np.float32)-s.xm)/s.xs)
        for i,(W,b) in enumerate(s.W):
            h=h@W.T+b
            if i<len(s.W)-1: h=np.maximum(h,0)
        return h*s.ys+s.ym
