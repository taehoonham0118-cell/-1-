"""v1 모델 로더 — Windows 한글 경로 대응 (v0_mjio.py와 같은 방식: xml · STL을 메모리 assets로 전달, include 포함)
사용: python v1_mjio.py ../mjcf/scene.xml            → 로드 확인 후 뷰어 (액추에이터 슬라이더 = Control 패널)
      python v1_mjio.py ../urdf/b1_v1.urdf --noview
"""
import os,sys,glob,mujoco
def _assets(path):
    root=os.path.dirname(os.path.dirname(os.path.abspath(path)))   # v1 폴더
    a={}
    for f in glob.glob(os.path.join(root,"mjcf","*.xml")): a[os.path.basename(f)]=open(f,"rb").read()
    for f in glob.glob(os.path.join(root,"meshes","**","*.stl"),recursive=True):   # week06: meshes/visual · meshes/collision (키 = meshdir 기준 상대 경로, / 구분)
        a[os.path.relpath(f,os.path.join(root,"meshes")).replace(os.sep,"/")]=open(f,"rb").read()
    return a
def load_model(path,strip_gravcomp=False):
    """strip_gravcomp=True: b1.xml의 gravcomp · actuatorgravcomp를 뺀 모델 (week06 기본 빌드는 중력보상 on → 보상 없는 비교용)"""
    a=_assets(path)
    if strip_gravcomp and "b1.xml" in a:
        import xml.etree.ElementTree as ET
        r=ET.fromstring(a["b1.xml"].decode("utf-8"))
        for b in r.iter("body"): b.attrib.pop("gravcomp",None)
        for j in r.iter("joint"): j.attrib.pop("actuatorgravcomp",None)
        a["b1.xml"]=ET.tostring(r,encoding="unicode").encode("utf-8")
    return mujoco.MjModel.from_xml_string(open(path,encoding="utf-8").read(),assets=a)
if __name__=="__main__":
    m=load_model(sys.argv[1]); print("loaded",os.path.basename(sys.argv[1]),"nq",m.nq,"nu",m.nu,"nsensor",m.nsensor,"mass %.3f"%sum(m.body_mass))
    if "--noview" not in sys.argv:
        import mujoco.viewer; mujoco.viewer.launch(m)
