# week04 | 검증용 테스트 STEP 만들기 (함태훈, B1)
# 목적: 우리 로봇 STEP이 오기 전에 「STEP → 링크별 STL · 질량 · COM · 관성」 파이프라인을
#       해석해(손계산 값)가 있는 단순 형상으로 먼저 검증한다.
# 실행: python sim/scripts/make_test_step.py            (저장소 루트에서, OpenCascade 필요: pip install cadquery-ocp)
# 출력: sim/test_cad/test_arm.step  (AP214, 단위 mm — 우리 로봇 STEP과 같은 조건)
#
# 형상 (단위 mm, 조립 좌표계 = 어깨 관절 중심이 원점, 팔은 -Z 로 늘어짐)
#   upper_block : 상완 상자 40 × 40 × 200, z = -200 ~ 0
#   upper_bolt  : 상완에 붙은 볼트(원통 r 5, h 40), 상자 옆면 +X 쪽, z = -100 에서 X 방향
#   fore_cyl    : 전완 원통 r 20, h 180, 축 = Z, z = -380 ~ -200
# 링크 분할(정답): upper_arm = upper_block + upper_bolt, forearm = fore_cyl
import os
from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCylinder
from OCP.gp import gp_Pnt, gp_Ax2, gp_Dir
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDataStd import TDataStd_Name
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.Interface import Interface_Static

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))          # sim/
OUT = os.path.join(ROOT, "test_cad", "test_arm.step")

parts = {
    "upper_block": BRepPrimAPI_MakeBox(gp_Pnt(-20, -20, -200), 40, 40, 200).Shape(),
    "upper_bolt": BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(20, 0, -100), gp_Dir(1, 0, 0)), 5, 40).Shape(),
    "fore_cyl": BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, 0, -380), gp_Dir(0, 0, 1)), 20, 180).Shape(),
}

doc = TDocStd_Document(TCollection_ExtendedString("XmlOcaf"))
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
for name, shp in parts.items():
    lab = st.AddShape(shp, False)
    TDataStd_Name.Set_s(lab, TCollection_ExtendedString(name))

os.makedirs(os.path.dirname(OUT), exist_ok=True)
Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
Interface_Static.SetCVal_s("write.step.unit", "MM")
w = STEPCAFControl_Writer()
w.SetNameMode(True)
w.Transfer(doc, STEPControl_AsIs)
w.Write(OUT)
print("saved", OUT, os.path.getsize(OUT), "bytes")
