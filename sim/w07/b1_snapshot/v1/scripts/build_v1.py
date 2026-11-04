# B1 v1 빌드 — URDF 생성 → MJCF 변환 → 덧붙이기를 한 번에 (week05, 박종진)
#   1) urdf/b1_v1.urdf        원본(single source of truth). 베이스 = planar 1개 (팀장 10/16, C3)
#      urdf/b1_v1_isaac.urdf  Isaac Sim용 사본: planar → prismatic x · prismatic y · revolute z + 가상 링크 2개 (체크리스트 C3-1)
#      docs/mirror_rules.md   좌우 미러 규칙표 (코드에서 자동 생성 → 표와 모델이 항상 일치)
#   2) mjcf/b1_urdf.xml       URDF → MuJoCo 변환본(mj_saveLastXML) — 수정 금지
#   3) mjcf/b1.xml            변환본 + site(ee · IMU) + default class(관절 그룹별) + 액추에이터 + 센서 ← actuator_params.json
#      mjcf/scene.xml         include b1.xml + 바닥 · 조명 · option
# 값: ../../assumptions_b1.json (B1 공용 가정값) + ../actuator_params.json (MJCF 덧붙이기, 함태훈 담당)
#     링크 질량 · 관성 = inputs/link_inertials_v1.json(함태훈 13b01ad, 관절점 원점), 메시 = ../../v0/meshes(함태훈 f9c9eff 원점 보정본 — 함태훈 robot_v1 메시와 collision 0.0 mm 동일, visual은 2만 면 단순화본)
# 사용: python build_v1.py            (저장소 어디서 실행해도 됨)
import os,sys,json,copy,shutil,numpy as np,trimesh,mujoco,xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation as Ro
HERE=os.path.dirname(os.path.abspath(__file__)); V1=os.path.dirname(HERE); MODELS=os.path.dirname(V1); REPO=os.path.dirname(os.path.dirname(MODELS))
A=json.load(open(os.path.join(MODELS,"assumptions_b1.json"),encoding="utf-8"))
P=json.load(open(os.path.join(V1,"actuator_params.json"),encoding="utf-8"))
# (week06) A2 구동기 사양(10/19) 반영: actuator_params.json "a2.case" → inputs/a2_actuators.json 의 case 값으로 class별 armature · forcerange, URDF effort 를 덮어씀
A2C=json.load(open(os.path.join(V1,"inputs","a2_actuators.json"),encoding="utf-8"))
CASE=os.environ.get("B1_A2_CASE",P.get("a2",{}).get("case","none"))
if CASE!="none":
    for cname,cv in A2C["cases"][CASE]["classes"].items():
        P["classes"][cname]["armature"]=cv["armature"]; P["classes"][cname]["forcerange"]=cv["peak"]
EFF={a:(A2C["cases"][CASE]["classes"][c]["peak"] if CASE!="none" else A["actuator"]["effort_peak"]) for c,cp in P["classes"].items() for a in cp["joints"]}
# 링크 질량 · 관성: 함태훈 sim/robot_v1/link_inertials_v1.json (13b01ad, 원점 = B1 골격 관절점) — 평행이동 변환(merge_taehoon) 불필요.
#   week04 B1 프레임 변환본과 질량 차 ≤ 2.4e-6 kg · COM ≤ 0.01 mm · 관성 ≤ 4e-8 (10/18 대조). 오른팔은 왼팔 미러
INER=json.load(open(os.path.join(V1,"inputs","link_inertials_v1.json"),encoding="utf-8"))
SRC_MESH=os.path.join(MODELS,"v0","meshes")
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
JG={k:v for k,v in P.get("joint_gains",{}).items() if not k.startswith("_")}   # (week07 게인 통일, 함태훈) 관절별 고정 kp · kv
for d in ("urdf","meshes","mjcf","docs"): os.makedirs(os.path.join(V1,d),exist_ok=True)

# ---- 왼팔 기준 기하 (link-split-v0 §3, 로봇 좌표 m) — 오른팔은 미러 규칙으로 생성
J_XYZ={"shoulder_pitch":(0,0.16975,0),"shoulder_roll":(0,0.1079,-0.0200),"shoulder_yaw":(0,0,-0.1823),"elbow":(0,0,-0.0938)}
J_AX ={"shoulder_pitch":(0,1,0),"shoulder_roll":(1,0,0),"shoulder_yaw":(0,0,1),"elbow":(1,0,0)}
TOOL=(-0.009,0,-0.1989)                      # 그리퍼 플랜지면 (left_elbow_link 프레임)
YAW0=A["zero_pose"]["left_shoulder_yaw_origin_yaw_rad"]
# ---- 미러 규칙 (거울면 y = 0): 점 · 벡터 y 반전 / 회전축(유사벡터)은 x · z 반전, y 유지 / rpy는 roll · yaw 반전
def m_pt(v): return (v[0],-v[1],v[2])
def m_axis(a): return (-a[0],a[1],-a[2])
def m_rpy(r): return (-r[0],r[1],-r[2])
def m_I(I): return [I[0],I[1],I[2],-I[3],I[4],-I[5]]   # ixx iyy izz ixy ixz iyz
def f(x): return " ".join(f"{v:.6g}" for v in x)
def box_I(m,sx,sy,sz): return [m*(sy*sy+sz*sz)/12,m*(sx*sx+sz*sz)/12,m*(sx*sx+sy*sy)/12,0,0,0]
def combine(parts):
    M=sum(p[0] for p in parts); C=sum(p[0]*np.array(p[1]) for p in parts)/M; It=np.zeros((3,3))
    for m,c,I in parts:
        Im=np.array([[I[0],I[3],I[4]],[I[3],I[1],I[5]],[I[4],I[5],I[2]]]); r=np.array(c)-C
        It+=Im+m*(r@r*np.eye(3)-np.outer(r,r))
    return M,C,[It[0,0],It[1,1],It[2,2],It[0,1],It[0,2],It[1,2]]
def inertial(m,com,I):
    return (f'    <inertial>\n      <origin xyz="{f(com)}" rpy="0 0 0"/>\n      <mass value="{m:.6g}"/>\n'
            f'      <inertia ixx="{I[0]:.6g}" iyy="{I[1]:.6g}" izz="{I[2]:.6g}" ixy="{I[3]:.6g}" ixz="{I[4]:.6g}" iyz="{I[5]:.6g}"/>\n    </inertial>\n')
DUMMY='    <inertial>\n      <mass value="0.001"/>\n      <inertia ixx="1e-7" iyy="1e-7" izz="1e-7" ixy="0" ixz="0" iyz="0"/>\n    </inertial>\n'
def boxgeo(name,xyz,size):
    g=f'<origin xyz="{f(xyz)}"/>\n      <geometry><box size="{f(size)}"/></geometry>'
    return f'    <visual name="{name}_visual">\n      {g}\n    </visual>\n    <collision name="{name}_collision">\n      {g}\n    </collision>\n'
def joint(name,typ,parent,child,xyz=(0,0,0),rpy=(0,0,0),axis=None,limit=None,dyn=None):
    s=f'  <joint name="{name}" type="{typ}">\n    <parent link="{parent}"/>\n    <child link="{child}"/>\n    <origin xyz="{f(xyz)}" rpy="{f(rpy)}"/>\n'
    if axis: s+=f'    <axis xyz="{f(axis)}"/>\n'
    if limit: s+='    <limit '+" ".join(f'{k}="{v}"' for k,v in limit.items())+'/>\n'
    if dyn: s+=f'    <dynamics damping="{dyn[0]}" friction="{dyn[1]}"/>\n'
    return s+'  </joint>\n'

# ---- 메시: 왼팔 · 몸통은 v0 통합본 그대로, 오른팔은 미러(y 반전 + 면 방향 뒤집기)
def meshes():
    info={}
    for k in ["torso_link"]+[f"left_{a}_link" for a in ARM]:
        for kind in ("visual","collision"):
            os.makedirs(os.path.join(V1,"meshes",kind),exist_ok=True)
            shutil.copy(os.path.join(SRC_MESH,f"{k}_{kind}.stl"),os.path.join(V1,"meshes",kind,f"{k}_{kind}.stl"))
            if k.startswith("left_"):
                t=trimesh.load(os.path.join(SRC_MESH,f"{k}_{kind}.stl")); t.apply_transform(np.diag([1,-1,1,1])); t.faces=t.faces[:,::-1]
                t.export(os.path.join(V1,"meshes",kind,f"{k.replace('left_','right_')}_{kind}.stl"))
            for kk in ([k] if not k.startswith("left_") else [k,k.replace("left_","right_")]):
                info[f"{kk}_{kind}"]=len(trimesh.load(os.path.join(V1,"meshes",kind,f"{kk}_{kind}.stl")).faces)
    return info

def build_urdf(variant):
    b=A["base"]; t=A["torso"]; act=A["actuator"]; lim=A["limits"]; g=A["gripper"]; c=A["control"]
    L=['<?xml version="1.0"?>\n<!-- 자동 생성: b1/models/v1/scripts/build_v1.py (week05, 박종진) — 손으로 고치지 말 것. 값: assumptions_b1.json -->\n',
       f'<robot name="b1_v1{"_isaac" if variant=="isaac" else ""}">\n',
       '  <mujoco>\n    <compiler strippath="false" fusestatic="false" discardvisual="false" balanceinertia="false"/>\n  </mujoco>\n\n  <link name="odom"/>\n']
    w=L.append
    # ---- 베이스 3-DOF (C3)
    if variant=="planar":
        w(joint("base_planar_joint","planar","odom","base_link",axis=(0,0,1)))
    else:   # Isaac Sim: planar 미지원 → prismatic x · prismatic y · revolute z (가상 링크 2개, 질량 1 g)
        bl={"lower":-100,"upper":100,"effort":2000,"velocity":c["base_vxy"]}
        w(joint("base_x_joint","prismatic","odom","base_x_link",axis=(1,0,0),limit=bl)); w('  <link name="base_x_link">\n'+DUMMY+'  </link>\n')
        w(joint("base_y_joint","prismatic","base_x_link","base_y_link",axis=(0,1,0),limit=bl)); w('  <link name="base_y_link">\n'+DUMMY+'  </link>\n')
        w(joint("base_yaw_joint","continuous","base_y_link","base_link",axis=(0,0,1),limit={"effort":500,"velocity":c["base_wz"]}))
    # base_link = 하체 box (원점 = 바닥면 중심, ICD 3.1)
    bx,by=b["plate_xy"]; zt=b["plate_top_z"]; zc=b["clearance_z"]; pt=b["plate_t"]; hb=zt-zc-pt
    w('  <link name="base_link">\n'+inertial(b["mass_lower"],(0,0,zc+hb/2),box_I(b["mass_lower"],bx,by,hb))+boxgeo("base_lower",(0,0,zc+hb/2),(bx,by,hb))+'  </link>\n')
    # 상판 = 별도 고정 링크 (팀장 10/16 C1)
    w(joint("base_plate_joint","fixed","base_link","base_plate_link",xyz=(0,0,zt-pt/2)))
    w('  <link name="base_plate_link">\n'+inertial(b["mass_plate"],(0,0,0),box_I(b["mass_plate"],bx,by,pt))+boxgeo("base_plate",(0,0,0),(bx,by,pt))+'  </link>\n\n')
    # 몸통: CAD(J1 RI85 고정측 ×2, 함태훈) + box 8 kg (C2)
    w(joint("torso_fixed_joint","fixed","base_link","torso_link",xyz=(0,0,t["shoulder_z"])))
    ti=INER["torso_link"]; tz0=zt-t["shoulder_z"]; tz1=t["box_top_above_shoulder"]; th=tz1-tz0
    M,C,I=combine([(ti["mass"],ti["com"],ti["inertia"]),(t["box_mass"],(0,0,(tz0+tz1)/2),box_I(t["box_mass"],t["box_x"],t["box_y"],th))])
    w('  <link name="torso_link">\n'+inertial(M,C,I))
    for kind in ("visual","collision"):
        w(f'    <{kind} name="torso_actuators_{kind}">\n      <geometry><mesh filename="../meshes/{kind}/torso_link_{kind}.stl"/></geometry>\n    </{kind}>\n')
    w(boxgeo("torso_box",(0,0,(tz0+tz1)/2),(t["box_x"],t["box_y"],th))+'  </link>\n')
    R=np.array(A["cad_to_base"]["R"],float); cc=np.array(A["cad_to_base"]["c_mm"],float)
    w(joint("cad_frame_joint","fixed","torso_link","cad_frame",xyz=R@(-cc)/1000.0,rpy=Ro.from_matrix(R).as_euler("xyz")))
    w('  <link name="cad_frame">\n'+DUMMY+'  </link>\n\n')
    # 양팔: 왼팔 정의 → 오른팔 = 미러 규칙
    rows=[]
    for side in ("left","right"):
        parent="torso_link"
        for a in ARM:
            xyz=J_XYZ[a]; ax=J_AX[a]; rpy=(0,0,YAW0) if a=="shoulder_yaw" else (0,0,0)
            li=INER[f"left_{a}_link"]; com=li["com"]; I=li["inertia"]
            if side=="right": xyz,ax,rpy,com,I=m_pt(xyz),m_axis(ax),m_rpy(rpy),m_pt(com),m_I(I)
            lo,hi=lim[a]
            w(joint(f"{side}_{a}_joint","revolute",parent,f"{side}_{a}_link",xyz,rpy,ax,
                    {"lower":lo,"upper":hi,"effort":EFF[a],"velocity":act["velocity"]},(act["damping"],act["friction"])))
            w(f'  <link name="{side}_{a}_link">\n'+inertial(li["mass"],com,I))
            for kind in ("visual","collision"):
                w(f'    <{kind} name="{side}_{a}_{kind}">\n      <geometry><mesh filename="../meshes/{kind}/{side}_{a}_link_{kind}.stl"/></geometry>\n    </{kind}>\n')
            w('  </link>\n'); parent=f"{side}_{a}_link"
            if side=="left": rows.append((a,J_XYZ[a],m_pt(J_XYZ[a]),J_AX[a],m_axis(J_AX[a]),(0,0,YAW0) if a=="shoulder_yaw" else (0,0,0),lim[a]))
        tx=TOOL if side=="left" else m_pt(TOOL)
        w(joint(f"{side}_tool0_joint","fixed",parent,f"{side}_tool0",tx)); w(f'  <link name="{side}_tool0">\n'+DUMMY+'  </link>\n')
        # 그리퍼 임시 box (S14, BOM 대기): 플랜지면에서 −z(전완 연장 방향)
        gz=g["box_xyz"][2]
        w(joint(f"{side}_gripper_joint","fixed",f"{side}_tool0",f"{side}_gripper_link"))
        w(f'  <link name="{side}_gripper_link">\n'+inertial(g["mass"],(0,0,-gz/2),box_I(g["mass"],*g["box_xyz"]))+boxgeo(f"{side}_gripper",(0,0,-gz/2),g["box_xyz"])+'  </link>\n\n')
    w('</robot>\n')
    return "".join(L),rows

def mirror_table(rows):
    T=["# 좌우 미러 규칙표 (v1, 자동 생성 — build_v1.py)","",
       "- 기준 = **왼팔**(ICD 이름 `left_*`)을 정의하고 오른팔(`right_*`)을 스크립트로 생성. 거울면 = base_link의 xz 평면(y = 0)",
       "- 규칙: 장착 위치 · COM · 메시 = **y 반전**, 관절 축 = **유사벡터 변환 (x · z 반전, y 유지)**, origin rpy = **roll · yaw 반전**, 관성 = ixy · iyz 반전, 관절 범위 = **같음**(안 A 거울, 팀장 승인 10/16)",
       "- 같은 관절각 명령 → 좌우 대칭 동작. 실제 구동기 +방향과의 차이는 B2 드라이버 계층 한 곳에서만 반전 ([joint-origin-check-v0.md](../../joint-origin-check-v0.md) §2)","",
       "| joint | left origin xyz [m] | right origin xyz | left axis | right axis | 축 부호 | origin rpy (left → right) | 범위 [rad] (양팔 동일) | 이유 |","|---|---|---|---|---|---|---|---|---|"]
    why={"shoulder_pitch":"축이 거울면 법선(y)과 평행 → 유사벡터 y 성분은 거울에서 그대로 → 안 바꿈",
         "shoulder_roll":"축이 거울면 안(x) → 거울상에서 회전 방향이 뒤집힘 → 부호 반전",
         "shoulder_yaw":"축이 거울면 안(z) → 부호 반전. 영점 rpy yaw도 반전(∓π/2)",
         "elbow":"축이 거울면 안(x) → 부호 반전 (양팔 모두 + = 전방 굽힘)"}
    for a,lx,rx,la,ra,rpy,lim in rows:
        flip="유지" if la==ra else "**반전**"
        T.append(f"| `{{side}}_{a}_joint` | ({f(lx)}) | ({f(rx)}) | ({f(la)}) | ({f(ra)}) | {flip} | ({f(rpy)}) → ({f(m_rpy(rpy))}) | {lim[0]} ~ {lim[1]} | {why[a]} |")
    T+=["",f"- tool0: left ({f(TOOL)}) → right ({f(m_pt(TOOL))}) [elbow_link 프레임], 그리퍼 box · ee site는 tool0 기준 (0, 0, −z)라 미러 불변",
        "- 판별 요령(구술 Q1): 회전축은 **유사벡터** — 거울면에 수직인 성분은 유지, 평행한 성분은 반전. 우리 팔은 pitch만 y축(거울면 법선)이라 유지, roll · yaw · elbow는 반전",
        "- 검증: 검증 ⑤ (같은 관절각 → left_ee · right_ee y 부호만 다르고 x · z 차 ≤ 1 mm) — [../results/validate_model.md](../results/validate_model.md)"]
    open(os.path.join(V1,"docs","mirror_rules.md"),"w",encoding="utf-8").write("\n".join(T)+"\n")

# ---- MJCF 덧붙이기 (actuator_params.json) — 변환본은 그대로 두고 b1.xml을 새로 만든다
def build_mjcf():
    up=os.path.join(V1,"urdf","b1_v1.urdf"); m=mujoco.MjModel.from_xml_path(up)
    raw=os.path.join(V1,"mjcf","b1_urdf.xml"); mujoco.mj_saveLastXML(raw,m)
    tree=ET.parse(raw); root=tree.getroot(); root.set("model","b1_v1")
    comp=root.find("compiler"); comp.set("meshdir","../meshes/")
    for me in root.iter("mesh"):   # URDF 상대 경로(../meshes/visual/x.stl) → MJCF는 meshdir 기준(visual/x.stl)
        f_=me.get("file","").replace("\\","/"); me.set("file","/".join(f_.split("/")[-2:]))
    # default class: 관절 그룹별 (어깨 · 팔꿈치) + 베이스
    dflt=ET.Element("default")
    for cname,cp in P["classes"].items():
        d=ET.SubElement(dflt,"default",{"class":cname})
        ET.SubElement(d,"joint",{"armature":f'{cp["armature"]}',"damping":f'{cp["damping"]}',"frictionloss":f'{cp["frictionloss"]}'})
        pos_attr={"kp":f'{cp["kp"]}',"dampratio":f'{cp["dampratio"]}',"forcerange":f'{-cp["forcerange"]} {cp["forcerange"]}',"inheritrange":"1"}
        if JG: pos_attr.pop("dampratio")   # (week07 게인 통일) 관절별 고정 kp · kv 는 액추에이터에 직접
        ET.SubElement(d,"position",pos_attr)
    bp=P["base"]; d=ET.SubElement(dflt,"default",{"class":"base"}); ET.SubElement(d,"velocity",{"kv":f'{bp["vx_kv"]}',"forcerange":f'{-bp["lin_forcerange"]} {bp["lin_forcerange"]}'})
    root.insert(list(root).index(root.find("worldbody")),dflt)
    cls_of={a:c for c,cp in P["classes"].items() for a in cp["joints"]}
    for j in root.iter("joint"):
        n=j.get("name","")
        for a,c in cls_of.items():
            if n.endswith(f"_{a}_joint"):
                j.set("class",c)
                for k in ("damping","armature","frictionloss"): j.attrib.pop(k,None)   # 값은 class(=params)에서
    # (함태훈 제안 10/17) 중력보상 B: 팔 body gravcomp + joint actuatorgravcomp — 보상 토크가 qfrc_actuator 로 들어가 ctrlrange 클리핑과 무관, 합계는 actuatorfrcrange 로 제한
    if P.get("gravcomp",{}).get("enable",False):
        for b in root.iter("body"):
            if b.get("name","").startswith(("left_","right_")): b.set("gravcomp","1")
        for d_ in dflt.iter("default"):
            j_=d_.find("joint")
            if d_.get("class") in P["classes"] and j_ is not None:
                fr=P["classes"][d_.get("class")]["forcerange"]; j_.set("actuatorgravcomp","true"); j_.set("actuatorfrcrange",f"{-fr} {fr}")
    # site: 끝점 ee (그리퍼 끝단) · 베이스 IMU (상판 중심)
    for b in root.iter("body"):
        if b.get("name","").endswith("_gripper_link"):
            s=b.get("name").replace("_gripper_link","_ee")
            ET.SubElement(b,"site",{"name":s,"pos":f'0 0 {-A["gripper"]["ee_offset"]}',"size":"0.012","rgba":"1 0.3 0.1 1","group":"3"})
        if b.get("name")=="base_plate_link":
            ET.SubElement(b,"site",{"name":"base_imu","pos":"0 0 0","size":"0.015","rgba":"0.1 0.6 1 1","group":"3"})
    act=ET.SubElement(root,"actuator")
    for nm,jn,kv,cr,fr in (("base_vx","base_planar_joint_TX",bp["vx_kv"],bp["v_max"],bp["lin_forcerange"]),("base_vy","base_planar_joint_TY",bp["vy_kv"],bp["v_max"],bp["lin_forcerange"]),("base_wz","base_planar_joint_RZ",bp["wz_kv"],bp["w_max"],bp["yaw_forcerange"])):
        ET.SubElement(act,"velocity",{"class":"base","name":nm,"joint":jn,"kv":f"{kv}","ctrlrange":f"{-cr} {cr}","forcerange":f"{-fr} {fr}"})
    for side in ("left","right"):
        for a in ARM:
            at={"class":cls_of[a],"name":f"{side}_{a}","joint":f"{side}_{a}_joint"}
            if a in JG: at.update(kp=f'{JG[a]["kp"]}',kv=f'{JG[a]["kv"]}')   # (week07) actuator_params.json joint_gains
            ET.SubElement(act,"position",at)
    sen=ET.SubElement(root,"sensor")
    for side in ("left","right"):
        for a in ARM:
            jn=f"{side}_{a}_joint"
            ET.SubElement(sen,"jointpos",{"name":f"{side}_{a}_pos","joint":jn}); ET.SubElement(sen,"jointvel",{"name":f"{side}_{a}_vel","joint":jn})
            ET.SubElement(sen,"actuatorfrc",{"name":f"{side}_{a}_frc","actuator":f"{side}_{a}"})
            if P.get("gravcomp",{}).get("enable",False): ET.SubElement(sen,"jointactuatorfrc",{"name":f"{side}_{a}_jfrc","joint":jn})   # 보상분 포함 관절 토크
    ET.SubElement(sen,"gyro",{"name":"base_gyro","site":"base_imu"}); ET.SubElement(sen,"accelerometer",{"name":"base_acc","site":"base_imu"})
    for s in P["sensors"]["ee_sites"]:
        ET.SubElement(sen,"framepos",{"name":f"{s}_pos","objtype":"site","objname":s}); ET.SubElement(sen,"framequat",{"name":f"{s}_quat","objtype":"site","objname":s})
    ET.indent(tree,"  ")
    hdr="<!-- 자동 생성: build_v1.py — b1_urdf.xml(URDF 변환본) + actuator_params.json 덧붙이기. 손으로 고치지 말고 params/json을 고친 뒤 다시 빌드 -->\n"
    open(os.path.join(V1,"mjcf","b1.xml"),"w",encoding="utf-8").write(hdr+ET.tostring(root,encoding="unicode")+"\n")
    open(os.path.join(V1,"mjcf","scene.xml"),"w",encoding="utf-8").write('''<!-- 환경만 (바닥 · 조명 · option). 실행: python -m mujoco.viewer --mjcf=scene.xml (한글 경로면 scripts/v1_mjio.py) -->
<mujoco model="b1_v1_scene">
  <include file="b1.xml"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>
  <visual>
    <headlight diffuse="0.7 0.7 0.7" ambient="0.35 0.35 0.35" specular="0.1 0.1 0.1"/>
    <global azimuth="150" elevation="-15" offwidth="1280" offheight="960"/>
  </visual>
  <asset>
    <texture name="sky" type="skybox" builtin="gradient" rgb1="0.92 0.94 0.97" rgb2="0.75 0.80 0.88" width="256" height="256"/>
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.72 0.74 0.76" width="300" height="300"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="1.5 -1.5 3" dir="-0.5 0.5 -1" directional="true"/>
    <geom name="floor" type="plane" size="5 5 0.05" material="grid" group="2"/>
  </worldbody>
</mujoco>
''')

if __name__=="__main__":
    info=meshes()
    u,rows=build_urdf("planar"); open(os.path.join(V1,"urdf","b1_v1.urdf"),"w",encoding="utf-8").write(u)
    ui,_=build_urdf("isaac");    open(os.path.join(V1,"urdf","b1_v1_isaac.urdf"),"w",encoding="utf-8").write(ui)
    mirror_table(rows); build_mjcf()
    json.dump(info,open(os.path.join(V1,"meshes","mesh_info.json"),"w"),indent=1)
    m=mujoco.MjModel.from_xml_path(os.path.join(V1,"mjcf","scene.xml"))
    print(f"A2 case: {CASE}, gravcomp {P.get('gravcomp',{}).get('enable',False)}, armature "+str({c:P['classes'][c]['armature'] for c in P['classes']}))
    print(f"built: nq {m.nq} nv {m.nv} nu {m.nu} nbody {m.nbody} nsensor {m.nsensor} mass {sum(m.body_mass):.3f} kg")
