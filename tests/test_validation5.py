# 검증 5종 회귀 테스트 (week10, 함태훈) — 5주차 validate_model.py 와 같은 정의, 통과 기준 = config/params.yaml regression
#   ① 질량 ±5 %  ② 앞으로 수평 5 s 정지 유지 ≤ 0.5°(gravcomp 빌드)  ③ 3 kg @ ee 15° 격자 최악 중력 토크(값 고정) · × 1.2 ≤ 정격
#   ④ 관절 하나씩 범위 sweep 자기 충돌 0  ⑤ 좌우 대칭 x · z ≤ 1 mm
import itertools,numpy as np,mujoco,pytest
from v1_mjio import load_model
@pytest.fixture(scope="module")
def mb(scene): return load_model(scene)                       # 빌드 그대로(gravcomp 켬)
@pytest.fixture(scope="module")
def ms(scene): return load_model(scene,strip_gravcomp=True)   # ①③④⑤ = 보상 없는 모델(5주차 기준과 같은 의미)
ARM=["shoulder_pitch","shoulder_roll","shoulder_yaw","elbow"]
def adr(m,side): return np.array([m.jnt_qposadr[m.joint(f"{side}_{a}_joint").id] for a in ARM])

def test_1_mass(ms,par):
    R=par["regression"]["mass_total_kg"]; tot=float(ms.body_mass.sum())
    assert abs(tot-R["ref"])/R["ref"]*100<=R["tol_pct"]
    assert tot==pytest.approx(R["value"],abs=0.002)            # 빌드가 질량을 바꾸지 않았는지(회귀)

def test_2_hold_forward(mb,par):
    m=mb; d=mujoco.MjData(m); qa=adr(m,"left"); q=np.array([-np.pi/2,0,0,0])
    d.qpos[qa]=q; mujoco.mj_forward(m,d)
    for i,a in enumerate(ARM): d.ctrl[m.actuator(f"left_{a}").id]=q[i]
    err=0.0
    for _ in range(int(5.0/m.opt.timestep)):
        mujoco.mj_step(m,d); err=max(err,np.degrees(np.abs(d.qpos[qa]-q)).max())
    assert err<=par["regression"]["hold_err_deg_max"]

def test_3_gravity_torque(ms,par):
    m=ms; d=mujoco.MjData(m); qa=adr(m,"left"); da=np.array([m.jnt_dofadr[m.joint(f"left_{a}_joint").id] for a in ARM])
    ee=m.site("left_ee").id; lo=m.jnt_range[[m.joint(f"left_{a}_joint").id for a in ARM],0]; hi=m.jnt_range[[m.joint(f"left_{a}_joint").id for a in ARM],1]
    grids=[np.radians(np.arange(np.ceil(np.degrees(lo[i])/15)*15,np.degrees(hi[i])+1e-6,15)) for i in range(4)]; worst=np.zeros(4)   # 0 · ±90° 포함 15° 격자(validate_model 과 같음)
    for q in itertools.product(*grids):
        d.qpos[:]=0; d.qpos[qa]=q; mujoco.mj_forward(m,d)
        jp=np.zeros((3,m.nv)); mujoco.mj_jacSite(m,d,jp,None,ee)
        worst=np.maximum(worst,np.abs(d.qfrc_bias[da]+3.0*9.81*jp[2,da]))
    R=par["regression"]["grav3kg_ee_pitch_Nm"]
    assert worst[0]==pytest.approx(R["value"],abs=R["tol"])
    import params as PRM
    for i,a in enumerate(ARM): assert PRM.rated(par,a)>=worst[i]*(1+par["torque_rule"]["min_margin"])

def test_4_range_sweep_no_self_collision(ms):
    m=ms; d=mujoco.MjData(m); floor=[i for i in range(m.ngeom) if m.geom_type[i]==mujoco.mjtGeom.mjGEOM_PLANE]; bad=0
    for side in ("left","right"):
        for a in ARM:
            j=m.joint(f"{side}_{a}_joint").id
            for x in np.arange(m.jnt_range[j,0],m.jnt_range[j,1]+1e-9,np.radians(1.0)):
                d.qpos[:]=m.qpos0; d.qpos[m.jnt_qposadr[j]]=x; mujoco.mj_forward(m,d)
                bad+=sum(1 for c in d.contact[:d.ncon] if c.geom1 not in floor and c.geom2 not in floor)
    assert bad==0

def test_5_mirror(ms,par):
    m=ms; d=mujoco.MjData(m); L=adr(m,"left"); R=adr(m,"right"); eL=m.site("left_ee").id; eR=m.site("right_ee").id
    lo=m.jnt_range[[m.joint(f"left_{a}_joint").id for a in ARM],0]; hi=m.jnt_range[[m.joint(f"left_{a}_joint").id for a in ARM],1]
    rng=np.random.default_rng(1); mx=0.0
    for _ in range(200):
        q=lo+(hi-lo)*rng.random(4); d.qpos[:]=m.qpos0; d.qpos[L]=q; d.qpos[R]=q; mujoco.mj_forward(m,d)
        a,b=d.site_xpos[eL],d.site_xpos[eR]; mx=max(mx,abs(a[0]-b[0])*1e3,abs(a[2]-b[2])*1e3,abs(a[1]+b[1])*1e3)
    assert mx<=par["regression"]["mirror_mm_max"]
