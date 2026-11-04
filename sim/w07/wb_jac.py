# week07 | 전신(베이스 3 + 팔 4) 자코비안 · 가중치 · 가중 DLS · 영공간 — 함태훈 임시 구현 (B1)
#  노션 7주차 팀 연계: 「박종진 wb_ik 의 자코비안 · 가중치 함수를 그대로 사용해 제어기 작성」.
#  박종진 wb_ik.py(인터페이스 v2) 공유 전이라 같은 형태로 먼저 만들어 제어기(w07_wb_rrc.py)에 씀 → 받으면 아래 4 함수만 교체.
#  제안 인터페이스 v2 (박종진 합의 대기):
#    wb_jacobian(m, d, side, include_base=True) -> (Jp[3×n], dofs[n])   열 순서 = base TX · TY · RZ, pitch · roll · yaw · elbow
#    wb_weights(include_base=True, w_xy=10, w_yaw=10) -> W[n]          대각 가중치(클수록 덜 움직임). 팔 = 1
#    wdls(J, W, lam) -> J#[n×3]                                         J# = W⁻¹Jᵀ (J W⁻¹ Jᵀ + λ² I)⁻¹
#    limit_avoid(m, d, side, include_base=True, k=1.0) -> q̇0[n]       영공간 2차 목표: 팔 관절 한계 회피(중앙으로), 베이스 0
#  q̇ = J# ẋ + (I − J# J) q̇0
import numpy as np, mujoco

ARM = ["shoulder_pitch", "shoulder_roll", "shoulder_yaw", "elbow"]
BASE = ["base_planar_joint_TX", "base_planar_joint_TY", "base_planar_joint_RZ"]


def dofs(m, side, include_base=True):
    names = (BASE if include_base else []) + [f"{side}_{a}_joint" for a in ARM]
    return np.array([m.jnt_dofadr[m.joint(n).id] for n in names])


def wb_jacobian(m, d, side, include_base=True):
    """끝점 site 위치 자코비안 (world 좌표). base TX · TY 열 = world x · y 단위벡터(slide 축 world 고정), RZ 열 = ẑ × (p − p_RZ)"""
    jp = np.zeros((3, m.nv)); mujoco.mj_jacSite(m, d, jp, None, m.site(f"{side}_ee").id)
    dd = dofs(m, side, include_base); return jp[:, dd], dd


def wb_weights(include_base=True, w_xy=10.0, w_yaw=10.0):
    return np.array(([w_xy, w_xy, w_yaw] if include_base else []) + [1.0] * 4)


def wdls(J, W, lam=0.03):
    Wi = np.diag(1.0 / W)
    return Wi @ J.T @ np.linalg.inv(J @ Wi @ J.T + lam ** 2 * np.eye(J.shape[0]))


def limit_avoid(m, d, side, include_base=True, k=1.0):
    """H = ½ Σ ((q − q_mid)/(q_max − q_min))² 의 음의 기울기 → 팔을 범위 중앙 쪽으로"""
    jid = [m.joint(f"{side}_{a}_joint").id for a in ARM]
    q = d.qpos[[m.jnt_qposadr[j] for j in jid]]; lo, hi = m.jnt_range[jid, 0], m.jnt_range[jid, 1]
    g = -k * (q - (lo + hi) / 2) / (hi - lo) ** 2
    return np.concatenate([np.zeros(3) if include_base else [], g])
