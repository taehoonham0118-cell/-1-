# week04 | convex hull 충돌의 한계를 MuJoCo 낙하 시험으로 보기 (함태훈, B1)
# MuJoCo 는 mesh geom 의 충돌을 그 메시의 convex hull 로 계산한다. 오목한 링크(C자 브래킷 등)에서는
# hull 이 빈 공간을 메워, 물체가 실제 표면보다 위에서 「가짜 접촉」으로 멈춘다.
# 공은 수직 slide 관절만 줘서 옆으로 굴러 떨어지지 않게 했다.
# 실행: python sim/scripts/hull_contact_demo.py --g1 mujoco_menagerie/unitree_g1 --link left_shoulder_roll_link
import argparse, os
import numpy as np
import trimesh, mujoco

ap = argparse.ArgumentParser()
ap.add_argument("--g1", default="mujoco_menagerie/unitree_g1")
ap.add_argument("--link", default="left_shoulder_roll_link")
ap.add_argument("--r", type=float, default=0.004, help="떨어뜨릴 공 반지름 [m]")
a = ap.parse_args()

stl = os.path.join(a.g1, "assets", a.link + ".STL")
mesh = trimesh.load(stl); hull = mesh.convex_hull
# hull 윗면(법선 z > 0.7)에서, 바로 아래 실제 메시 표면까지 가장 깊은 점 찾기
pts, fid = trimesh.sample.sample_surface(hull, 20000, seed=1)
up = hull.face_normals[fid][:, 2] > 0.7
pts = pts[up]
loc, idx_ray, _ = mesh.ray.intersects_location(pts + [0, 0, 1e-4], np.tile([0, 0, -1.0], (len(pts), 1)))
depth = np.full(len(pts), np.nan)
for p_i, z in zip(idx_ray, loc[:, 2]):
    d = pts[p_i, 2] - z
    if np.isnan(depth[p_i]) or d < depth[p_i]:
        depth[p_i] = d                            # 가장 위쪽 실제 표면까지 거리
k = int(np.nanargmax(depth)); p = pts[k]; true_z = p[2] - depth[k]
print(f"{a.link}: hull 윗면 점 {np.round(p*1000,1)} mm, 그 아래 실제 표면까지 {depth[k]*1000:.1f} mm")

xml = f"""<mujoco>
<option timestep="0.001"/>
<asset><mesh name="m" file="{a.link}.STL"/></asset>
<worldbody><geom type="mesh" mesh="m"/>
<body pos="{p[0]} {p[1]} {p[2] + 0.03}"><joint type="slide" axis="0 0 1"/><geom type="sphere" size="{a.r}" mass="0.01"/></body></worldbody></mujoco>"""
m = mujoco.MjModel.from_xml_string(xml, {a.link + ".STL": open(stl, "rb").read()})   # 파일은 Python 이 읽어 넘김(한글 경로 대비)
d = mujoco.MjData(m)
for _ in range(2000):
    mujoco.mj_step(m, d)
z_rest = p[2] + 0.03 + d.qpos[0] - a.r     # slide 관절: 수직으로만 움직이게 해 미끄러짐을 막음
print(f"MuJoCo 2 s 낙하 후 공 바닥 높이 {z_rest*1000:.1f} mm  |  hull 표면 {p[2]*1000:.1f} mm  |  실제 메시 표면 {true_z*1000:.1f} mm")
print(f"→ 실제 표면보다 {(z_rest-true_z)*1000:.1f} mm 위에서 멈춤 (convex hull 이 메운 틈). 접촉 수 {d.ncon}")
