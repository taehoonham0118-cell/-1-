# week02 | Git 없이 mujoco_menagerie 의 unitree_g1 폴더만 내려받기 (함태훈, B1)
# 실행: python sim/scripts/fetch_g1.py   (저장소 루트에서)  ->  ./mujoco_menagerie/unitree_g1/
# raw.githubusercontent.com 에서 파일 63개 (약 38 MB) 를 받는다. 이미 있는 파일은 건너뜀.
import os, urllib.request

RAW = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/main/"
FILES = ['unitree_g1/CHANGELOG.md',
    'unitree_g1/LICENSE',
    'unitree_g1/README.md',
    'unitree_g1/assets/head_link.STL',
    'unitree_g1/assets/left_ankle_pitch_link.STL',
    'unitree_g1/assets/left_ankle_roll_link.STL',
    'unitree_g1/assets/left_elbow_link.STL',
    'unitree_g1/assets/left_hand_index_0_link.STL',
    'unitree_g1/assets/left_hand_index_1_link.STL',
    'unitree_g1/assets/left_hand_middle_0_link.STL',
    'unitree_g1/assets/left_hand_middle_1_link.STL',
    'unitree_g1/assets/left_hand_palm_link.STL',
    'unitree_g1/assets/left_hand_thumb_0_link.STL',
    'unitree_g1/assets/left_hand_thumb_1_link.STL',
    'unitree_g1/assets/left_hand_thumb_2_link.STL',
    'unitree_g1/assets/left_hip_pitch_link.STL',
    'unitree_g1/assets/left_hip_roll_link.STL',
    'unitree_g1/assets/left_hip_yaw_link.STL',
    'unitree_g1/assets/left_knee_link.STL',
    'unitree_g1/assets/left_rubber_hand.STL',
    'unitree_g1/assets/left_shoulder_pitch_link.STL',
    'unitree_g1/assets/left_shoulder_roll_link.STL',
    'unitree_g1/assets/left_shoulder_yaw_link.STL',
    'unitree_g1/assets/left_wrist_pitch_link.STL',
    'unitree_g1/assets/left_wrist_roll_link.STL',
    'unitree_g1/assets/left_wrist_yaw_link.STL',
    'unitree_g1/assets/logo_link.STL',
    'unitree_g1/assets/pelvis.STL',
    'unitree_g1/assets/pelvis_contour_link.STL',
    'unitree_g1/assets/right_ankle_pitch_link.STL',
    'unitree_g1/assets/right_ankle_roll_link.STL',
    'unitree_g1/assets/right_elbow_link.STL',
    'unitree_g1/assets/right_hand_index_0_link.STL',
    'unitree_g1/assets/right_hand_index_1_link.STL',
    'unitree_g1/assets/right_hand_middle_0_link.STL',
    'unitree_g1/assets/right_hand_middle_1_link.STL',
    'unitree_g1/assets/right_hand_palm_link.STL',
    'unitree_g1/assets/right_hand_thumb_0_link.STL',
    'unitree_g1/assets/right_hand_thumb_1_link.STL',
    'unitree_g1/assets/right_hand_thumb_2_link.STL',
    'unitree_g1/assets/right_hip_pitch_link.STL',
    'unitree_g1/assets/right_hip_roll_link.STL',
    'unitree_g1/assets/right_hip_yaw_link.STL',
    'unitree_g1/assets/right_knee_link.STL',
    'unitree_g1/assets/right_rubber_hand.STL',
    'unitree_g1/assets/right_shoulder_pitch_link.STL',
    'unitree_g1/assets/right_shoulder_roll_link.STL',
    'unitree_g1/assets/right_shoulder_yaw_link.STL',
    'unitree_g1/assets/right_wrist_pitch_link.STL',
    'unitree_g1/assets/right_wrist_roll_link.STL',
    'unitree_g1/assets/right_wrist_yaw_link.STL',
    'unitree_g1/assets/torso_link_rev_1_0.STL',
    'unitree_g1/assets/waist_roll_link_rev_1_0.STL',
    'unitree_g1/assets/waist_yaw_link_rev_1_0.STL',
    'unitree_g1/g1.png',
    'unitree_g1/g1.xml',
    'unitree_g1/g1_mjx.xml',
    'unitree_g1/g1_mjx_colliders.png',
    'unitree_g1/g1_with_hands.png',
    'unitree_g1/g1_with_hands.xml',
    'unitree_g1/scene.xml',
    'unitree_g1/scene_mjx.xml',
    'unitree_g1/scene_with_hands.xml']


def fetch(rel, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    req = urllib.request.Request(RAW + rel, headers={"User-Agent": "b1-week02-fetch"})
    with urllib.request.urlopen(req, timeout=120) as r, open(dst, "wb") as f:
        f.write(r.read())


if __name__ == "__main__":
    ok = 0
    for i, rel in enumerate(FILES, 1):
        dst = os.path.join("mujoco_menagerie", rel)
        if os.path.exists(dst):
            ok += 1
            continue
        print(f"  [{i}/{len(FILES)}] {rel}", flush=True)
        try:
            fetch(rel, dst)
            ok += 1
        except Exception as e:
            print("  FAILED:", rel, e)
    print(f"done: {ok}/{len(FILES)} files in mujoco_menagerie/unitree_g1")
