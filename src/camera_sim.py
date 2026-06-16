import pybullet as p
import pybullet_data
import time
import numpy as np
import collections
import os
import threading
import uuid

# ----------------------------
# Setup
# ----------------------------
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF("plane.urdf")

# simple robot substitute (replace with OpenArm URDF later)
robot = p.loadURDF("r2d2.urdf", [0, 0, 1])

num_joints = p.getNumJoints(robot)

dt = 1.0 / 240.0
t = 0.0

# ----------------------------
# Camera definitions
# ----------------------------

def get_camera(view, proj):
    img = p.getCameraImage(
        width=320,
        height=240,
        viewMatrix=view,
        projectionMatrix=proj
    )
    rgb = img[2]
    return np.array(rgb)

def make_view(pos, target):
    return p.computeViewMatrix(
        cameraEyePosition=pos,
        cameraTargetPosition=target,
        cameraUpVector=[0, 0, 1]
    )

proj = p.computeProjectionMatrixFOV(
    fov=60,
    aspect=1.0,
    nearVal=0.1,
    farVal=10
)

# ----------------------------
# Fixed cameras (simulating OpenArm setup)
# ----------------------------

# ceiling camera (world fixed)
view_ceiling = make_view([0, 0, 2], [0, 0, 0])

# wrist cameras (fake attachment using robot position)
def get_wrist_views():
    pos, orn = p.getBasePositionAndOrientation(robot)

    left = make_view([pos[0]-0.2, pos[1], pos[2]+0.2], pos)
    right = make_view([pos[0]+0.2, pos[1], pos[2]+0.2], pos)
    return left, right

# ZED stereo (baseline offset)
def get_zed_views():
    pos, _ = p.getBasePositionAndOrientation(robot)

    baseline = 0.05
    left = make_view([pos[0], pos[1]-baseline, pos[2]+0.3], pos)
    right = make_view([pos[0], pos[1]+baseline, pos[2]+0.3], pos)
    return left, right

# ----------------------------
# Buffers (simulate async sensors)
# ----------------------------

joint_buffer = collections.deque(maxlen=1000)

cam_buffers = {
    "wrist_left": collections.deque(maxlen=1000),
    "wrist_right": collections.deque(maxlen=1000),
    "ceiling": collections.deque(maxlen=1000),
    "zed_left": collections.deque(maxlen=1000),
    "zed_right": collections.deque(maxlen=1000),
}

# episode storage
episodes_dir = os.path.join(os.path.dirname(__file__), "..", "data", "episodes")
os.makedirs(episodes_dir, exist_ok=True)

# recording state
recording = False
current_episode = []


def save_episode(episode_samples):
    """Save an episode (list of samples) to a compressed npz file.

    Each sample is a dict with keys: t, joint_state (q,dq), images (per cam), image_ts (per cam)
    Images are stored as uint8 arrays; to keep the example simple we store raw arrays.
    """
    if not episode_samples:
        print("No samples to save")
        return None

    fname = f"episode_{time.strftime('%Y%m%d_%H%M%S')}_{uuid.uuid4().hex[:8]}.npz"
    path = os.path.join(episodes_dir, fname)

    # Build arrays/lists to save. For simplicity save timestamps and joint arrays and pickled images.
    ts = np.array([s['t'] for s in episode_samples])
    qs = np.array([s['joint_state']['q'] for s in episode_samples])
    dqs = np.array([s['joint_state']['dq'] for s in episode_samples])

    # images are variable size arrays; store as object array via numpy.savez_compressed
    imgs = {}
    img_ts = {}
    cams = ['wrist_left', 'wrist_right', 'ceiling', 'zed_left', 'zed_right']
    for cam in cams:
        imgs[cam] = [s['images'][cam] for s in episode_samples]
        img_ts[cam] = np.array([s.get('image_ts', {}).get(cam, np.nan) for s in episode_samples])

    # Use numpy.savez_compressed with allow_pickle for heterogeneous lists
    np.savez_compressed(path,
                        ts=ts,
                        q=qs,
                        dq=dqs,
                        **{f"{c}_imgs": np.array(imgs[c], dtype=object) for c in cams},
                        **{f"{c}_ts": img_ts[c] for c in cams})

    print(f"Saved episode {path} ({len(episode_samples)} samples)")
    return path

# ----------------------------
# FPS simulation
# ----------------------------

step = 0

def should_sample(fps, step):
    return step % int(240 / fps) == 0

# ----------------------------
# Main loop
# ----------------------------
while True:
    p.stepSimulation()
    t += dt
    step += 1

    # ----------------------------
    # JOINT STATE (CAN simulation)
    # ----------------------------
    joints = p.getJointStates(robot, range(num_joints))
    q = [j[0] for j in joints]
    dq = [j[1] for j in joints]

    # store precise timestamped joint state
    joint_buffer.append((t, np.array(q, dtype=float), np.array(dq, dtype=float)))

    # ----------------------------
    # CAMERA 1: wrist (30 FPS)
    # ----------------------------
    if should_sample(30, step):
        wl, wr = get_wrist_views()

        cam_buffers["wrist_left"].append((t, get_camera(wl, proj)))
        cam_buffers["wrist_right"].append((t, get_camera(wr, proj)))

    # ----------------------------
    # CAMERA 2: ceiling (15 FPS)
    # ----------------------------
    if should_sample(15, step):
        cam_buffers["ceiling"].append((t, get_camera(view_ceiling, proj)))

    # ----------------------------
    # CAMERA 3: ZED stereo (60 FPS)
    # ----------------------------
    if should_sample(60, step):
        zl, zr = get_zed_views()

        cam_buffers["zed_left"].append((t, get_camera(zl, proj)))
        cam_buffers["zed_right"].append((t, get_camera(zr, proj)))

    # ----------------------------
    # SYNCHRONIZATION (dataset creation)
    # ----------------------------

    # choose master timestamp = joint stream; wait for at least two joint states for interpolation
    if len(joint_buffer) < 2:
        continue

    t_master, q_master, dq_master = joint_buffer[-1]

    def get_latest_with_ts(cam):
        return cam_buffers[cam][-1] if len(cam_buffers[cam]) > 0 else (np.nan, None)

    # Helper: interpolate joint state to arbitrary timestamp using nearest two samples
    def interp_joint(t_query):
        # find two samples surrounding t_query in joint_buffer
        buf = list(joint_buffer)
        times = [s[0] for s in buf]
        if t_query <= times[0]:
            return buf[0][1], buf[0][2]
        if t_query >= times[-1]:
            return buf[-1][1], buf[-1][2]

        # binary search
        import bisect
        i = bisect.bisect_left(times, t_query)
        t0, q0, dq0 = buf[i-1]
        t1, q1, dq1 = buf[i]
        if t1 == t0:
            return q0, dq0
        alpha = (t_query - t0) / (t1 - t0)
        q = q0 * (1 - alpha) + q1 * alpha
        dq = dq0 * (1 - alpha) + dq1 * alpha
        return q, dq

    # Build sample keyed by joint master time, but include per-camera image timestamps and joint states
    images = {}
    image_ts = {}
    cams = ["wrist_left", "wrist_right", "ceiling", "zed_left", "zed_right"]
    for cam in cams:
        ts_cam, img = get_latest_with_ts(cam)
        images[cam] = img
        image_ts[cam] = ts_cam

    # Also build per-camera-aligned joint states (interpolated to image timestamps)
    joints_by_cam = {}
    for cam in cams:
        ts_cam = image_ts[cam]
        if img is None or np.isnan(ts_cam):
            joints_by_cam[cam] = {'q': None, 'dq': None}
        else:
            q_i, dq_i = interp_joint(ts_cam)
            joints_by_cam[cam] = {'q': q_i, 'dq': dq_i}

    sample = {
        "t": t_master,

        "joint_state": {
            "q": q_master,
            "dq": dq_master
        },

        "images": images,
        "image_ts": image_ts,
        "joints_by_cam": joints_by_cam,
    }

    # ----------------------------
    # DEBUG OUTPUT (pretend dataset logging)
    # ----------------------------
    if step % 120 == 0:
        cams_ready = sum([1 for c in cams if sample['images'][c] is not None])
        print("t:", round(sample["t"], 3),
              "| joints:", len(sample["joint_state"]["q"]),
              f"| cams ready: {cams_ready}/{len(cams)}")

    # Recording toggle via keyboard events (press 'r' to start/stop) - use pybullet keyboard events
    keys = p.getKeyboardEvents()
    if ord('r') in keys and keys[ord('r')] & p.KEY_WAS_TRIGGERED:
        recording = not recording
        if recording:
            current_episode = []
            print("Recording started")
        else:
            print("Recording stopped, saving...")
            # save in background
            threading.Thread(target=save_episode, args=(current_episode,)).start()

    # If recording, append sample (make images small or None if missing)
    if recording:
        # keep small payload: downsample images to 160x120 and convert to uint8
        sample_copy = {
            't': sample['t'],
            'joint_state': {'q': sample['joint_state']['q'].tolist(), 'dq': sample['joint_state']['dq'].tolist()},
            'images': {},
            'image_ts': sample['image_ts']
        }
        for cam in cams:
            img = sample['images'][cam]
            if img is None:
                sample_copy['images'][cam] = None
            else:
                # downsample by simple slicing
                small = img[::2, ::2, :].astype(np.uint8)
                sample_copy['images'][cam] = small

        current_episode.append(sample_copy)