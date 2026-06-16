# OpenArm 2.0 — Data Collection Pipeline (Take-home)

This repo implements a simulated data collection pipeline for the OpenArm 2.0 robotic arm. Because of lack of hardware access, Task 1 was skipped, for Tasks 2 and 3 the CAN data stream and camera inputs were simulated. It includes:

- A PyBullet-based simulator that produces joint states and 4 camera streams (`src/camera_sim.py`).
- A small REST API to list and download saved episodes (`src/api.py`).
- Utilities to record episodes (saved as compressed NumPy `.npz` files) into `data/episodes`.

What I completed
- Task 2 (CAN data reading): simulated CAN / joint-state stream. Joint states are timestamped, and buffered so that each arm's postion is different to the other's. 

- Task 3 (Multi-camera synchronization): Used PyBullet to simulate the robot environment and to produce camera images,implemented per-camera sampling of frames at different FPS, stored per-image timestamps, and implemented interpolation of joint states to camera timestamps so each image can be associated with the closest/interpolated joint state.


- Task 4 (Data storage backend): implemented a simple storage format using compressed NumPy archives and a small Flask API to list and download episodes. Justification below.

Assumptions
- No hardware was available, so CAN FD and cameras are simulated in software using PyBullet.
- Cameras have different frame rates: wrist cameras 30 FPS, ceiling 15 FPS, ZED stereo 60 FPS.
- Images are stored as raw RGB arrays in `.npz` using object arrays for simplicity. 


Design and rationale

- Syncronization: keep a fast joint-state stream and timestamp each camera frame. For each image, linearly interpolate the two surrounding joint samples to get the joint state at the image time. This is simple and keeps timing accurate across different camera frame rates.
- Storage: use compressed NumPy (`.npz`) for quick prototyping and easy inspection. This works for demos; for real data collection prefer MCAP or chunked formats (HDF5/Zarr) and compress images per-frame.
- API: small Flask app to list and download episode files; useful for quick inspection and sharing.

Next steps if given ahrdware access:

1. Wire up real inputs
	- Replace mocked joint stream with a real CAN reader process that publishes timestamped joint states. Ensure the clock used for CAN timestamps is consistent (record offsets if needed).
	- Replace PyBullet camera frames with real camera captures (Arducam/ZED). Timestamp frames at capture and push to the same per-camera buffers.

2. Validate synchronization
	- Run short recordings and verify image timestamps align with interpolated joint states. Visual checks: overlay joint/pose values on image frames or plot joint time-series next to frame times.

3. Storage and performance
	- Switch to MCAP or per-frame JPEG+metadata for production-size recordings. Test end-to-end write throughput (images/sec) and disk usage.



