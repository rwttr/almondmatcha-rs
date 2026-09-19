"""Jetson perception process: camera capture + lane detection in one process.

No ROS. See `docs/RUST_REWRITE_PLAN.md` §1.1/§1.2/§9 step 11 for why this
replaces two ROS 2 nodes (`camera_stream_node`, `lane_detection_node`) with
one: the camera-to-detector hop was 1:1 on the same machine, so the ROS
round trip (a `cv_bridge` encode, a ~2.7 MB serialize, a localhost UDP hop,
a decode) bought nothing but latency, once per frame, at 30 FPS.
"""
