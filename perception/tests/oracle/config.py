"""
config.py
Workspace:  ws_jetson  |  Package: vision_navigation

Configuration Module - Vision Navigation System

Centralizes all system parameters and tunable constants.

Author: AlmondMatcha Rover Team
Date: February 27, 2026
"""


# ===================== CAMERA CONFIGURATION =====================

class CameraConfig:
    """Camera streaming parameters"""
    
    # D415 Resolution and Framerate
    WIDTH = 1280
    HEIGHT = 720
    FPS = 30
    
    # Camera modes
    OPEN_CAMERA_DISPLAY = False      # Show camera feed in OpenCV window
    ENABLE_DEPTH_STREAM = False      # Stream depth data from D415
    LOOP_VIDEO = True                # Loop video when file reaches end
    
    # File paths
    VIDEO_PATH = ""                  # Empty string = use D415 camera
    JSON_CONFIG_PATH = ""             # Path to RealSense advanced mode JSON
    
    # D415 specific
    D415_SERIAL = "806312060441"     # Specific D415 camera serial number
    
    @classmethod
    def get_resolution(cls):
        """Get camera resolution as tuple"""
        return (cls.WIDTH, cls.HEIGHT)
    
    @classmethod
    def get_fps(cls):
        """Get frames per second"""
        return cls.FPS
    
    @classmethod
    def is_video_mode(cls):
        """Check if running in video file mode"""
        return len(cls.VIDEO_PATH) > 0


# ===================== LANE DETECTION CONFIGURATION =====================

class LaneDetectionConfig:
    """Lane detection algorithm parameters"""

    # ===== Color Segmentation (adaptive) =====
    # Red-track / white-line split points are NOT fixed constants. A fixed
    # cutoff tuned against one camera's color pipeline (its white balance,
    # saturation curve, compression) does not transfer to a different sensor
    # looking at the same physical track -- confirmed against a phone photo
    # of the actual track, where a sun-glare patch on the red surface reads
    # almost identically to real white paint in HSV brightness/saturation.
    # segment_track_colors() re-fits both splits per frame with Otsu's method
    # instead: LAB a* (red-green axis) for red-vs-not-red, then L*
    # (lightness) for white-vs-not, evaluated only on pixels already on/near
    # the red blob so sunlit grass or dirt elsewhere in the ROI can't skew
    # the white cut. See docs/VISION_PIPELINE.md section 2.
    #
    # White-vs-not was originally chroma distance from neutral gray
    # (sqrt((a*-128)^2+(b*-128)^2)) with L* rejected, on the reasoning that
    # the track's own L* range under directional sun overlaps real white
    # paint's. That reasoning came from a phone photo audit only. Validated
    # against real D415 footage 2026-08-06 (dev/ captures) and found
    # backwards for this sensor: the D415 ROI's a*/b* channels span only
    # ~35-40 levels total, too narrow for Otsu to find a real bimodal chroma
    # split, so it was misclassifying ~45% of the frame (most of the plain
    # track) as white candidate. L* on the same footage has a clean bimodal
    # split (track ~110-115, paint >~170) and is what segment_track_colors()
    # uses now. Re-check this if a future camera/mount/lighting change
    # reintroduces the overlap the phone audit found.
    #
    # PLACEHOLDER, not calibrated: this kernel size was picked by eye against
    # a mobile-phone audit photo, not the D415 -- like a pixel-area count, a
    # pixel kernel size is ground-sample-distance-sensitive (depends on the
    # capturing lens/FOV). Qualitatively it still produced clean single-pixel
    # -width line masks in the 2026-08-06 D415 validation above, but that is
    # not the same as re-deriving it from a real measurement -- do that
    # before trusting this value far from the footage it was eyeballed
    # against.
    SEGMENTATION_MORPH_KERNEL_PX = 9  # Close/open kernel for the color masks

    # ===== Line shape filter (bird's-eye view) =====
    # Rejects candidate "white" blobs that aren't shaped like a painted line
    # -- e.g. a sun-glare patch on the track, which is broad and short-lived
    # in depth, unlike a real line which is a near-constant-width stripe that
    # runs the length of the ROI. Evaluated in the metric BEV canvas (see
    # BEV_PX_PER_M below) so these are physical/geometric facts about the
    # track and camera, not color values -- they hold regardless of which
    # camera produced the color-stage candidates.
    # Measured: painted lines are 5 cm wide. 0.10 m (2x measured) leaves
    # margin for warp/anti-aliasing blowup without opening the door to a
    # merged line+curb or line+glare blob passing as a single line.
    LINE_MAX_WIDTH_M = 0.10
    LINE_MIN_DEPTH_FRAC = 0.35       # Must persist over >=35% of the ROI's warped depth
    LINE_MIN_ASPECT = 1.5            # height/width in BEV -- a line is tall and narrow

    # PLACEHOLDER, not a calibrated value: picked by eye against a mobile-
    # phone audit photo (see docs/VISION_PIPELINE.md section 2), not the
    # D415. Unlike LINE_MAX_WIDTH_M/LINE_MIN_DEPTH_FRAC/LINE_MIN_ASPECT
    # (dimensionless ratios or a real metre measurement, both intrinsics-
    # independent), a raw pixel-area count is sensitive to ground-sample-
    # distance, which depends on the capturing lens/FOV -- the phone's, not
    # the D415's. Re-derive against real D415 footage before trusting it.
    MIN_LINE_COMPONENT_AREA_PX = 15  # Drops single-pixel speckle before the shape test

    # ===== Lane Finding Parameters =====
    MIN_LANE_PIXELS = 50    # Minimum pixels for valid detection
    SLIDING_WINDOWS = 9     # Number of vertical search windows
    MIN_WINDOW_PIXELS = 50  # Minimum pixels to recenter window

    # Search window half-width. MUST stay below half the spacing between
    # adjacent painted lines, or a window captures two lines at once and
    # `current_x = mean(x)` lands in the empty gap between them.
    #
    # Measured track geometry: 2.50 m red-edge-to-red-edge, three 5 cm painted
    # lines (two edges + one centre), centre line offset +-0.50 m from the
    # track's own midpoint -- NOT centred, and which side it's offset to
    # depends on which way the rover is heading, so it cannot be assumed
    # fixed. That makes the two adjacent-line gaps ASYMMETRIC:
    #     near gap  = 1.25 - 0.50 = 0.75 m
    #     far gap   = 1.25 + 0.50 = 1.75 m
    # Both must be treated as possible on either side, so the binding
    # constraint is always the SMALLER one -- 0.75 m -- regardless of which
    # way the rover is facing. 0.75 m -> 150 px at BEV_PX_PER_M -> hard
    # ceiling 75. 40 leaves margin. (Superseded an earlier, incorrect 0.61 m
    # /122 px/61 px derivation that assumed a symmetric 1.22 m lane; that
    # figure was never a real measurement of this track.)
    WINDOW_MARGIN = 40      # Horizontal search margin (±pixels)

    # Half-width of the band (around the expected lane position) that the
    # start-of-search histogram is allowed to look in. This is what stops the
    # detector locking onto the wrong painted line: with three parallel lines
    # in view, an unrestricted argmax picks whichever happens to carry the most
    # pixels, and that choice flips between frames. Must also stay below half
    # the smaller adjacent-line gap (75 px -- see WINDOW_MARGIN above).
    # Doubles as the per-frame jump limiter: the tracked line can move at
    # most this far between frames.
    # Float, not int: this is the declared default for the ROS parameter, and
    # ROS 2 derives the parameter's type from it. Declared as int, the 45.0 in
    # the params YAML is rejected as DOUBLE-vs-INTEGER and lane_detection_node
    # refuses to start.
    SEARCH_BAND_PX = 45.0

    # Absolute plausibility bound on the fitted offset, in METRES (b itself
    # is metres -- see compute_lane_params in lane_detector.py).
    #
    # SEARCH_BAND_PX limits how far the tracked line may move BETWEEN frames,
    # but it does not stop a steady walk: at 45 px/frame the search can travel
    # 900 px in 20 frames while every individual step looks legal. A field log
    # shows exactly that -- b ran from -46 px to -230 px over 19 frames
    # (-9.7 px/frame average) and settled 1.15 m off the rover's centreline,
    # outside the ROI entirely, with the detector still reporting "Detected"
    # throughout. (Previously described as "1.9 line spacings" under the old,
    # incorrect assumption of a single uniform 0.61 m spacing -- the track's
    # two adjacent-line gaps are actually asymmetric, 0.75 m and 1.75 m, see
    # WINDOW_MARGIN above, so "line spacings" is no longer a well-defined
    # unit here; the absolute distance is what matters anyway.)
    #
    # 0.50 m, which is also the downstream clamp in rover_kinematic_control.
    # Past it the value cannot reach the controller unsaturated, so it is not
    # a usable measurement whatever its true cause, and continuing to track
    # it only steers hard over on a bogus lock.
    MAX_ABS_B_M = 0.50

    # Oldest a frame may be (seconds, measured from camera_stream_node's
    # capture-time header.stamp) before this node drops it instead of
    # detecting on it.
    #
    # The QoS depth=1 on the subscription already stops a backlog from
    # accumulating -- a new frame replaces an unconsumed old one at the DDS
    # layer -- but a frame already in flight when a slowdown hits (a single
    # slow process_frame() call, scheduling jitter) can still be stale by the
    # time this node acts on it. This is the second layer: reject on
    # measured age, independent of queueing behaviour.
    #
    # 0.1 s is a few multiples of one camera period at 30 FPS (~33 ms), loose
    # enough that normal frame-to-frame jitter never trips it, tight enough
    # that a real hiccup gets caught and logged rather than silently steering
    # on old geometry while the rover has kept moving.
    MAX_FRAME_AGE_SEC = 0.1

    # ===== Perspective Transform =====
    # ROI (Region of Interest) points for bird eye view (1280x720 base).
    #
    # Derived from the physical camera mount rather than hand-authored, so the
    # warp is a true ground-plane rectification:
    #     D415 colour stream, 50 cm above the ground, tilted 15 deg down
    #     (+-3 deg mounting tolerance -- the numbers below use the nominal
    #     15 deg), mounted on the lateral centreline, 8 cm behind the front
    #     axle.
    # These four points are the image projection of the ground rectangle
    #     X = -0.90 .. +0.90 m (lateral),  Z = 1.30 .. 3.00 m (forward, from
    #     the camera) = 1.22 .. 2.92 m ahead of the front axle.
    # Regenerate these with `regenerate_roi.py` (same package) if the camera
    # height, tilt, mount position, or lens changes -- do not hand-recompute;
    # see docs/VISION_PIPELINE.md "Regenerating the ROI".
    ROI_BASE_POINTS = [
        [39, 458],       # Bottom-left   (X=-0.90 m, Z=1.30 m)
        [1241, 458],     # Bottom-right  (X=+0.90 m, Z=1.30 m)
        [915, 270],      # Top-right     (X=+0.90 m, Z=3.00 m)
        [365, 270]       # Top-left      (X=-0.90 m, Z=3.00 m)
    ]
    ROI_BASE_WIDTH = 1280.0   # Reference resolution ROI_BASE_POINTS were authored against
    ROI_BASE_HEIGHT = 720.0
    CROP_MARGIN_PX = 20.0     # Margin around the ROI bounding box before cropping
                              # (in ROI_BASE_WIDTH/HEIGHT units, scaled to actual frame size)
                              # -- trims the unwanted surroundings before the
                              # per-pixel preprocessing runs.

    # ===== Bird's-eye canvas (metric) =====
    # The warp output is a FIXED canvas, not the crop size, so the bird's-eye
    # view has the same scale in both axes. That is what makes the reported
    # values physical: theta is a real heading angle in degrees, b is a real
    # cross-track offset, and curvature is a real 1/m arc.
    # ROI_BASE_POINTS spans 1.80 m laterally and maps to the middle 50% of the
    # canvas (see PERSPECTIVE_LEFT/RIGHT_MARGIN), so:
    #     BEV_WIDTH_PX  = 1.80 m * 2 * BEV_PX_PER_M
    #     BEV_HEIGHT_PX = 1.70 m * BEV_PX_PER_M
    BEV_PX_PER_M = 200.0
    BEV_WIDTH_PX = 720
    BEV_HEIGHT_PX = 340

    # Derived: LINE_MAX_WIDTH_M expressed in BEV pixels at BEV_PX_PER_M scale.
    LINE_MAX_WIDTH_PX = LINE_MAX_WIDTH_M * BEV_PX_PER_M

    # Destination margins for warped image
    PERSPECTIVE_LEFT_MARGIN = 0.25   # 25% from left
    PERSPECTIVE_RIGHT_MARGIN = 0.75  # 75% from right
    
    # ===== Visualization =====
    SHOW_WINDOW = False              # Display lane detection output
    VISUALIZATION_WIDTH = 720        # Screen display width
    CROP_MARGIN = 100                # Crop sides for visualization
    
    @classmethod
    def get_roi_points(cls):
        """Get ROI points as numpy array"""
        import numpy as np
        return np.float32(cls.ROI_BASE_POINTS)


# ===================== STEERING CONTROL CONFIGURATION =====================

class ControlConfig:
    """Rover steering control parameters"""

    # ===== Static Feedback Gains =====
    # Control law (static-gain feedback + Ackermann feedforward, single-step
    # MPC form):
    #     steer = K_LAT*b_ema + K_HEAD*theta_ema + atan(WHEELBASE_M*curvature_m_ema)
    # theta/b/steer_angle all use "+ = correct by steering right" in this
    # codebase, so both feedback terms are ADDED (see
    # rover_kinematic_control_node.py for the sign-convention note).
    #
    # Replaces the previous PID (kp*e + ki*integral(e) + kd*de/dt) with a
    # pure proportional law. K_LAT/K_HEAD are a fixed discrete-time LQR
    # solution on the linearized error model (b_dot = v*theta,
    # theta_dot = -(v/L)*delta; v ~ 0.15-0.20 m/s, L = WHEELBASE_M,
    # dt ~ 1/20 s measured control-loop period), solved offline:
    #   LQR solve: k1 = 3.162 rad/m, k2 = 2.024 rad/rad
    #   K_LAT  = k1 * (180/pi) = 181.17  -- rad->deg (b is metres, steer is degrees)
    #   K_HEAD = k2             = 2.024  -- unit-invariant (theta and steer are both angles)
    K_HEAD = 2.024      # Gain on heading error theta (deg/deg)
    K_LAT = 181.17       # Gain on lateral offset b (deg/metre)

    # ===== Ackermann Feedforward =====
    # u_ff = atan(WHEELBASE_M * curvature_m_ema). curvature (from the lane
    # fit) is a BEV-pixel parabola coefficient A, 1/px; real curvature is
    # 1/R = 2*S*A with BEV scale S = LaneDetectionConfig.BEV_PX_PER_M.
    WHEELBASE_M = 0.4875   # Front-to-rear axle distance, metres

    # ===== Filtering =====
    EMA_ALPHA = 0.05   # Exponential moving average smoothing factor
    # Higher alpha = more responsive to new data
    # Lower alpha = more smoothing

    # ===== Output Saturation =====
    # 45.0 (not the full +-60 mechanical limit): conservative margin while
    # this static-gain law is unvalidated on the track.
    STEER_MAX_DEGREES = 45.0        # Maximum steering angle (±degrees)
    STEER_WHEN_LOST = 0.0           # Steering angle when lane not detected

    @classmethod
    def get_static_gains(cls):
        """Get static feedback gains as dict"""
        return {
            "k_lat": cls.K_LAT,
            "k_head": cls.K_HEAD
        }


# ===================== LOGGING CONFIGURATION =====================

class LoggingConfig:
    """Data logging and CSV output parameters"""
    
    # ===== File Paths =====
    LOG_DIRECTORY = "logs"           # Base directory for all logs
    CAMERA_LOG_FILE = "camera_stream.csv"
    LANE_DETECTION_LOG = "lane_pub_log.csv"
    CONTROL_LOG_FILE = "rover_ctl_log_ver_3.csv"
    
    # ===== CSV Headers =====
    CAMERA_LOG_COLUMNS = ["timestamp", "frame_id", "width", "height"]
    LANE_DETECTION_COLUMNS = ["timestamp", "theta", "b", "detected"]
    CONTROL_LOG_COLUMNS = ["time_sec", "theta_ema", "b_ema", "u", "e_sum"]
    
    # ===== Logging Control =====
    ENABLE_CAMERA_LOGGING = False
    ENABLE_LANE_LOGGING = True
    ENABLE_CONTROL_LOGGING = True
    LOG_FRAME_INTERVAL = 1           # Log every Nth frame (1 = every frame)


# ===================== TOPIC CONFIGURATION =====================

class TopicConfig:
    """ROS2 topic names - follows tpc_* convention"""
    
    # Camera topics
    RGB_STREAM = "tpc_rover_d415_rgb"
    DEPTH_STREAM = "tpc_rover_d415_depth"
    
    # Lane detection topics
    LANE_PARAMETERS = "tpc_rover_nav_lane"    # [theta, b, detected]
    
    # Control topics
    ROVER_CTRL_CMD = "tpc_rover_ctrl_cmd"   # [steer_angle, speed_cmd, detected]
    
    @classmethod
    def get_all_topics(cls):
        """Get dict of all topics"""
        return {
            "rgb": cls.RGB_STREAM,
            "depth": cls.DEPTH_STREAM,
            "lane": cls.LANE_PARAMETERS,
            "rover_ctrl_cmd": cls.ROVER_CTRL_CMD
        }


# ===================== SYSTEM CONFIGURATION =====================

class SystemConfig:
    """Overall system parameters"""
    
    # ===== ROS2 Domain =====
    ROS_DOMAIN_ID = 5               # Domain for inter-robot communication
    
    # ===== Node Names =====
    CAMERA_NODE_NAME = "camera_stream_node"
    LANE_NODE_NAME = "lane_detection_node"
    CONTROL_NODE_NAME = "rover_kinematic_control"
    
    # ===== Initialization Timing =====
    CAMERA_INIT_DELAY = 2.0         # Seconds: wait for camera to initialize
    LANE_INIT_DELAY = 0.5           # Seconds: wait after camera ready
    CONTROL_INIT_DELAY = 0.5        # Seconds: wait after lane detection ready
    
    # ===== Quality of Service =====
    QOS_DEPTH = 5                   # Message queue depth
    QOS_RELIABILITY = "BEST_EFFORT" # "BEST_EFFORT" or "RELIABLE"
    
    # ===== Debug/Development =====
    DEBUG_MODE = False               # Extra logging and visualization
    PROFILE_PERFORMANCE = False      # Measure execution time


# ===================== HELPER: CONFIGURATION OVERRIDE ======================

def override_from_environment():
    """Override config values from environment variables.
    
    Pattern: MODULENAME_PARAMNAME (e.g., CAMERA_WIDTH, CONTROL_K_LAT)
    """
    import os

    # Camera overrides
    if "CAMERA_WIDTH" in os.environ:
        CameraConfig.WIDTH = int(os.environ["CAMERA_WIDTH"])
    if "CAMERA_HEIGHT" in os.environ:
        CameraConfig.HEIGHT = int(os.environ["CAMERA_HEIGHT"])
    if "CAMERA_FPS" in os.environ:
        CameraConfig.FPS = int(os.environ["CAMERA_FPS"])

    # Control overrides
    if "CONTROL_K_LAT" in os.environ:
        ControlConfig.K_LAT = float(os.environ["CONTROL_K_LAT"])
    if "CONTROL_K_HEAD" in os.environ:
        ControlConfig.K_HEAD = float(os.environ["CONTROL_K_HEAD"])




if __name__ == "__main__":
    """Display configuration values"""
    print("Vision Navigation System Configuration")
    print("=" * 50)
    print(f"\nCamera Configuration:")
    print(f"  Resolution: {CameraConfig.get_resolution()}")
    print(f"  FPS: {CameraConfig.get_fps()}")
    print(f"  Video mode: {CameraConfig.is_video_mode()}")
    
    print(f"\nControl Configuration:")
    print(f"  Static gains: {ControlConfig.get_static_gains()}")
    print(f"  Max steering: {ControlConfig.STEER_MAX_DEGREES} degrees")
    
    print(f"\nLogging Configuration:")
    print(f"  Directory: {LoggingConfig.LOG_DIRECTORY}")
    print(f"  Enabled: Lane={LoggingConfig.ENABLE_LANE_LOGGING}, "
          f"Control={LoggingConfig.ENABLE_CONTROL_LOGGING}")
    
    print(f"\nTopic Configuration:")
    for name, topic in TopicConfig.get_all_topics().items():
        print(f"  {name}: {topic}")
