# Stereo Camera and IMU ROS1/ROS2 Bag Conversion Pipeline for Calibration and Data Processing

A Dockerized data-ingestion and calibration pipeline for synchronizing multi-camera video feeds (GoPro, DJI, or generic MP4/MOV) with high-rate IMU telemetry into **ROS 1 `.bag`** files (for [Kalibr](https://github.com/ethz-asl/kalibr) intrinsic, stereo, and camera–IMU calibration) and **LZ4-compressed ROS 2 `.mcap`** bags (for ROS 2 Jazzy/Humble SLAM, VIO, and 3D reconstruction).

Running Kalibr and the conversion pipeline inside an isolated Ubuntu 20.04 / ROS 1 Noetic Docker container keeps modern host systems (such as **Ubuntu 24.04 with ROS 2 Jazzy**) completely clean while outputting native ROS 2 MCAP bags directly.

---

## Features

* **Zero-Config Embedded IMU Extraction:** Extracts hardware-synchronized gyroscope and accelerometer streams directly from GoPro (`gpmd`) and DJI video files via [`telemetry-parser`](https://github.com/AdrianEddy/telemetry-parser), or parses external Gyroflow CSV exports.
* **True SI Units & Nanosecond Alignment:** Automatically scales angular velocity to $\text{rad/s}$ and linear acceleration to $\text{m/s}^2$, trims recordings, and aligns secondary cameras/IMUs via a sub-millisecond `--offset_sec` shift on a 19-digit Unix epoch nanosecond base.
* **Dual-Format Output (`--format ros1 | ros2 | both`):**
  * **ROS 1 (`.bag`):** Writes `mono8` grayscale image topics (`/cam0/image_raw`, `/cam1/image_raw`) and `/imu0`, `/imu1` compatible with `kalibr_calibrate_cameras` and `kalibr_calibrate_imu_camera`.
  * **ROS 2 (`_ros2/`):** Writes full-color `bgr8` image topics and IMU telemetry into **LZ4-compressed `.mcap`** containers (or legacy `.db3` SQLite3) with valid ROS 2 Jazzy `metadata.yaml`.
* **Bounded-RAM Direct Streaming (`--direct`):** Streams decoded frames and IMU batches straight from video containers into ROS bags using a bounded multiprocessing queue—eliminating tens of gigabytes of temporary `.png` files.
* **Flexible Sensor Topologies:** Supports **Stereo + Dual IMU**, **Stereo + Single IMU**, **Monocular + IMU**, **Camera-Only** (intrinsic calibration), and **IMU-Only** (Allan Variance noise characterization).

---

## Installation & Setup

### 1. Host Prerequisites (Ubuntu 22.04 / 24.04)

Install Docker, `ffmpeg` (for lossless MP4 chapter merging), and `mpv` (for frame-accurate sync offset inspection):

```bash
sudo apt-get update && sudo apt-get install -y \
  docker.io \
  git \
  ffmpeg \
  mpv

sudo usermod -aG docker $USER
newgrp docker
```

*(Optional)* For IMU Allan Variance characterization on **ROS 2 Jazzy**, install [`allan_variance_ros2`](https://github.com/Autoliv-Research/allan_variance_ros2) in your ROS 2 workspace.

### 2. Cloning the GitHub Repo

Cloning the stereo_camera_and_imu_to_rosbag_converter GitHub Repo:

```bash
git clone https://github.com/AqeelGhani/stereo_camera_and_imu_to_rosbag_converter.git
cd ./stereo_camera_and_imu_to_rosbag_converter
```

### 3. Build the Kalibr Base Image

ETHZ-ASL does not publish pre-built Kalibr images on Docker Hub. Run the helper script to compile the `kalibr` base image locally (one-time setup, ~15–30 minutes):

```bash
chmod +x scripts/build_kalibr_base.sh
./scripts/build_kalibr_base.sh
```

### 4. Build the Converter Image

Build the unified pipeline image on top of `kalibr`:

```bash
docker build -t stereo_camera_and_imu_to_rosbag_converter .
```

---

## Data Preparation

### 1. Merge Split MP4 Chapters (If Applicable)

Action cameras split long recordings into ~4 GB chapters (`GX010001.MP4`, `GX020001.MP4`, ...). Standard video editors strip the embedded IMU telemetry track. Merge chapters losslessly while preserving all data streams:

```bash
printf "file '%s'\n" GX010001.MP4 GX020001.MP4 > files.txt
ffmpeg -f concat -safe 0 -i files.txt -c copy -map 0 merged_cam0.mp4
```

### 2. Measure the Camera Sync Offset (`--offset_sec`)

If using two unsynchronized cameras (`cam0` and `cam1`), trigger a sharp visual event visible to both lenses (e.g., a clap, flashlight blink, or sudden board motion) and find the exact timestamps in `mpv`:

```bash
mpv --osd-fractions=yes data/cam0.mp4
mpv --osd-fractions=yes data/cam1.mp4
```

* Press **`Space`** to pause near the sync event, and use **`,` (comma)** and **`.` (period)** to step backward/forward single frames.
* Compute the offset:
  $$\mathrm{offset\_sec} = t_{\mathrm{cam0}} - t_{\mathrm{cam1}}$$
  *(For example, if the event occurs at `12.450s` in `cam0` and `11.449s` in `cam1`, pass `--offset_sec 1.001`.)*[cite: 15]

---

## Stage 1: Bag Generation (`stereo_cam_imu_to_rosbag.py`)

All `docker run` commands mount your current working directory (`-v $(pwd):/data`) and pass `--user $(id -u):$(id -g) -e HOME=/tmp` so generated bags are owned by your normal Linux user rather than `root`.

### Example A: Stereo + Dual Embedded IMU (ROS 1 `.bag` + ROS 2 `.mcap`)

Generates both a grayscale ROS 1 bag for Kalibr (`calib_run.bag`) and a full-color LZ4-compressed ROS 2 MCAP bag (`calib_run_ros2/`) using direct RAM streaming:

```bash
docker run -it --rm \
  --user $(id -u):$(id -g) \
  -e HOME=/tmp \
  -v $(pwd):/data \
  stereo_camera_and_imu_to_rosbag_converter \
  --cam0 data/cam0.mp4 --tp_cam0 \
  --cam1 data/cam1.mp4 --tp_cam1 \
  --offset_sec 1.001 \
  --start_trim 15.0 \
  --end_trim 120.0 \
  --fps 20 \
  --direct \
  --output_bag calib_run \
  --format both
```

### Example B: Monocular Camera + External Gyroflow CSV

```bash
docker run -it --rm \
  --user $(id -u):$(id -g) \
  -e HOME=/tmp \
  -v $(pwd):/data \
  stereo_camera_and_imu_to_rosbag_converter \
  --cam0 data/cam0.mp4 \
  --imu0 data/cam0_gyroflow.csv \
  --start_trim 10.0 \
  --end_trim 90.0 \
  --direct \
  --output_bag mono_vio \
  --format ros1
```

### Example C: IMU-Only Extraction (for Allan Variance)

Extracts only the IMU stream from a stationary recording without decoding video frames:

```bash
docker run -it --rm \
  --user $(id -u):$(id -g) \
  -e HOME=/tmp \
  -v $(pwd):/data \
  stereo_camera_and_imu_to_rosbag_converter \
  --cam0 data/stationary_imu.mp4 --tp_cam0 --no_video \
  --start_trim 600 \
  --direct \
  --output_bag imu_allan \
  --format ros2
```

### CLI Argument Reference

| Flag | Default | Description |
| :--- | :--- | :--- |
| `--cam0` | `None` | Path to Master video file (relative to `$(pwd)`). |
| `--cam1` | `None` | Path to Secondary video file (enables Stereo mode). |
| `--tp_cam0` / `--tp_cam1` | `False` | Extract embedded IMU telemetry from `--cam0` / `--cam1` via `telemetry-parser`. |
| `--imu0` / `--imu1` | `None` | Path to external Gyroflow IMU CSV (mutually exclusive with `--tp_cam*`). |
| `--no_video` | `False` | Skip video frame decoding (used with `--tp_cam0` for IMU-only bags). |
| `--offset_sec` | `None` | Time offset $t_{\text{cam0}} - t_{\text{cam1}}$ in seconds (required if `--cam1`/`--imu1`/`--tp_cam1` is used). |
| `--start_trim` | `0.0` | Start timestamp in Master camera seconds. |
| `--end_trim` | `EOF` | End timestamp in Master camera seconds. |
| `--fps` | Native | Optional target FPS for video downsampling (IMU stays at full native rate). |
| `--output_bag` | `calib_data` | Base output name (produces `<name>.bag` and/or `<name>_ros2/`). |
| `--format` | `ros1` | Output format: `ros1` (Kalibr `mono8`), `ros2` (color `bgr8`), or `both`. |
| `--storage` | `mcap` | ROS 2 storage backend: `mcap` (LZ4-compressed, default) or `sqlite3` (`.db3`). |
| `--direct` | `False` | Stream frames/IMU straight into bags via bounded RAM queue (zero temp PNGs). |
| `--queue_size` | `32` | Maximum buffered frames/batches in RAM when `--direct` is enabled. |

---

## Stage 2: IMU Noise Characterization (`allan_variance_ros2`)

Before running camera–IMU calibration, characterize your IMU's white noise and bias random walk:

1. **Record Stationary Data:** Leave the camera/IMU completely still on a vibration-free surface for **2 to 4 hours**.
2. **Convert to ROS 2 MCAP:** Run Example C above (`--no_video --format ros2`) to generate `imu_allan_ros2/`.
3. **Compute Allan Deviation (ROS 2 Jazzy):**
   ```bash
   ros2 run allan_variance_ros2 allan_variance \
     imu_allan_ros2 \
     config/allan_variance.yaml

   python3 <path_to_ros2_ws>/src/allan_variance_ros2/src/allan_variance_ros2/scripts/analysis.py --data allan_variance.csv
   ```
4. **Populate `imu0.yaml`:** Transfer the resulting `accelerometer_noise_density`, `accelerometer_random_walk`, `gyroscope_noise_density`, and `gyroscope_random_walk` into `config/imu_template.yaml` (save as `imu0.yaml`).

---

## Stage 3: Camera & Stereo Extrinsic Calibration (Kalibr)

### 1. Configure `target.yaml`

Copy `config/target_aprilgrid.yaml` into your working folder as `target.yaml` and enter your physical board dimensions in meters:

```yaml
target_type: 'aprilgrid'
tagCols: 6               # Number of AprilTags horizontally
tagRows: 6               # Number of AprilTags vertically
tagSize: 0.088           # Edge-to-edge width of ONE black tag square [m]
tagSpacing: 0.3          # Ratio of gap between tags to tagSize
```

### 2. Launch an Interactive Kalibr Shell

Pass `--shm-size=4g` so Kalibr's parallel AprilGrid corner extractor does not exhaust Docker's default 64 MB `/dev/shm`:

```bash
docker run -it --rm \
  --shm-size=4g \
  --user $(id -u):$(id -g) \
  -e HOME=/tmp \
  --entrypoint bash \
  -v $(pwd):/data \
  stereo_camera_and_imu_to_rosbag_converter
```

Inside the container prompt, source ROS 1 Noetic and Kalibr:

```bash
source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash
```

---

### 3. Stereo Camera Intrinsic & Extrinsic Calibration (`kalibr_calibrate_cameras`)

Kalibr solves for both cameras' intrinsics **and** the rigid 6-DOF stereo extrinsic transformation ($T_{\text{cam1, cam0}}$) simultaneously when both image topics are passed to `kalibr_calibrate_cameras`.

> **Recording Tip for Stereo Extrinsics:** Ensure the calibration target is simultaneously visible in **both** `cam0` and `cam1` across varied distances, tilts, and positions across the overlapping field of view.

Use `--bag-freq 4` to subsample dense 30–120 fps bags down to ~4 Hz (preventing memory bloat while preserving geometric diversity) and `--dont-show-report` for headless execution:

```bash
rosrun kalibr kalibr_calibrate_cameras \
  --bag calib_run.bag \
  --topics /cam0/image_raw /cam1/image_raw \
  --models pinhole-equi pinhole-equi \
  --target target.yaml \
  --bag-freq 4 \
  --dont-show-report
```

* **Lens Model Selection (`--models`):**
  * `pinhole-equi` (equidistant/fisheye): Recommended for wide-angle action cameras (GoPro, DJI)[cite: 1].
  * `pinhole-radtan` (radial-tangential): Recommended for standard rectilinear lenses[cite: 1].
  * Models can be mixed if your stereo rig uses two different lenses (e.g., `--models pinhole-equi pinhole-radtan`).
* **Monocular-Only Option:** If calibrating a single camera's intrinsics alone, pass only `--topics /cam0/image_raw --models pinhole-equi`[cite: 2].
* **Using Pre-Calibrated Intrinsics (Optional):** If you already calibrated `cam0` and `cam1` individually and want to initialize stereo extrinsics from existing files, merge them into a seed `initial_camchain.yaml` and pass `--init-focal-length` / `--cams` or let `kalibr_calibrate_cameras` refine intrinsics + extrinsics jointly (joint optimization is generally recommended so baseline and focal lengths are globally consistent).

#### Validating the Stereo Output (`calib_run-camchain.yaml` & `calib_run-report-cam.pdf`)

1. **Check Reprojection Error (`calib_run-report-cam.pdf`):** Both `cam0` and `cam1` should achieve a mean reprojection error **< 1.0 px** (ideally **< 0.5 px**)[cite: 1].
2. **Verify the Stereo Baseline (`T_cn_cnm1` in `calib_run-camchain.yaml`):**
   Kalibr writes the extrinsic transform from `cam0` to `cam1` under `cam1 -> T_cn_cnm1` (a $4 \times 4$ homogeneous transformation matrix $[R \mid t]$):
   ```yaml
   cam1:
     T_cn_cnm1:
     - [0.9998, -0.0041,  0.0182, -0.1195]   # tx = -0.1195 m (~12.0 cm baseline)
     - [0.0043,  0.9999, -0.0110,  0.0012]   # ty
     - [-0.0181, 0.0111,  0.9998,  0.0034]   # tz
     - [0.0,     0.0,     0.0,     1.0]
   ```
   * Measure the physical center-to-center distance between your left (`cam0`) and right (`cam1`) camera lenses with a ruler and confirm it matches $\Vert{}t\Vert{} = \sqrt{t_x^2 + t_y^2 + t_z^2}$ within a few millimeters.
   * For a standard horizontal side-by-side stereo rig (`cam0` = left, `cam1` = right), $t_x$ should be negative (roughly `-baseline_meters`), while $t_y$ and $t_z$ should be close to `0.0`.

---

### 4. Camera–IMU Extrinsics & Timeshift (`kalibr_calibrate_imu_camera`)

Once `calib_run-camchain.yaml` is validated, run `kalibr_calibrate_imu_camera` on the full-framerate bag (do **not** use `--bag-freq` here, as continuous frame-to-frame motion is needed to align with IMU integration)[cite: 1, 5]:

```bash
rosrun kalibr kalibr_calibrate_imu_camera \
  --bag calib_run.bag \
  --cams calib_run-camchain.yaml \
  --imu imu0.yaml \
  --target target.yaml \
  --dont-show-report
```

*(If calibrating two IMUs simultaneously, pass `--imu imu0.yaml imu1.yaml`[cite: 1]. If a high-FPS bag exceeds RAM during corner extraction, add `--bag-from-to <start_sec> <end_sec>` to calibrate on a 30–60 second window of active multi-axis motion[cite: 5].)*

The final calibrated intrinsics, stereo extrinsics ($T_{\text{cam1, cam0}}$), camera-to-IMU extrinsics ($T_{\text{cam, imu}}$), and hardware clock offset (`timeshift_cam_imu`) are saved to `calib_run-camchain-imucam.yaml`[cite: 1].

---

## ROS 2 MCAP Playback & Troubleshooting

### Smooth Playback of High-Resolution Stereo MCAP Bags

Uncompressed 1080p `bgr8` frames are ~6.22 MB each. When playing back stereo `.mcap` bags in ROS 2, keep `--read-ahead-queue-size` low (`100` $\approx 620\text{ MB}$ RAM) so `ros2 bag play` does not trigger the Linux OOM killer:

* **For 3D Reconstruction / VIO / SLAM (lossless `RELIABLE` QoS):**
  ```bash
  ros2 bag play calib_run_ros2 \
    --clock \
    --rate 0.25 \
    --read-ahead-queue-size 100
  ```
* **For Live GUI Inspection (`rqt_image_view` with `BEST_EFFORT` QoS):**
  ```bash
  ros2 bag play calib_run_ros2 \
    --qos-profile-overrides-path config/qos_override.yaml \
    --topics /cam0/image_raw /cam1/image_raw \
    --read-ahead-queue-size 100
  ```

### Common Issues

| Symptom | Cause | Fix |
| :--- | :--- | :--- |
| `kalibr_calibrate_cameras: command not found` | Catkin executables live in `/catkin_ws/devel/lib/kalibr/`. | Prefix commands with `rosrun kalibr ...`. |
| `ConnectionResetError: [Errno 104]` during corner extraction | Docker's 64 MB `/dev/shm` filled up on a high-FPS bag. | Launch Docker with `--shm-size=4g` and pass `--bag-freq 4` to `kalibr_calibrate_cameras`. |
| `ros2 bag play` exits with `Killed` | `--read-ahead-queue-size` buffered too many 6.22 MB raw frames in RAM. | Set `--read-ahead-queue-size 100`. |
| `FileNotFoundError: target.yaml` inside container | `target.yaml` is not in the host folder mounted to `/data`. | Place `target.yaml` in the same directory where you execute `docker run`. |