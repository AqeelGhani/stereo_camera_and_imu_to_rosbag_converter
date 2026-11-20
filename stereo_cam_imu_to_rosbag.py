import cv2
import csv
import math
import os
import sys
import argparse
import subprocess
import shutil
import numpy as np
from multiprocessing import Process, Queue
from tqdm import tqdm

# Avoid Ubuntu apport getpwuid() crash noise when running Docker with --user $(id -u):$(id -g)
sys.excepthook = sys.__excepthook__

# Native ROS 1 Noetic Python packages inside the Kalibr container
sys.path.append("/opt/ros/noetic/lib/python3/dist-packages")
import rospy
import rosbag
from sensor_msgs.msg import Image as Ros1Image, Imu as Ros1Imu

import telemetry_parser
from mcap.writer import Writer as McapWriter, CompressionType
from rosbags.rosbag2 import Writer as Rosbag2Writer
from rosbags.typesys import Stores, get_typestore

_TS_ROS2 = get_typestore(Stores.ROS2_HUMBLE)
Time = _TS_ROS2.types['builtin_interfaces/msg/Time']
Quaternion = _TS_ROS2.types['geometry_msgs/msg/Quaternion']
Vector3 = _TS_ROS2.types['geometry_msgs/msg/Vector3']
Image = _TS_ROS2.types['sensor_msgs/msg/Image']
Imu = _TS_ROS2.types['sensor_msgs/msg/Imu']
Header = _TS_ROS2.types['std_msgs/msg/Header']
serialize_cdr = _TS_ROS2.serialize_cdr

# Ensures 19-digit nanosecond timestamps even if start_trim is 0.0s
BASE_EPOCH_NS = 1700000000000000000
DEG2RAD = math.pi / 180.0
IMU_BATCH_SIZE = 2000

ROS2_MSG_SCHEMAS = {
    "sensor_msgs/msg/Image": (
        "std_msgs/Header header\n"
        "uint32 height\n"
        "uint32 width\n"
        "string encoding\n"
        "uint8 is_bigendian\n"
        "uint32 step\n"
        "uint8[] data\n"
        "================================================================================\n"
        "MSG: std_msgs/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n"
    ),
    "sensor_msgs/msg/Imu": (
        "std_msgs/Header header\n"
        "geometry_msgs/Quaternion orientation\n"
        "float64[9] orientation_covariance\n"
        "geometry_msgs/Vector3 angular_velocity\n"
        "float64[9] angular_velocity_covariance\n"
        "geometry_msgs/Vector3 linear_acceleration\n"
        "float64[9] linear_acceleration_covariance\n"
        "================================================================================\n"
        "MSG: std_msgs/Header\n"
        "builtin_interfaces/Time stamp\n"
        "string frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/Time\n"
        "int32 sec\n"
        "uint32 nanosec\n"
        "================================================================================\n"
        "MSG: geometry_msgs/Quaternion\n"
        "float64 x 0\n"
        "float64 y 0\n"
        "float64 z 0\n"
        "float64 w 1\n"
        "================================================================================\n"
        "MSG: geometry_msgs/Vector3\n"
        "float64 x\n"
        "float64 y\n"
        "float64 z\n"
    ),
}


class McapRos2Writer:
    """Writes a ROS 2-compatible MCAP bag directory (with LZ4 chunk compression + metadata.yaml)."""

    def __init__(self, bag_dir):
        self.bag_dir = bag_dir
        bag_name = os.path.basename(os.path.normpath(bag_dir))
        self.mcap_filename = f"{bag_name}_0.mcap"
        self.mcap_path = os.path.join(bag_dir, self.mcap_filename)
        self._fh = None
        self._writer = None
        self._schemas = {}
        self._channels = {}
        self._min_ts = None
        self._max_ts = None
        self._total_msgs = 0

    def open(self):
        os.makedirs(self.bag_dir, exist_ok=True)
        self._fh = open(self.mcap_path, "wb")
        self._writer = McapWriter(self._fh, compression=CompressionType.LZ4)
        self._writer.start(profile="ros2", library="cam_vid_imu_to_rosbag")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()

    def add_connection(self, topic, msgtype):
        if msgtype not in self._schemas:
            self._schemas[msgtype] = self._writer.register_schema(
                name=msgtype,
                encoding="ros2msg",
                data=ROS2_MSG_SCHEMAS[msgtype].encode("utf-8"),
            )
        ch_id = self._writer.register_channel(
            topic=topic,
            message_encoding="cdr",
            schema_id=self._schemas[msgtype],
        )
        self._channels[ch_id] = {"name": topic, "type": msgtype, "count": 0}
        return ch_id

    def write(self, conn_id, ts_ns, raw_cdr_bytes):
        ts_int = int(ts_ns)
        self._writer.add_message(
            channel_id=conn_id,
            log_time=ts_int,
            data=bytes(raw_cdr_bytes),
            publish_time=ts_int,
        )
        self._channels[conn_id]["count"] += 1
        self._total_msgs += 1
        if self._min_ts is None or ts_int < self._min_ts:
            self._min_ts = ts_int
        if self._max_ts is None or ts_int > self._max_ts:
            self._max_ts = ts_int

    def close(self):
        if self._writer is not None:
            self._writer.finish()
            self._writer = None
        if self._fh is not None:
            self._fh.close()
            self._fh = None
        self._write_metadata_yaml()

    def _write_metadata_yaml(self):
        start_ns = self._min_ts if self._min_ts is not None else 0
        duration_ns = (self._max_ts - self._min_ts) if (self._min_ts is not None and self._max_ts is not None) else 0

        lines = [
            "rosbag2_bagfile_information:",
            "  version: 9",
            "  storage_identifier: mcap",
            "  duration:",
            f"    nanoseconds: {duration_ns}",
            "  starting_time:",
            f"    nanoseconds_since_epoch: {start_ns}",
            f"  message_count: {self._total_msgs}",
            "  topics_with_message_count:",
        ]
        for ch in self._channels.values():
            lines.extend([
                "    - topic_metadata:",
                f"        name: {ch['name']}",
                f"        type: {ch['type']}",
                "        serialization_format: cdr",
                '        offered_qos_profiles: []',
                '        type_description_hash: ""',
                f"      message_count: {ch['count']}",
            ])
        lines.extend([
            '  compression_format: ""',
            '  compression_mode: ""',
            "  relative_file_paths:",
            f"    - {self.mcap_filename}",
            "  files:",
            f"    - path: {self.mcap_filename}",
            "      starting_time:",
            f"        nanoseconds_since_epoch: {start_ns}",
            "      duration:",
            f"        nanoseconds: {duration_ns}",
            f"      message_count: {self._total_msgs}",
            "  custom_data: {}",
            "  ros_distro: jazzy",
        ])

        with open(os.path.join(self.bag_dir, "metadata.yaml"), "w") as f:
            f.write("\n".join(lines) + "\n")


def create_ros2_writer(bag_path, storage="mcap"):
    return McapRos2Writer(bag_path) if storage == "mcap" else Rosbag2Writer(bag_path)


def iter_video_frames(video_path, start_trim_sec, end_trim_sec, time_offset_ns, worker_id=0, keep_color=True, target_fps=None):
    """Yields (ts_ns, frame) tuples with drift-free native-FPS timing and optional downsampling."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        tqdm.write(f"[ERROR] Could not open video: {video_path}")
        return

    native_fps = cap.get(cv2.CAP_PROP_FPS)
    if native_fps <= 0:
        tqdm.write(f"[ERROR] Invalid FPS ({native_fps}) reported for {video_path}")
        cap.release()
        return

    total_vid_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if end_trim_sec == float('inf'):
        total_frames_to_read = total_vid_frames if total_vid_frames > 0 else None
    else:
        total_frames_to_read = int(end_trim_sec * native_fps)

    downsample = (target_fps is not None) and (target_fps < native_fps)
    if downsample:
        target_period = 1.0 / target_fps
        half_frame_tol = 0.5 / native_fps
        next_save_time = start_trim_sec

    frame_idx = 0
    saved_count = 0

    with tqdm(total=total_frames_to_read, desc=os.path.basename(video_path), position=worker_id, leave=True) as pbar:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            time_sec = frame_idx / native_fps
            if time_sec > end_trim_sec:
                break

            if start_trim_sec <= time_sec <= end_trim_sec:
                should_save = True
                if downsample:
                    if time_sec + half_frame_tol >= next_save_time:
                        steps = int((time_sec + half_frame_tol - next_save_time) // target_period) + 1
                        next_save_time += steps * target_period
                    else:
                        should_save = False

                if should_save:
                    ts_ns = BASE_EPOCH_NS + int((time_sec * 1e9) + time_offset_ns)
                    out_frame = frame if keep_color else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                    saved_count += 1
                    yield ts_ns, out_frame

            frame_idx += 1
            pbar.update(1)

    cap.release()
    mode_str = "Color (BGR)" if keep_color else "Grayscale"
    fps_str = f"{native_fps:.2f} -> {target_fps:.2f} fps" if downsample else f"native {native_fps:.2f} fps"
    tqdm.write(f"\n[SUCCESS] {os.path.basename(video_path)}: Processed {saved_count} {mode_str} frames ({fps_str})")


def iter_imu_telemetry_parser(video_path, start_trim_sec, end_trim_sec, time_offset_ns=0):
    """Yields (ts_ns, wx, wy, wz, ax, ay, az) tuples in SI units (rad/s, m/s^2) via telemetry-parser."""
    tqdm.write(f"[INFO] Parsing embedded IMU telemetry from {os.path.basename(video_path)}...")
    try:
        tp = telemetry_parser.Parser(video_path)
        cam_model = getattr(tp, "camera", "Unknown Camera")
        tqdm.write(f"[INFO] Detected {cam_model} — extracting normalized IMU stream...")
        samples = tp.normalized_imu()
    except Exception as e:
        tqdm.write(f"\n[FATAL ERROR] telemetry-parser failed on {video_path}: {e}")
        return

    written_count = 0
    for sample in samples:
        gyro = sample.get('gyro')
        accl = sample.get('accl')
        if gyro is None or accl is None:
            continue

        time_sec = float(sample['timestamp_ms']) / 1000.0
        if start_trim_sec <= time_sec <= end_trim_sec:
            ts_ns = BASE_EPOCH_NS + int((time_sec * 1e9) + time_offset_ns)
            written_count += 1
            yield (
                ts_ns,
                float(gyro[0]) * DEG2RAD, float(gyro[1]) * DEG2RAD, float(gyro[2]) * DEG2RAD,
                float(accl[0]), float(accl[1]), float(accl[2]),
            )

    if written_count == 0:
        tqdm.write(f"[WARNING] No valid IMU samples extracted from {video_path} within trim bounds.")
    else:
        tqdm.write(f"[SUCCESS] Embedded IMU extracted ({written_count} samples) from {os.path.basename(video_path)}")


def iter_imu_csv(input_csv, start_trim_sec, end_trim_sec, time_offset_ns=0):
    """Yields (ts_ns, wx, wy, wz, ax, ay, az) tuples from a Gyroflow CSV."""
    tqdm.write(f"[INFO] Reading external IMU CSV from {os.path.basename(input_csv)}...")

    with open(input_csv, 'r') as infile:
        reader = csv.DictReader(infile)
        fields = reader.fieldnames or []

        if 'timestamp_ms' in fields:
            ts_col, ts_scale = 'timestamp_ms', 1e3
        elif 'timestamp_us' in fields:
            ts_col, ts_scale = 'timestamp_us', 1e6
        else:
            tqdm.write(f"\n[FATAL ERROR] No timestamp column found in {input_csv}. Available: {fields}")
            return

        gyro_cols = ('org_gyro_x', 'org_gyro_y', 'org_gyro_z') if 'org_gyro_x' in fields else ('gyro_x', 'gyro_y', 'gyro_z')
        if 'org_acc_x' in fields:
            acc_cols = ('org_acc_x', 'org_acc_y', 'org_acc_z')
        elif 'acc_x' in fields:
            acc_cols = ('acc_x', 'acc_y', 'acc_z')
        else:
            acc_cols = ('accel_x', 'accel_y', 'accel_z')

        written_count = 0
        for row in reader:
            try:
                time_sec = float(row[ts_col]) / ts_scale
                if start_trim_sec <= time_sec <= end_trim_sec:
                    ts_ns = BASE_EPOCH_NS + int((time_sec * 1e9) + time_offset_ns)
                    written_count += 1
                    yield (
                        ts_ns,
                        float(row[gyro_cols[0]]) * DEG2RAD,
                        float(row[gyro_cols[1]]) * DEG2RAD,
                        float(row[gyro_cols[2]]) * DEG2RAD,
                        float(row[acc_cols[0]]),
                        float(row[acc_cols[1]]),
                        float(row[acc_cols[2]]),
                    )
            except KeyError as e:
                tqdm.write(f"\n[FATAL ERROR] IMU CSV column mismatch. Could not find {e} in {input_csv}.")
                tqdm.write(f"Available columns: {fields}")
                return

    tqdm.write(f"[SUCCESS] External IMU read ({written_count} rows) from {os.path.basename(input_csv)}")

# MODE 1: DISK-BACKED PNG/CSV WORKERS (Default / Resumable)

def extract_frames_to_disk(video_path, output_dir, start_trim_sec, end_trim_sec, time_offset_ns, worker_id=0, keep_color=True, target_fps=None):
    os.makedirs(output_dir, exist_ok=True)
    for ts_ns, frame in iter_video_frames(video_path, start_trim_sec, end_trim_sec, time_offset_ns, worker_id, keep_color, target_fps):
        cv2.imwrite(os.path.join(output_dir, f"{ts_ns}.png"), frame)


def write_imu_to_disk(imu_iter, output_csv):
    with open(output_csv, 'w', newline='') as outfile:
        writer = csv.writer(outfile)
        writer.writerow(['timestamp', 'omega_x', 'omega_y', 'omega_z', 'alpha_x', 'alpha_y', 'alpha_z'])
        for row in imu_iter:
            writer.writerow(row)


def extract_imu_tp_to_disk(video_path, output_csv, start_trim_sec, end_trim_sec, time_offset_ns=0):
    write_imu_to_disk(iter_imu_telemetry_parser(video_path, start_trim_sec, end_trim_sec, time_offset_ns), output_csv)


def format_imu_csv_to_disk(input_csv, output_csv, start_trim_sec, end_trim_sec, time_offset_ns=0):
    write_imu_to_disk(iter_imu_csv(input_csv, start_trim_sec, end_trim_sec, time_offset_ns), output_csv)


def run_kalibr_bag_creator(dataset_dir, output_bag_path):
    """Runs Kalibr's bag creator to generate a ROS 1 .bag (mono8) from dataset-dir."""
    tqdm.write("\n[INFO] Generating ROS 1 Bag for Kalibr (mono8)...")
    local_cmd = (
        "source /opt/ros/noetic/setup.bash && "
        "source /catkin_ws/devel/setup.bash && "
        f"rosrun kalibr kalibr_bagcreater --folder {dataset_dir}/. --output-bag {output_bag_path}"
    )
    result = subprocess.run(local_cmd, shell=True, executable='/bin/bash')
    if result.returncode == 0:
        tqdm.write(f"[SUCCESS] ROS 1 bag created: {os.path.basename(output_bag_path)}")
        return True
    tqdm.write("[ERROR] ROS 1 bag creation failed. Check the logs above.")
    return False


def write_ros2_bag_from_disk(dataset_dir, ros2_full_path, storage="mcap"):
    """Writes full-color (bgr8) images and/or IMU data from dataset-dir into a ROS 2 bag."""
    tqdm.write(f"\n[INFO] Generating ROS 2 bag ({os.path.basename(ros2_full_path)}/ | storage={storage})...")
    if os.path.exists(ros2_full_path):
        shutil.rmtree(ros2_full_path)

    try:
        with create_ros2_writer(ros2_full_path, storage=storage) as writer:
            for cam_name in ["cam0", "cam1"]:
                cam_dir = os.path.join(dataset_dir, cam_name)
                if not os.path.isdir(cam_dir):
                    continue
                png_files = sorted(f for f in os.listdir(cam_dir) if f.endswith(".png"))
                if not png_files:
                    continue

                topic = f"/{cam_name}/image_raw"
                conn = writer.add_connection(topic, "sensor_msgs/msg/Image")
                for fname in tqdm(png_files, desc=f"ROS2 {topic} (bgr8)", leave=True):
                    ts_ns = int(fname[:-4])
                    img = cv2.imread(os.path.join(cam_dir, fname), cv2.IMREAD_COLOR)
                    height, width, channels = img.shape
                    stamp = Time(sec=int(ts_ns // 1_000_000_000), nanosec=int(ts_ns % 1_000_000_000))
                    msg = Image(
                        header=Header(stamp=stamp, frame_id=cam_name),
                        height=height,
                        width=width,
                        encoding="bgr8",
                        is_bigendian=0,
                        step=width * channels,
                        data=img.ravel(),
                    )
                    writer.write(conn, ts_ns, serialize_cdr(msg, "sensor_msgs/msg/Image"))

            for imu_name in ["imu0", "imu1"]:
                imu_csv = os.path.join(dataset_dir, f"{imu_name}.csv")
                if not os.path.isfile(imu_csv):
                    continue
                topic = f"/{imu_name}"
                conn = writer.add_connection(topic, "sensor_msgs/msg/Imu")
                with open(imu_csv, "r") as f:
                    for row in csv.DictReader(f):
                        ts_ns = int(row["timestamp"])
                        stamp = Time(sec=int(ts_ns // 1_000_000_000), nanosec=int(ts_ns % 1_000_000_000))
                        msg = Imu(
                            header=Header(stamp=stamp, frame_id=imu_name),
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                            orientation_covariance=np.array([-1.0] + [0.0] * 8, dtype=np.float64),
                            angular_velocity=Vector3(
                                x=float(row["omega_x"]),
                                y=float(row["omega_y"]),
                                z=float(row["omega_z"]),
                            ),
                            angular_velocity_covariance=np.zeros(9, dtype=np.float64),
                            linear_acceleration=Vector3(
                                x=float(row["alpha_x"]),
                                y=float(row["alpha_y"]),
                                z=float(row["alpha_z"]),
                            ),
                            linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
                        )
                        writer.write(conn, ts_ns, serialize_cdr(msg, "sensor_msgs/msg/Imu"))

        tqdm.write(f"[SUCCESS] ROS 2 bag created ({storage}): {os.path.basename(ros2_full_path)}/")
        return True
    except Exception as e:
        tqdm.write(f"[ERROR] ROS 2 bag creation failed: {e}")
        return False

# MODE 2: DIRECT STREAMING WITH BOUNDED QUEUE (Zero Temp PNGs)

def stream_frames_worker(cam_name, video_path, start_trim_sec, end_trim_sec, time_offset_ns, worker_id, keep_color, target_fps, out_queue):
    """Producer: Decodes video frames and pushes into bounded queue (blocks if consumer is busy)."""
    try:
        for ts_ns, frame in iter_video_frames(video_path, start_trim_sec, end_trim_sec, time_offset_ns, worker_id, keep_color, target_fps):
            out_queue.put(("FRAME", cam_name, ts_ns, frame))
    finally:
        out_queue.put(("DONE", cam_name))


def stream_imu_worker(imu_name, use_tp, source_path, start_trim_sec, end_trim_sec, time_offset_ns, out_queue):
    """Producer: Extracts IMU samples and pushes compact batches into bounded queue."""
    try:
        imu_iter = (
            iter_imu_telemetry_parser(source_path, start_trim_sec, end_trim_sec, time_offset_ns)
            if use_tp
            else iter_imu_csv(source_path, start_trim_sec, end_trim_sec, time_offset_ns)
        )
        batch = []
        for row in imu_iter:
            batch.append(row)
            if len(batch) >= IMU_BATCH_SIZE:
                out_queue.put(("IMU_BATCH", imu_name, batch))
                batch = []
        if batch:
            out_queue.put(("IMU_BATCH", imu_name, batch))
    finally:
        out_queue.put(("DONE", imu_name))


class DirectBagConsumer:
    """Single-writer consumer that drains the bounded queue straight into ROS 1 and/or ROS 2 bags."""

    def __init__(self, fmt, ros1_bag_path, ros2_bag_path, ros2_storage="mcap"):
        self.write_ros1 = fmt in ('ros1', 'both')
        self.write_ros2 = fmt in ('ros2', 'both')
        self.ros1_bag_path = ros1_bag_path
        self.ros2_bag_path = ros2_bag_path
        self.ros2_storage = ros2_storage

        self.ros1_bag = None
        self.ros2_writer = None
        self.ros2_conns = {}

    def __enter__(self):
        if self.write_ros1:
            if os.path.exists(self.ros1_bag_path):
                os.remove(self.ros1_bag_path)
            self.ros1_bag = rosbag.Bag(self.ros1_bag_path, 'w')

        if self.write_ros2:
            if os.path.exists(self.ros2_bag_path):
                shutil.rmtree(self.ros2_bag_path)
            self.ros2_writer = create_ros2_writer(self.ros2_bag_path, storage=self.ros2_storage)
            self.ros2_writer.open()

        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.ros1_bag is not None:
            self.ros1_bag.close()
        if self.ros2_writer is not None:
            self.ros2_writer.close()

    def write_frame(self, cam_name, ts_ns, frame):
        topic = f"/{cam_name}/image_raw"
        sec = int(ts_ns // 1_000_000_000)
        nsec = int(ts_ns % 1_000_000_000)

        if self.write_ros1:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            h, w = gray.shape
            stamp = rospy.Time(secs=sec, nsecs=nsec)
            msg = Ros1Image()
            msg.header.stamp = stamp
            msg.header.frame_id = cam_name
            msg.height = h
            msg.width = w
            msg.encoding = "mono8"
            msg.is_bigendian = 0
            msg.step = w
            msg.data = gray.tobytes()
            self.ros1_bag.write(topic, msg, t=stamp)

        if self.write_ros2:
            if topic not in self.ros2_conns:
                self.ros2_conns[topic] = self.ros2_writer.add_connection(topic, "sensor_msgs/msg/Image")
            h, w, c = frame.shape
            msg = Image(
                header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=cam_name),
                height=h,
                width=w,
                encoding="bgr8",
                is_bigendian=0,
                step=w * c,
                data=frame.ravel(),
            )
            self.ros2_writer.write(self.ros2_conns[topic], ts_ns, serialize_cdr(msg, "sensor_msgs/msg/Image"))

    def write_imu_batch(self, imu_name, rows):
        topic = f"/{imu_name}"
        if self.write_ros2 and topic not in self.ros2_conns:
            self.ros2_conns[topic] = self.ros2_writer.add_connection(topic, "sensor_msgs/msg/Imu")

        for ts_ns, wx, wy, wz, ax, ay, az in rows:
            sec = int(ts_ns // 1_000_000_000)
            nsec = int(ts_ns % 1_000_000_000)

            if self.write_ros1:
                stamp = rospy.Time(secs=sec, nsecs=nsec)
                msg = Ros1Imu()
                msg.header.stamp = stamp
                msg.header.frame_id = imu_name
                msg.orientation.w = 1.0
                msg.orientation_covariance[0] = -1.0
                msg.angular_velocity.x = wx
                msg.angular_velocity.y = wy
                msg.angular_velocity.z = wz
                msg.linear_acceleration.x = ax
                msg.linear_acceleration.y = ay
                msg.linear_acceleration.z = az
                self.ros1_bag.write(topic, msg, t=stamp)

            if self.write_ros2:
                msg = Imu(
                    header=Header(stamp=Time(sec=sec, nanosec=nsec), frame_id=imu_name),
                    orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
                    orientation_covariance=np.array([-1.0] + [0.0] * 8, dtype=np.float64),
                    angular_velocity=Vector3(x=wx, y=wy, z=wz),
                    angular_velocity_covariance=np.zeros(9, dtype=np.float64),
                    linear_acceleration=Vector3(x=ax, y=ay, z=az),
                    linear_acceleration_covariance=np.zeros(9, dtype=np.float64),
                )
                self.ros2_writer.write(self.ros2_conns[topic], ts_ns, serialize_cdr(msg, "sensor_msgs/msg/Imu"))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="DEEP-UNDER ROV: Mono/Stereo + Telemetry-Parser/CSV + Direct Streaming Pipeline")
    parser.add_argument('--cam0', default=None, help="(Optional) Path to Master Video (e.g. GoPro)")
    parser.add_argument('--cam1', default=None, help="(Optional) Path to Secondary Video (e.g. DJI)")
    parser.add_argument('--tp_cam0', action='store_true', help="Enable embedded IMU extraction from --cam0 via telemetry-parser")
    parser.add_argument('--tp_cam1', action='store_true', help="Enable embedded IMU extraction from --cam1 via telemetry-parser")
    parser.add_argument('--imu0', default=None, help="(Optional) Path to external Master IMU CSV (when --tp_cam0 is off)")
    parser.add_argument('--imu1', default=None, help="(Optional) Path to external Secondary IMU CSV (when --tp_cam1 is off)")
    parser.add_argument('--no_video', action='store_true', help="Skip video frame extraction (useful with --tp_cam0 for IMU-only Allan Variance)")
    parser.add_argument('--offset_sec', type=float, default=None, help="Time offset in seconds (Required if --cam1, --imu1, or --tp_cam1 is used)")
    parser.add_argument('--start_trim', type=float, default=0.0, help="(Optional) Start trim time in seconds (default: 0.0)")
    parser.add_argument('--end_trim', type=float, default=None, help="(Optional) End trim time in seconds (default: end of file)")
    parser.add_argument('--fps', type=float, default=None, help="(Optional) Target output FPS for video downsampling (default: native FPS)")
    parser.add_argument('--output_bag', default="calib_data", help="Output base name or .bag filename")
    parser.add_argument('--format', choices=['ros1', 'ros2', 'both'], default='ros1', help="ros1 (Kalibr mono8), ros2 (color bgr8), or both")
    parser.add_argument('--storage', choices=['mcap', 'sqlite3'], default='mcap', help="ROS 2 bag storage format: mcap (default, LZ4-compressed) or sqlite3 (.db3)")
    parser.add_argument('--direct', action='store_true', help="Stream frames/IMU directly into bags via a bounded RAM queue (skips temporary PNGs)")
    parser.add_argument('--queue_size', type=int, default=32, help="Max buffered frames/batches in RAM for --direct mode (default: 32)")

    args = parser.parse_args()

    if args.tp_cam0 and args.imu0:
        parser.error("Cannot use both --tp_cam0 and --imu0 together. Choose either embedded telemetry-parser or an external CSV.")
    if args.tp_cam1 and args.imu1:
        parser.error("Cannot use both --tp_cam1 and --imu1 together. Choose either embedded telemetry-parser or an external CSV.")
    if args.tp_cam0 and not args.cam0:
        parser.error("--tp_cam0 requires --cam0 to be specified.")
    if args.tp_cam1 and not args.cam1:
        parser.error("--tp_cam1 requires --cam1 to be specified.")

    has_cam0 = bool(args.cam0 and not args.no_video)
    has_cam1 = bool(args.cam1 and not args.no_video)
    has_imu0 = bool(args.imu0 or args.tp_cam0)
    has_imu1 = bool(args.imu1 or args.tp_cam1)

    if not (has_cam0 or has_cam1 or has_imu0 or has_imu1):
        parser.error("Nothing to extract. Provide --cam0, --imu0, or --cam0 with --tp_cam0.")

    if (args.cam1 or args.imu1 or args.tp_cam1) and args.offset_sec is None:
        parser.error("--offset_sec is required whenever --cam1, --imu1, or --tp_cam1 is provided.")

    if args.fps is not None and args.fps <= 0:
        parser.error("--fps must be a positive number.")

    if args.queue_size < 1:
        parser.error("--queue_size must be at least 1.")

    end_trim = args.end_trim if args.end_trim is not None else float('inf')

    workspace_dir = os.path.abspath(os.getcwd())
    base_name = args.output_bag[:-4] if args.output_bag.endswith('.bag') else args.output_bag
    ros1_bag_name = f"{base_name}.bag"
    ros2_bag_dir = f"{base_name}_ros2"
    ros1_bag_path = os.path.join(workspace_dir, ros1_bag_name)
    ros2_bag_path = os.path.join(workspace_dir, ros2_bag_dir)

    if not has_cam0 and not has_cam1:
        mode = "IMU-Only (Allan Variance)"
    elif has_cam1:
        mode = "Stereo (Dual-Camera)"
    else:
        mode = "Monocular (Single-Camera)"

    imu0_src = "telemetry-parser (cam0)" if args.tp_cam0 else (os.path.basename(args.imu0) if args.imu0 else "OFF")
    imu1_src = "telemetry-parser (cam1)" if args.tp_cam1 else (os.path.basename(args.imu1) if args.imu1 else "OFF")
    end_label = f"{end_trim}s" if end_trim != float('inf') else "EOF"
    fps_label = f"{args.fps} Hz" if args.fps is not None else "Native"
    io_label = f"DIRECT STREAM (queue_size={args.queue_size})" if args.direct else "DISK PNG CACHE"
    fmt_label = f"{args.format.upper()} ({args.storage.upper()})" if args.format in ('ros2', 'both') else args.format.upper()

    print(f"\n=== Video and IMU to Rosbag Converter [{mode} | Format: {fmt_label} | {io_label}] ===")
    print(f"Video FPS: {fps_label} | IMU0: {imu0_src} | IMU1: {imu1_src}")
    print(f"Master Trims: {args.start_trim}s to {end_label}")

    if args.cam1 or args.imu1 or args.tp_cam1:
        offset_ns = int(args.offset_sec * 1e9)
        sec_start_trim = args.start_trim - args.offset_sec
        sec_end_trim = (end_trim - args.offset_sec) if end_trim != float('inf') else float('inf')
        sec_end_label = f"{sec_end_trim:.3f}s" if sec_end_trim != float('inf') else "EOF"
        print(f"Secondary Trims: {sec_start_trim:.3f}s to {sec_end_label}")

    print("---------------------------------------------\n")

    keep_color = args.format in ('ros2', 'both')
    processes = []

    if args.direct:
        frame_queue = Queue(maxsize=args.queue_size)

        if has_cam0:
            processes.append(Process(
                target=stream_frames_worker,
                args=("cam0", args.cam0, args.start_trim, end_trim, 0, 0, keep_color, args.fps, frame_queue),
            ))
        if has_cam1:
            processes.append(Process(
                target=stream_frames_worker,
                args=("cam1", args.cam1, sec_start_trim, sec_end_trim, offset_ns, 1, keep_color, args.fps, frame_queue),
            ))
        if has_imu0:
            src0 = args.cam0 if args.tp_cam0 else args.imu0
            processes.append(Process(
                target=stream_imu_worker,
                args=("imu0", args.tp_cam0, src0, args.start_trim, end_trim, 0, frame_queue),
            ))
        if has_imu1:
            src1 = args.cam1 if args.tp_cam1 else args.imu1
            processes.append(Process(
                target=stream_imu_worker,
                args=("imu1", args.tp_cam1, src1, sec_start_trim, sec_end_trim, offset_ns, frame_queue),
            ))

        for p in processes:
            p.start()

        active_producers = len(processes)
        finished_producers = 0
        writer_pos = (1 if has_cam0 else 0) + (1 if has_cam1 else 0)

        try:
            with DirectBagConsumer(args.format, ros1_bag_path, ros2_bag_path, ros2_storage=args.storage) as consumer, \
                 tqdm(desc="Bag Writer", position=writer_pos, unit="msg", leave=True) as wbar:
                while finished_producers < active_producers:
                    item = frame_queue.get()
                    kind = item[0]
                    if kind == "DONE":
                        finished_producers += 1
                    elif kind == "FRAME":
                        _, cam_name, ts_ns, frame = item
                        consumer.write_frame(cam_name, ts_ns, frame)
                        wbar.update(1)
                    elif kind == "IMU_BATCH":
                        _, imu_name, rows = item
                        consumer.write_imu_batch(imu_name, rows)
                        wbar.update(len(rows))
        except BaseException as e:
            if isinstance(e, KeyboardInterrupt):
                tqdm.write("\n[WARNING] Interrupted by user — shutting down workers...")
            else:
                tqdm.write(f"\n[ERROR] Bag writer aborted ({e}) — terminating workers...")
            for p in processes:
                if p.is_alive():
                    p.terminate()
            if isinstance(e, KeyboardInterrupt):
                sys.exit(1)
            raise
        finally:
            for p in processes:
                p.join()

        print(f"\n[PIPELINE COMPLETE] Direct-streamed output ready in {workspace_dir}")

    else:
        dataset_dir = os.path.join(workspace_dir, "dataset-dir")
        if os.path.exists(dataset_dir):
            print("[INFO] Cleaning up old dataset-dir...")
            shutil.rmtree(dataset_dir)
        os.makedirs(dataset_dir, exist_ok=True)

        if has_cam0:
            processes.append(Process(
                target=extract_frames_to_disk,
                args=(args.cam0, os.path.join(dataset_dir, "cam0"), args.start_trim, end_trim, 0, 0, keep_color, args.fps),
            ))
        if has_cam1:
            processes.append(Process(
                target=extract_frames_to_disk,
                args=(args.cam1, os.path.join(dataset_dir, "cam1"), sec_start_trim, sec_end_trim, offset_ns, 1, keep_color, args.fps),
            ))
        if has_imu0:
            imu0_out = os.path.join(dataset_dir, "imu0.csv")
            if args.tp_cam0:
                processes.append(Process(target=extract_imu_tp_to_disk, args=(args.cam0, imu0_out, args.start_trim, end_trim, 0)))
            else:
                processes.append(Process(target=format_imu_csv_to_disk, args=(args.imu0, imu0_out, args.start_trim, end_trim, 0)))
        if has_imu1:
            imu1_out = os.path.join(dataset_dir, "imu1.csv")
            if args.tp_cam1:
                processes.append(Process(target=extract_imu_tp_to_disk, args=(args.cam1, imu1_out, sec_start_trim, sec_end_trim, offset_ns)))
            else:
                processes.append(Process(target=format_imu_csv_to_disk, args=(args.imu1, imu1_out, sec_start_trim, sec_end_trim, offset_ns)))

        for p in processes:
            p.start()
        for p in processes:
            p.join()

        ros1_ok = True
        ros2_ok = True

        if args.format in ('ros1', 'both'):
            ros1_ok = run_kalibr_bag_creator(dataset_dir, ros1_bag_path)

        if args.format in ('ros2', 'both'):
            ros2_ok = write_ros2_bag_from_disk(dataset_dir, ros2_bag_path, storage=args.storage)

        if ros1_ok and ros2_ok:
            print(f"\n[PIPELINE COMPLETE] Output ready in {workspace_dir}")
            if os.path.exists(dataset_dir):
                print("[INFO] Emptying the trash to save hard drive space...")
                shutil.rmtree(dataset_dir)
                print("[SUCCESS] Temporary 'dataset-dir' deleted.")