FROM kalibr

USER root

RUN apt-get update && apt-get install -y \
    python3-pip \
    libgl1-mesa-glx \
    libglib2.0-0 \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install opencv-python tqdm "rosbags<0.10" telemetry-parser mcap lz4

COPY stereo_cam_imu_to_rosbag.py /opt/stereo_cam_imu_to_rosbag.py
RUN chmod +x /opt/stereo_cam_imu_to_rosbag.py

WORKDIR /data

ENTRYPOINT ["python3", "/opt/stereo_cam_imu_to_rosbag.py"]
