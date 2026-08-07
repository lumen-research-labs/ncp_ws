# Robot container

Built and run **on the robot**, never on the workstation. TensorRT artifacts
are device-local; cross-compiling them is not supported and not attempted.

Board: Jetson Orin Nano Super. ROS2 Humble.

## Pinning the base image

```bash
# on the robot
cat /etc/nv_tegra_release
```

Pin `Dockerfile.jetson` to that exact L4T release. A mismatch between the
container's CUDA/TensorRT and the host driver fails at runtime, not build time.

## Build and run

```bash
cd ~/ncp_ws/docker
docker compose build
docker compose up -d
docker compose exec robot bash
```

`--network host` is required for DDS discovery — inside the container,
`ros2 topic list` must match what the host sees. Device passthrough covers the
lidar, camera, IMU and the STM32 serial link.
