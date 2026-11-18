# Enabling Windows Capable Solution

*Analysis date: 2026-11-18. Scope: what it takes to run LIO-SAM Studio (the survey processing and recording application in this repository) as a Windows application, and the level of effort.*

## Summary

LIO-SAM Studio currently runs on Ubuntu with ROS 2 Jazzy. There are three realistic ways to offer it on Windows:

| Route | What the user gets | Effort | Risk |
|---|---|---|---|
| **A. WSL2 bundle** (recommended first step) | Windows installer and Start-menu icon. The existing Linux build runs hidden inside WSL2, and its windows appear on the Windows desktop. | **~2–3 weeks** | Low–medium |
| **B. Native Windows build** | A true Windows install with no Linux component | **~8–12 weeks**, plus ongoing maintenance | High |
| **C. ROS-free offline processor** | A single Windows program that processes bag files directly | **3+ months** | Medium; best long-term product |

**Recommendation:** deliver Route A first, because it reuses the build that has already been validated on real survey data. Take on Route B only if WSL is not acceptable (for example, a customer's IT policy blocks virtualization). Consider Route C if Windows becomes a long-term product line.

## What the application depends on

Any Windows version has to supply all of the following.

**SLAM core (C++, `src/`)**
- ROS 2 client libraries: `rclcpp`, `tf2` (+ `tf2_ros`, `tf2_eigen`, `tf2_geometry_msgs`, `tf2_sensor_msgs`), and generated code for the custom `cloud_info` message and `save_map` service
- **GTSAM 4.2**, including `gtsam_unstable` (factor-graph optimization)
- **PCL** (point clouds), **OpenCV**, **Eigen**
- `cv_bridge`, `pcl_conversions`

**ROS runtime tools the application launches**
- `ros2 launch`, `ros2 bag play` (rosbag2 with SQLite storage)
- **RViz2** (Qt-based 3D viewer)
- `robot_state_publisher`, `xacro`, `static_transform_publisher`
- DDS middleware (ROS 2 messaging)

**Python side (GUI and processing scripts)**
- Tkinter (GUI), numpy, scipy
- `rosbags` (ROS 1 → ROS 2 bag conversion), `laspy` + `lazrs` (LAS/LAZ export), `pyproj` plus PROJ data files (georeferencing)
- The `pcl_pcd2ply` command-line tool (PLY export)

## Route A: WSL2 bundle (recommended)

**How it works.** Export the working Ubuntu environment as a WSL distribution image (~3–5 GB). A Windows installer imports that image and creates a Start-menu shortcut that runs `wsl -d LIO-SAM-Studio …/lio_sam_studio.sh`. WSLg, which is built into current Windows, displays the Tk windows and RViz on the Windows desktop.

**Work required**
- Remove hard-coded Linux paths: the workspace path in `lio_sam_gui.py`, `BAGS_DIR`, and the paths in the shell scripts.
- Map Windows drives so users can pick bag files from `C:\` or `D:\`.
- Write the installer (Inno Setup or WiX), including a check that WSL is installed and an uninstaller.
- Test on clean Windows 10 and Windows 11 machines.

**Limitations**
- Requires **Windows 10 21H2 or later, or Windows 11**, with hardware virtualization enabled. Some corporate IT environments lock this down.
- **Reading a 20 GB bag from a Windows drive through WSL is much slower** than reading it from inside WSL's own disk. Either copy bags into WSL first, or accept slower conversion and playback.
- **RViz runs through WSLg's GPU translation layer.** It works, but expect lower performance than on native Linux.
- **Live recording from the Ouster** requires WSL's "mirrored" networking mode (Windows 11) so the sensor's network traffic reaches WSL.

## Route B: native Windows build

ROS 2 Jazzy officially supports Windows 10 (Visual Studio 2019), but this application depends on some of the hardest pieces to build there.

| # | Work item | Estimate | Notes |
|---|---|---|---|
| 1 | Build **GTSAM 4.2** with MSVC | 1–2 weeks | **Biggest risk.** Expect friction with Boost and TBB, and with exporting `gtsam_unstable` symbols from DLLs. |
| 2 | **PCL, OpenCV, `pcl_conversions`, `cv_bridge`** | 1–2 weeks | PCL via vcpkg is heavy but workable. `pcl_conversions`, and possibly `cv_bridge`, are probably not in the Windows ROS distribution and would need building from source. |
| 3 | **LIO-SAM C++ portability fixes** (`src/mapOptmization.cpp`) | ~1 week | Replace `getenv("HOME")` with `USERPROFILE`; replace `system("rm -rf …")` / `mkdir -p` with `std::filesystem`; expect MSVC issues with point-type memory alignment and OpenMP. |
| 4 | **GUI process handling** | ~1 week | Every launch currently runs `source …setup.bash` in bash and uses Unix-only process control (`setsid`, `killpg`, `pkill`). These need Windows environment setup and Windows process control. |
| 5 | **Bundling and installer** | 1–2 weeks | Collect the ROS install tree, all DLLs, Qt, Python and PROJ data (~1.5–3 GB installed), plus the MSVC runtime and an installer. **Code signing** costs ~$200–500/year for a certificate; without it, Windows SmartScreen warns users. |
| 6 | **Testing on clean machines** | ~1 week | Includes Windows Firewall prompts for DDS network traffic. |
| | **Total** | **~8–12 weeks** | Plus maintenance with each ROS or GTSAM update. |

## Route C: ROS-free offline processor

Processing a recorded bag does not actually need live ROS messaging. LIO-SAM's four programs (IMU preintegration, image projection, feature extraction, map optimization) currently talk to each other over ROS topics. They could be combined into a single offline program that reads the bag directly and writes the map, with RViz replaced by an export to a viewer such as CloudCompare.

That removes ROS, DDS and Qt from the Windows build entirely, leaving GTSAM, PCL, OpenCV and Eigen. The result is one program that is far easier to maintain on Windows. It is a significant refactor of the SLAM code, so treat it as a product decision rather than a porting task.

## Issues that apply to every route

- **Live recording has no Windows sensor driver.** The Recorder only records topics published by an Ouster ROS driver, and `ouster-ros` is Linux-only. On Windows, recording would use Ouster's own SDK (which supports Windows and records PCAP files), plus a PCAP → bag converter. That adds **~1–2 weeks**. The alternative is to keep recording on a Linux device and offer processing only on Windows.
- **Licensing allows distribution.** LIO-SAM, GTSAM and PCL are BSD-licensed, ROS 2 is Apache 2.0, and Qt is LGPL, which permits bundling when dynamically linked, as RViz is. The installer should include a third-party license notice.

## Recommended next steps

1. **Route A prototype (~2–3 weeks):** remove hard-coded paths, export the WSL image, build the installer, and test on clean Windows 10 and 11 machines.
2. **Decide on recording:** either Windows processing only (recording stays on Linux), or add Ouster SDK PCAP recording.
3. **Revisit Route B or C** only if WSL is rejected by target users, or if Windows becomes a long-term product line.
