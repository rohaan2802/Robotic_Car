<div align="center">

# Robotic Car

**Wi-Fi teleoperation and ultrasonic occupancy mapping on an ESP8266**

[![Firmware](https://img.shields.io/badge/ESP8266-NodeMCU-156a77?style=for-the-badge)](firmware/NodeMCU_Car/NodeMCU_Car.ino)
[![Python](https://img.shields.io/badge/Python-occupancy_grid-245375?style=for-the-badge)](robotic_car/)
[![Report](https://img.shields.io/badge/Project-report-4d567b?style=for-the-badge)](docs/project_report.pdf)
[![Demo](https://img.shields.io/badge/Hardware-demo-9b581d?style=for-the-badge)](media/hardware_demo.mp4)
[![Tests](https://img.shields.io/badge/Tests-unittest-2b644e?style=for-the-badge)](tests/)

[![Desktop GUI](https://img.shields.io/badge/Desktop_GUI-Tkinter-5E4A78?style=for-the-badge&logo=python&logoColor=white)](robotic_car/workbench.py)
[![Mapping](https://img.shields.io/badge/Mapping-NumPy-265A82?style=for-the-badge&logo=numpy&logoColor=white)](robotic_car/mapping.py)
[![Offline demo](https://img.shields.io/badge/Demo-Offline-96611B?style=for-the-badge&logo=plotly&logoColor=white)](#mapping-workbench)
[![Controls](https://img.shields.io/badge/Controls-Arduino-166A70?style=for-the-badge&logo=arduino&logoColor=white)](docs/hardware_setup.md)

</div>

## Live Links

| What | Link |
| --- | --- |
| Repository | [https://github.com/rohaan2802/Robotic_Car](https://github.com/rohaan2802/Robotic_Car) |
| GitHub Pages | [https://rohaan2802.github.io/Robotic_Car/](https://rohaan2802.github.io/Robotic_Car/) |
| Project report | [https://github.com/rohaan2802/Robotic_Car/blob/main/docs/project_report.pdf](https://github.com/rohaan2802/Robotic_Car/blob/main/docs/project_report.pdf) |

## Table of Contents

[Overview](#overview) · [What's Inside](#whats-inside) · [Project Breakdown](#project-breakdown) · [Folder Structure](#folder-structure) · [Setup](#setup) · [Mapping Workbench](#mapping-workbench) · [Results](#results) · [Limitations](#limitations) · [Figure Index](#figure-index) · [About the Author](#about-the-author)

## Overview

Robotic Car is a NodeMCU-powered teleoperation and occupancy-mapping prototype. A phone or laptop connects to the car’s ESP8266 access point. The firmware controls the L298N motor driver and exposes ultrasonic range and MPU6050 orientation over HTTP. The Python tools inspect the saved map, demonstrate the mapping algorithm offline, and query or send short drive pulses to the board.

The Fall 2024 FAST-NUCES CS-3015 group project was titled *Nano Navigators: Mapping with Precision*. The archive retains the source associated with the original GitHub submission and the report preserves the group’s implementation and design material.

## What's Inside

| Material | Contents |
| --- | --- |
| [Firmware sketch](firmware/NodeMCU_Car/NodeMCU_Car.ino) | Drive control and onboard web interface |
| [Desktop workbench](robotic_car/workbench.py) | Offline grid viewer, manual range input, and NumPy export |
| [Python tools](robotic_car/) | HTTP client, occupancy grid and offline review |
| [Saved occupancy grid](data/saved_occupancy.npy) | Original 500 × 500 probability array |
| [Chassis measurements](data/car_measurements.xlsx) | Original dimensions worksheet |
| [Project report](docs/project_report.pdf) | Group report and alternate TCP implementation |
| [Assignment brief](docs/assignment_brief.pdf) | Original course requirements |
| [Hardware video](media/hardware_demo.mp4) | Original recording |
| [Prototype archive](archive/README.md) | Original HTTP sketch and Python mapper |

## Project Breakdown

| Subsystem | Input | Output |
| --- | --- | --- |
| Drive control | F, B, L, R, RL, RR, S | ESP8266 translates commands into L298N pin states and caps requested motion at 1 second. |
| Range | Forward ultrasonic echo | Measured centimeters or −1 when no echo arrives. |
| Orientation | MPU6050 DMP packet | Yaw, pitch and roll in degrees. Missing or stale samples return HTTP 503. |
| Mapping | Pose (x, y, heading) and a range ≤100 cm | Mark preceding cells free and the measured cell occupied with bounded Bayesian evidence. |
| Review | Saved .npy grid or eight synthetic observations | Inspectable map plot with classified cell counts. |

Each valid observation starts at the externally supplied pose. The mapper samples the ray at half-cell intervals, updates each traversed cell once, excludes the hit cell from the free evidence, and then applies occupied evidence to that endpoint. Missing, non-finite, non-positive or over-range measurements leave the grid unchanged. The 0.35 and 0.65 classification thresholds make free, unknown and occupied cells visible. The historical map retains its original values.

The maintained sketch starts with the outputs stopped, validates all drive requests, and polls the motion expiry before and after handling HTTP. The onboard controller sends one short pulse per tap and exposes an explicit Stop. The Python client checks the firmware protocol identifier before issuing motion.

## Folder Structure

```text
Robotic_Car/
├── launch_workbench.cmd    Windows desktop launcher
├── firmware/NodeMCU_Car/   ESP8266 sketch and onboard controls
├── robotic_car/            Python client, mapper and command line
├── tests/                  Mapping and client regression checks
├── data/                   Saved occupancy grid and measurements
├── docs/                   Course report, brief and hardware setup
├── media/                  Original hardware demonstration
├── archive/ap_prototype/   Original HTTP sketch and mapper
├── assets/                 Site styling, scripts and 24 figures
└── index.html              GitHub Pages entry point
```

## Setup

Use Python 3.10 or newer. From the repository root in PowerShell:

```powershell
python -m venv .venv
.venv\Scripts\python.exe -m pip install -r requirements.txt
.venv\Scripts\python.exe -m unittest discover -s tests -v
.venv\Scripts\python.exe -m robotic_car review
.venv\Scripts\python.exe -m robotic_car demo
```

Append `--image path.png` to either map command for a headless plot. The demo mode can also export a NumPy array with `--save-map path.npy`. Open [index.html](index.html) to preview the Pages site.

For the car, follow the [hardware setup and HTTP API](docs/hardware_setup.md). After flashing and joining the SoftAP, open the car controller at `http://192.168.4.1/` or run:

```powershell
.venv\Scripts\python.exe -m robotic_car telemetry
.venv\Scripts\python.exe -m robotic_car pulse F --ms 250
.venv\Scripts\python.exe -m robotic_car pulse S
```

To add one live range reading at a known pose without commanding motion, run:

```powershell
.venv\Scripts\python.exe -m robotic_car scan --pose 0 0 90 --save-map scan.npy
```

Pass `--map scan.npy` to extend a previous scan. The pose is supplied as x and y in centimeters and heading in degrees counterclockwise from +x. View the resulting array with `python -m robotic_car review --map scan.npy`.

## Mapping Workbench

Run `python -m robotic_car.workbench` from an environment with the dependencies installed, or open [launch_workbench.cmd](launch_workbench.cmd) on Windows. The workbench starts with the synthetic map, so the first demonstration works without hardware or Wi-Fi.

1. Open **Original saved map** to inspect the original 500 × 500 probability grid. Use **Open .npy** for another square probability array.
2. Select **Blank grid** or **Synthetic demo**. Enter x and y in centimeters, a heading in degrees counterclockwise from +x, and a range from just above zero to 100 cm.
3. Apply the observation. The dashed overlay shows the latest observation. Cell counts update using the same probability thresholds as the command-line tool.
4. Export the grid and reopen it in the workbench or command-line reviewer.

For a short demonstration, start from a blank grid and apply a 22 cm range at (0, 0) with a 45° heading. Then repeat from the same pose to reinforce the evidence, or change the heading to inspect how a second ray projects into the map.

The GUI uses the same [mapping model](robotic_car/mapping.py) as the CLI. The desktop workbench never connects to the robot. Use the [HTTP client](robotic_car/client.py) and the documented CLI commands for physical telemetry or drive pulses.

## Results

[<img src="assets/hardware_demo_poster.webp" width="380" height="380" alt="Three robotic cars in the original demonstration">](media/hardware_demo.mp4)


The original 500 × 500 snapshot contains 1,195 free, 248,801 unknown, and 4 occupied cells at the documented thresholds. The sparse rays are evidence from the saved grid. The synthetic demo runs eight labeled range observations from two poses to exercise the same mapping code without hardware.

The [hardware video](media/hardware_demo.mp4), report excerpts and saved map are original project artifacts. The offline commands and regression suite recompute results locally. The tests cover range validation, cell classification, ray endpoints, edge clipping, deterministic demonstrations and the Python HTTP interface. See the [verification notes](docs/verification.md) for the checks performed on this version.

## Limitations

The MPU6050 yaw can drift. A single forward ultrasonic sensor samples a narrow region, and the map assumes that pose is supplied and calibrated. The grid does not recover position, fuse scans from multiple cars, or perform autonomous planning. The course report describes a multi-robot design, while the maintained code controls one car over HTTP. The requested motion timeout is enforced in the firmware loop and can be affected by loop latency. Treat the saved occupancy grid as sparse scan evidence.

## Figure Index

Open any preview to inspect its full 1440 × 900 capture.

### Saved occupancy detail

<a href="assets/screenshots/saved_occupancy_detail.png"><img src="assets/thumbs/saved_occupancy_detail.webp" width="760" height="475" alt="Saved occupancy detail"></a>

Crop around classified cells in the original occupancy map.

### Synthetic mapping run

<a href="assets/screenshots/synthetic_mapping_run.png"><img src="assets/thumbs/synthetic_mapping_run.webp" width="760" height="475" alt="Synthetic mapping run"></a>

Eight deterministic range observations from two poses. This is a software demonstration.

### Manual range observation

<a href="assets/screenshots/manual_range_observation.png"><img src="assets/thumbs/manual_range_observation.webp" width="760" height="475" alt="Manual range observation"></a>

A 22 cm observation at 45° is applied to the synthetic grid. The dashed line shows the newly added ray.

### Firmware motion expiry

<a href="assets/screenshots/firmware_motion_expiry.png"><img src="assets/thumbs/firmware_motion_expiry.webp" width="760" height="475" alt="Firmware motion expiry"></a>

The ESP8266 checks elapsed time and stops the motor outputs after a requested pulse.

### Workbench observation validation

<a href="assets/screenshots/workbench_observation_validation.png"><img src="assets/thumbs/workbench_observation_validation.webp" width="760" height="475" alt="Workbench observation validation"></a>

The desktop workbench validates the pose and range before modifying the grid.

### Chassis design photos

<a href="assets/screenshots/chassis_design_photos.png"><img src="assets/thumbs/chassis_design_photos.webp" width="760" height="475" alt="Chassis design photos"></a>

Original report page 8 documents the physical chassis and assembly.

### Hardware demo frame 2

<a href="assets/screenshots/hardware_demo_frame_2.png"><img src="assets/thumbs/hardware_demo_frame_2.webp" width="760" height="475" alt="Hardware demo frame 2"></a>

Frame at 22.1 seconds from the original hardware demonstration video.

### Hardware demo frame 3

<a href="assets/screenshots/hardware_demo_frame_3.png"><img src="assets/thumbs/hardware_demo_frame_3.webp" width="760" height="475" alt="Hardware demo frame 3"></a>

Frame at 38.4 seconds from the original hardware demonstration video.

### Mounted hardware photos

<a href="assets/screenshots/mounted_hardware_photos.png"><img src="assets/thumbs/mounted_hardware_photos.webp" width="760" height="475" alt="Mounted hardware photos"></a>

Original report page 9 records the assembled hardware.

### Report mapping output

<a href="assets/screenshots/report_mapping_output.png"><img src="assets/thumbs/report_mapping_output.webp" width="760" height="475" alt="Report mapping output"></a>

Original report page 24 shows captured software output.

### Report visualization excerpt

<a href="assets/screenshots/report_visualization_excerpt.png"><img src="assets/thumbs/report_visualization_excerpt.webp" width="760" height="475" alt="Report visualization excerpt"></a>

Original report page 25 preserves visual output from the semester project.

### Report demonstration montage

<a href="assets/screenshots/report_demonstration_montage.png"><img src="assets/thumbs/report_demonstration_montage.webp" width="760" height="475" alt="Report demonstration montage"></a>

Original report page 26 collects demonstration visuals.

### Project workflow diagram

<a href="assets/screenshots/project_workflow_diagram.png"><img src="assets/thumbs/project_workflow_diagram.webp" width="760" height="475" alt="Project workflow diagram"></a>

Original report page 28 illustrates the planned workflow.

### Saved occupancy overview

<a href="assets/screenshots/saved_occupancy_overview.png"><img src="assets/thumbs/saved_occupancy_overview.webp" width="760" height="475" alt="Saved occupancy overview"></a>

The original 500 × 500 probability grid, shown at full extent.

### Saved grid classification

<a href="assets/screenshots/saved_grid_classification.png"><img src="assets/thumbs/saved_grid_classification.webp" width="760" height="475" alt="Saved grid classification"></a>

Counts are computed from the saved grid at the documented 0.35 and 0.65 thresholds.

### Synthetic grid classification

<a href="assets/screenshots/synthetic_grid_classification.png"><img src="assets/thumbs/synthetic_grid_classification.webp" width="760" height="475" alt="Synthetic grid classification"></a>

Counts from the current mapper running the deterministic synthetic demo.

### Range-ray evidence

<a href="assets/screenshots/range_ray_evidence.png"><img src="assets/thumbs/range_ray_evidence.webp" width="760" height="475" alt="Range-ray evidence"></a>

A 7 cm hit leaves cells beyond the endpoint unchanged.

### Chassis measurement worksheet

<a href="assets/screenshots/chassis_measurement_worksheet.png"><img src="assets/thumbs/chassis_measurement_worksheet.webp" width="760" height="475" alt="Chassis measurement worksheet"></a>

Excerpt from the archived CarMeasures workbook.

### NodeMCU pin assignments

<a href="assets/screenshots/nodemcu_pin_assignments.png"><img src="assets/thumbs/nodemcu_pin_assignments.webp" width="760" height="475" alt="NodeMCU pin assignments"></a>

Pin pairs follow the maintained NodeMCU sketch.

### Occupancy evidence update

<a href="assets/screenshots/occupancy_evidence_update.png"><img src="assets/thumbs/occupancy_evidence_update.webp" width="760" height="475" alt="Occupancy evidence update"></a>

Source excerpt showing per-ray deduplication and bounded Bayesian evidence.

### Python command validation

<a href="assets/screenshots/python_command_validation.png"><img src="assets/thumbs/python_command_validation.webp" width="760" height="475" alt="Python command validation"></a>

The HTTP client validates command names and pulse durations before dispatch.

### Range response parsing

<a href="assets/screenshots/range_response_parsing.png"><img src="assets/thumbs/range_response_parsing.webp" width="760" height="475" alt="Range response parsing"></a>

Malformed and non-positive range values are represented as missing.

### HTTP control and telemetry

<a href="assets/screenshots/http_control_and_telemetry.png"><img src="assets/thumbs/http_control_and_telemetry.webp" width="760" height="475" alt="HTTP control and telemetry"></a>

Request paths from the maintained ESP8266 interface.

### Mapping regression coverage

<a href="assets/screenshots/mapping_regression_coverage.png"><img src="assets/thumbs/mapping_regression_coverage.webp" width="760" height="475" alt="Mapping regression coverage"></a>

Behaviors exercised by the local Python regression suite.

## About the Author

**Mohammad Rohaan** collaborated with **Famia Malik** and **Awais Khan** on the Fall 2024 FAST-NUCES CS-3015 project, supervised by **Umarah Qaseem**. [GitHub profile](https://github.com/rohaan2802)
