English| [简体中文](./README_cn.md)

# Function Introduction

hobotcv_benchmark is a tool for statistical analysis of processing time for images using hobot_cv VPS, BPU, and OpenCV. By default, hobotcv_benchmark outputs the frame rate and the maximum, minimum, and average latency per frame every 1000 calls. Users can modify the startup parameters to configure different acceleration methods and image operations. Image data is sourced from local image feeds.

# Compilation

## Dependencies

- hobot_cv package

## Development Environment

- Programming Language: C/C++
- Development Platform: X3/X86
- System Version: Ubuntu 20.0.4
- Compilation Toolchain: Linux GCC 9.3.0 / Linaro GCC 9.3.0

# User Guide

## Dependencies

## Parameters

| Parameter Name  | Meaning                     | Value                         | Default Value     |
| --------------  | ---------------------------  | ----------------------------- | ----------------- |
| image_file     | Path of the input image       | String                        | config/test.jpg   |
| dst_width      | Width of output image after resize | Int                         | 960               |
| dst_height     | Height of output image after resize | Int                        | 540               |
| rotation       | Rotation angle                 | 90/180/270                    | 180               |
| process_type   | Image processing operation     | 0: resize 1: rotate           | 0                 |
| img_fmt        | Image format for hobot_cv interface | 0: cv::Mat 1: nv12        | 0                 |
| speedup_type   | Image processing acceleration method | 0: VPS 1: BPU 2: OpenCV   | 0                 |
| static_cycle   | Number of images processed in one cycle | Int                   | 1000               |

## Execution

To run, use ros2 run to start:

```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# Test benchmark data for resize using BPU acceleration method, interface type is cv::Mat data interface
ros2 run hobot_cv hobotcv_benchmark --ros-args -p speedup_type:=1 -p img_fmt:=0 -p process_type:=0

```

To run using launch file:```
export COLCON_CURRENT_PREFIX=./install
source ./install/setup.bash

# Start launch file
ros2 launch hobot_cv hobot_cv_benchmark.launch.py

# Calculate the time consumption of opencv resize
ros2 launch hobot_cv hobot_cv_benchmark.launch.py speedup_type:=2 process_type:=0 image_file:=config/test.jpg dst_width:=960 dst_height:=540

```

## Result Analysis
Launch Command: ros2 launch hobot_cv hobot_cv_benchmark.launch.py
Output:
```
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [hobotcv_benchmark-1]: process started with pid [5796]
[hobotcv_benchmark-1] [WARN] [1666377438.249075414] [benchmark]: This is hobot_cv benchmark!
[hobotcv_benchmark-1] [ERROR]["vps"][vps/hb_vps_api.c:191] [87.462736]HB_VPS_StopGrp[191]: VPS StopGrp err: bad group num 4!
[hobotcv_benchmark-1]
[hobotcv_benchmark-1] [ERROR]["vps"][vps/hb_vps_api.c:87] [87.462805]HB_VPS_DestroyGrp[87]: VPS destroy grp error: unexist group
[hobotcv_benchmark-1]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 88.4777fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.295ms,  max: 11.938ms,  min: 11.12ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 89.3716fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.1855ms,  max: 11.387ms,  min: 11.118ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 88.586fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.2793ms,  max: 12.069ms,  min: 11.102ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 89.4254fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.178ms,  max: 11.418ms,  min: 11.102ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 89.3923fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.1729ms,  max: 12.385ms,  min: 11.094ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 88.6265fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.2744ms,  max: 12.102ms,  min: 11.082ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 89.464fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.1735ms,  max: 11.423ms,  min: 11.102ms]
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 Throughput 88.7525fps
[hobotcv_benchmark-1]  hobotcv VPS mat resize 1920x1080 to 960x540 latency: [avg: 11.2604ms,  max: 11.837ms,  min: 11.111ms]

```
According to the log output, the benchmark test was conducted using the hobotcv vps acceleration method, with the input image format as cv::Mat. The average, maximum, and minimum time consumption for resizing 1920x1080 images to 960x540 every 1000 times was calculated, and the output frame rate of hobot_cv was also displayed.

## Important Note

Pay attention to the frequency lock when running:
```
sudo bash -c 'echo performance > /sys/devices/system/cpu/cpufreq/policy0/scaling_governor'
```To check the CPU usage of the benchmark process:

1. First use `ps -ef | grep hobotcv_benchmark` to check the process ID.
2. Then use `top -p process id` to check the CPU usage and memory usage of the process.

To check the BPU usage of the benchmark process: `hrut_somstatus`

## Comparison of Different Load Tests

Using the hobot_cv benchmark tool for testing, reading local images with a resolution of 1920x1080, resizing the resolution to 512x512, with a static_cycle of 1000 for processing images in one cycle.
Calculate the maximum, minimum, and average time-consuming values for VPS, BPU, and OPENCV in the following cases, output the frame rate, and resource usage. The statistics do not include the time required for the initial hardware setup.
- case1: Testing under no load condition
- case2: Starting the test program to make the CPU usage of each core around 50%. Testing under CPU load condition.
- case3: VPS load, testing with two hobot_cv VPS acceleration programs already started.
- case4: BPU load 35% (starting dnn program to infer fcos model, CPU load 30%)
- case5: BPU load 50% (starting dnn program to infer yolov5 model, CPU load 16.6%)

<table>
  <tr>
    <th></th>
    <th colspan="3">No Load</th>
    <th colspan="3">CPU Load 50%</th>
    <th colspan="3">VPS Load</th>
    <th colspan="3">BPU Load 35%</th>
    <th colspan="3">BPU Load 50%</th>
  </tr >
  <tr>
    <td>Statistical Type</td>
    <td>VPS Acceleration</td>
    <td>BPU Acceleration</td>
    <td>opencv</td>
    <td>VPS Acceleration</td>
    <td>BPU Acceleration</td>
    <td>opencv</td>
    <td>VPS Acceleration</td>
    <td>BPU Acceleration</td>
    <td>opencv</td>
    <td>VPS Acceleration</td>
    <td>BPU Acceleration</td>
    <td>opencv</td>
    <td>VPS Acceleration</td>
    <td>BPU Acceleration</td>
    <td>opencv</td>
  </tr >
  <tr >
    <td>Maximum Value (ms)</td>
    <td>11.699</td>
    <td>8.18</td>
    <td>19.326</td>
    <td>18.906</td>    <td>14.899</td>
    <td>39.086</td>
    <td>26.711</td>
    <td>11.683</td>
    <td>18.38</td>
    <td>13.667</td>
    <td>21.412</td>
    <td>20.293</td>
    <td>10.817</td>
    <td>63.973</td>
    <td>19.748</td>
  </tr>
  <tr >
    <td>Minimum Value (ms)</td>
    <td>10.752</td>
    <td>5.562</td>
    <td>7.397</td>
    <td>10.819</td>
    <td>5.602</td>
    <td>7.616</td>
    <td>11.124</td>
    <td>5.827</td>
    <td>7.381</td>
    <td>11.314</td>
    <td>5.831</td>
    <td>7.52</td>
    <td>13.383</td>
    <td>5.768</td>
    <td>7.55</td>
  </tr>
  <tr >
    <td>Average Value (ms)</td>
    <td>10.8882</td>
    <td>5.79068</td>
    <td>8.21311</td>
    <td>10.946</td>
    <td>5.945</td>
    <td>16.55</td>
    <td>15.66</td>
    <td>6.6787</td>
    <td>9.0663</td>
    <td>11.658</td>
    <td>8.418</td>
    <td>10.264</td>
    <td>11.333</td>
    <td>10.714</td>
    <td>9.2706</td>
  </tr>
  <tr >
    <td>Frame Rate (fps)</td>    <td>91.8155</td>
    <td>172.546</td>
    <td>121.567</td>
    <td>91.2686</td>
    <td>164.10</td>
    <td>60.365</td>
    <td>63.81</td>
    <td>149.57</td>
    <td>110.17</td>
    <td>85.736</td>
    <td>118.59</td>
    <td>97.328</td>
    <td>88.202</td>
    <td>92.884</td>
    <td>107.73</td>
  </tr>
  <tr >
    <td>CPU Usage (%)</td>
    <td>20.3</td>
    <td>71.1</td>
    <td>380</td>
    <td>20.3</td>
    <td>67.9</td>
    <td>210</td>
    <td>17.3</td>
    <td>71.8</td>
    <td>355.6</td>
    <td>33.1</td>
    <td>54.5</td>
    <td>324.8</td>
    <td>23.2</td>
    <td>44.2</td>
    <td>350.2</td>
  </tr>
  <tr >
    <td>CPU Usage at 30fps (%)</td>
    <td>6.63</td>
    <td>12.36</td>
    <td>93.82</td>
    <td>6.67</td>
    <td>12.41</td>
    <td>104.37</td>
    <td>8.13</td>
    <td>14.42</td>
    <td>96.89</td>
    <td>11.57</td>
    <td>13.78</td>
    <td>100.12</td>
    <td>7.89</td>
    <td>14.27</td>    <td>97.52</td>
  </tr>
  <tr >
    <td>Ratio bpu0</td>
    <td>0</td>
    <td>35</td>
    <td>0</td>
    <td>0</td>
    <td>34</td>
    <td>0</td>
    <td>0</td>
    <td>34</td>
    <td>0</td>
    <td>35</td>
    <td>61</td>
    <td>34</td>
    <td>44</td>
    <td>62</td>
    <td>41</td>
  </tr>
  <tr >
    <td>Ratio bpu1</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>0</td>
    <td>33</td>
    <td>35</td>
    <td>33</td>
    <td>43</td>
    <td>49</td>
    <td>47</td>
  </tr>
</table>