# Object Tracking using Kalman Filter in MATLAB

<p align="center">
  <img src="https://logos-world.net/wp-content/uploads/2020/12/MATLAB-Emblem.png" width="300"/>
</p>

<p align="center">
  <strong>An implementation of a Kalman filter for real-time and video-based object tracking.</strong>
</p>

<p align="center">
  <img src="https://img.shields.io/badge/Language-MATLAB-blue.svg" alt="Language: MATLAB">
  <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="License: MIT">
</p>

---

## 📖 Overview

This project presents a robust object tracking system developed in MATLAB. It utilizes the **Kalman filter**, a powerful algorithm for estimating the state of a dynamic system from a series of noisy measurements. The primary application demonstrated here is tracking a colored ball in both pre-recorded videos and a live webcam feed.

The repository includes two main versions of the implementation:
* **`kalmanPROYECTOV2.m`**: Tracks an object in a pre-recorded video file (`.mp4`).
* **`kalmanPROYECTOV3.m`**: Tracks an object in real-time using a webcam.

The core of the project lies in combining color-based segmentation and motion detection to identify the object of interest, and then applying a Kalman filter to predict and correct its trajectory, resulting in smooth and accurate tracking even with occlusions or measurement noise.

---

## ✨ Features

* **State Estimation**: Implements a linear Kalman filter to estimate the position and velocity of the tracked object.
* **Real-Time Tracking**: `v3` of the code interfaces directly with your webcam for live object tracking.
* **Video File Processing**: `v2` of the code processes video files to track objects frame-by-frame.
* **Hybrid Detection Method**: Combines YCbCr color space segmentation with frame-differencing for robust object detection.
* **Dynamic Noise Adaptation**: The filter's process noise (`Q` matrix) can be adjusted based on the object's dynamics.
* **Visualization**: Provides real-time visual feedback by drawing a bounding box around the estimated position of the object.

---

## 🚀 Getting Started

### Prerequisites

* **MATLAB** (R2016b or later recommended)
* **Image Processing Toolbox**
* **Computer Vision Toolbox**
* A webcam for the real-time version (`kalmanPROYECTOV3.m`).

### Installation

1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/YOUR_USERNAME/YOUR_REPOSITORY_NAME.git](https://github.com/YOUR_USERNAME/YOUR_REPOSITORY_NAME.git)
    ```
2.  **Navigate to the project directory:**
    ```bash
    cd YOUR_REPOSITORY_NAME
    ```
3.  **Open MATLAB.**

### Usage

#### Version 2: Tracking from a Video File

1.  Place your video file (e.g., `pelota_rebote_1D_comprimido.mp4`) in the project directory.
2.  Open the `kalmanPROYECTOV2.m` script in MATLAB.
3.  Modify the `video_name` variable to match your video file's name.
4.  Run the script. The output will be a video file named `tracked_ball_output.mp4` showing the tracking results.

#### Version 3: Real-Time Tracking with Webcam

1.  Ensure your webcam is connected and recognized by MATLAB.
2.  Open the `kalmanPROYECTOV3.m` script in MATLAB.
3.  Run the script. A figure window will open, displaying the live webcam feed with the tracked object.
4.  To stop the tracking, simply close the figure window or press `Ctrl+C` in the MATLAB command window.

---

## 🔧 Configuration and Parameters

The behavior of the tracker can be fine-tuned by adjusting the following parameters in the scripts:

* **`u_cb`, `u_cr`, `sigma_cb`, `sigma_cr`**: These define the mean and standard deviation for the Cb and Cr channels in the YCbCr color space. Adjust these to track objects of different colors.
* **`dt`**: The time interval between frames. In `v2`, it's set to `1` but is dynamically calculated in `v3` based on the frame rate.
* **`A`**: The state transition matrix. It models the physics of the object's motion.
* **`H`**: The observation matrix. It maps the state space to the measurement space.
* **`Q`**: The process noise covariance. Represents the uncertainty in the motion model.
* **`R`**: The measurement noise covariance. Represents the uncertainty in our detection/measurement.
* **`P`**: The initial error covariance matrix.
* **`alpha`**: A smoothing factor for the estimated position.

---

## Technical Details

The tracking algorithm follows these main steps in a loop:

1.  **Image Acquisition**: A new frame is captured from the video or webcam.
2.  **Object Detection (Measurement)**:
    * The frame is converted to the **YCbCr color space** to segment the object based on its color.
    * **Motion segmentation** is performed by calculating the absolute difference between the current and previous frames.
    * The results of color and motion segmentation are combined to create a final binary mask of the object.
    * A **Region of Interest (ROI)** (a bounding box) is calculated from this mask.
3.  **Kalman Filter Cycle**:
    * **Prediction**: The filter predicts the next state (position and velocity) of the object based on the current state and the motion model (`A` matrix).
    * **Correction**: If the object is detected, the measurement (the position from the ROI) is used to correct the predicted state. The **Kalman Gain** determines how much to trust the new measurement versus the prediction.
4.  **Visualization**: The estimated position is used to draw a bounding box on the output frame.

This cycle of prediction and correction allows the filter to maintain a smooth track of the object, even if the detection fails for a few frames.

---

## 🤝 Contributing

Contributions are welcome! If you have suggestions for improvements, please feel free to open an issue or submit a pull request.

---


