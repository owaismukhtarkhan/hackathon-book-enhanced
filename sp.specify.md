# /sp.specify.md

## 📘 Specification: Physical AI & Humanoid Robotics (Spec-Driven Book Content)

**Governing Authority:** `/sp.constitution.md`
**Status:** Mandatory & Enforceable
**Audience:** Absolute Beginners
**Delivery Format:** Docusaurus → GitHub Pages (Hackathon Submission)

---

## 1. Book Focus & Theme

### 1.1 Core Focus

**Physical AI & Humanoid Robotics**

This book focuses on **AI systems operating in the physical world**, emphasizing **Embodied Intelligence**—AI that understands physics, perception, motion, and interaction rather than existing purely in digital environments.

### 1.2 Central Goal

Bridge the gap between:

* The **digital brain** (AI models, planning, perception)
* The **physical body** (robots, sensors, actuators)

Students will apply AI knowledge to control **humanoid robots** in **simulated and real-world environments**.

---

## 2. Quarter-Level Specification

### 2.1 Quarter Objective

By the end of the quarter, students MUST be able to:

* Design Physical AI systems
* Simulate humanoid robots
* Deploy AI-driven robotic behavior
* Integrate perception, planning, and action pipelines

### 2.2 Core Technology Stack

Mandatory tools introduced progressively:

* **ROS 2** (Robot Operating System)
* **Gazebo** (Physics Simulation)
* **Unity** (High-fidelity visualization)
* **NVIDIA Isaac Sim & Isaac ROS**
* **Vision–Language–Action (VLA) pipelines**

---

## 3. Module Specifications

### Module 1: The Robotic Nervous System (ROS 2)

**Focus:** Middleware for robot control

**Required Concepts:**

* ROS 2 nodes, topics, services, actions
* Python-based ROS agents using `rclpy`
* Bridging AI logic to robot controllers
* URDF (Unified Robot Description Format) for humanoids

**Mandatory Outputs:**

* Working ROS 2 packages
* Valid URDF humanoid model

---

### Module 2: The Digital Twin (Gazebo & Unity)

**Focus:** Physics simulation and environment modeling

**Required Concepts:**

* Physics simulation (gravity, collisions)
* Gazebo world building
* Unity-based visualization for HRI
* Sensor simulation:

  * LiDAR
  * Depth Cameras
  * IMUs

**Mandatory Outputs:**

* Simulated humanoid in Gazebo
* Sensor data streams validated

---

### Module 3: The AI–Robot Brain (NVIDIA Isaac™)

**Focus:** Advanced perception, navigation, and training

**Required Concepts:**

* NVIDIA Isaac Sim (photorealistic simulation)
* Synthetic data generation
* Isaac ROS acceleration
* Visual SLAM (VSLAM)
* Nav2 for humanoid path planning

**Mandatory Outputs:**

* Working perception pipeline
* Navigating humanoid agent in simulation

---

### Module 4: Vision–Language–Action (VLA)

**Focus:** Convergence of LLMs and Robotics

**Required Concepts:**

* Voice-to-Action pipelines
* Speech recognition
* Cognitive planning via LLMs
* Translation of natural language into ROS 2 action sequences

⚠️ **AI API Rule:**
If external AI APIs are required, **Google Gemini MUST be used**. OpenAI APIs are prohibited.

**Mandatory Outputs:**

* Natural language command → robot action execution

---

## 4. Capstone Project Specification

### 4.1 Capstone Title

**The Autonomous Humanoid**

### 4.2 Capstone Requirements

A simulated humanoid robot MUST:

1. Receive a voice command
2. Interpret intent via language model
3. Plan a sequence of actions
4. Navigate obstacles
5. Identify an object using computer vision
6. Manipulate or interact with the object

All steps MUST be:

* Spec-defined
* Logged
* Validated

---

## 5. Learning Outcomes (Mandatory)

By completing the book, students WILL:

* Understand Physical AI and embodied intelligence
* Master ROS 2 for robotic control
* Simulate robots using Gazebo and Unity
* Develop with NVIDIA Isaac platform
* Design humanoid interactions
* Build conversational robotic systems

---

## 6. Weekly Progression Specification

### Weeks 1–2: Physical AI Foundations

* Embodied intelligence principles
* Sensors: LiDAR, cameras, IMUs
* Physical laws vs digital AI

### Weeks 3–5: ROS 2 Fundamentals

* ROS 2 architecture
* Nodes, topics, services, actions
* Python-based ROS packages
* Launch files and parameters

### Weeks 6–7: Robot Simulation

* Gazebo setup
* URDF and SDF
* Physics & sensor simulation
* Intro to Unity visualization

### Weeks 8–10: NVIDIA Isaac Platform

* Isaac SDK & Sim
* Perception pipelines
* Reinforcement learning
* Sim-to-real transfer

### Weeks 11–12: Humanoid Development

* Kinematics & dynamics
* Bipedal locomotion
* Manipulation & grasping
* Human–robot interaction

### Week 13: Conversational Robotics

* Language models in robotics
* Speech recognition
* Multi-modal interaction

---

## 7. Assessment Specifications

Mandatory assessments include:

* ROS 2 package development
* Gazebo simulation
* Isaac-based perception pipeline
* Final capstone demonstration

---

## 8. Hardware & Infrastructure Specification

### 8.1 Digital Twin Workstation (Required)

Minimum requirements:

* GPU: NVIDIA RTX 4070 Ti (12GB VRAM) or higher
* CPU: Intel i7 (13th Gen+) or AMD Ryzen 9
* RAM: 64GB DDR5 (32GB absolute minimum)
* OS: Ubuntu 22.04 LTS

RTX GPUs are **non-negotiable** for Isaac Sim.

---

### 8.2 Edge AI Kit (Physical AI)

Mandatory components:

* NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)
* Intel RealSense D435i or D455
* USB IMU (BNO055 or equivalent)
* USB Microphone Array

---

### 8.3 Robot Lab Options

* **Option A (Recommended):** Proxy robots (Unitree Go2)
* **Option B:** Miniature humanoids (Unitree G1, Robotis OP3)
* **Option C:** Premium humanoid deployment (Unitree G1)

---

## 9. Cloud-Native Alternative (Ether Lab)

If RTX workstations are unavailable:

* Cloud GPU instances (AWS g5 / g6e)
* Isaac Sim via Omniverse Cloud
* Local Jetson kit for physical deployment

⚠️ Cloud-to-robot control is prohibited due to latency. Models MUST be deployed locally.

---

## 10. Compliance & Validation

Every chapter and module MUST include:

* A written spec
* Validation checklist
* Working outputs

Non-compliance results in automatic rejection under `/sp.constitution.md`.

---

**This file defines WHAT the book must teach. HOW it is taught is governed by individual chapter specs.**
