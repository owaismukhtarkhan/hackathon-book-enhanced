# /sp.structure.md

## 📘 Book Structure Specification — Physical AI & Humanoid Robotics

**Governing Authority:** `/sp.constitution.md`
**Aligned Specification:** `/sp.specify.md`
**Status:** Mandatory & Enforceable

---

## 1. Purpose

This document defines the **canonical structure** of the book, including:

* Weeks
* Modules
* Chapters
* Required specs per chapter

No content MAY exist outside this structure.
No chapter MAY exist without its governing spec.

---

## 2. Global Structural Rules

* Structure is **linear and progressive**
* No chapter may skip prerequisites
* Each chapter MUST map to exactly **one spec file**
* Each week MUST include:

  * Learning goals
  * Chapters
  * Validation checklist

---

## 3. Quarter Structure Overview

| Phase   | Weeks | Focus                      |
| ------- | ----- | -------------------------- |
| Phase 1 | 1–2   | Physical AI Foundations    |
| Phase 2 | 3–5   | ROS 2 Fundamentals         |
| Phase 3 | 6–7   | Simulation & Digital Twins |
| Phase 4 | 8–10  | NVIDIA Isaac Platform      |
| Phase 5 | 11–12 | Humanoid Intelligence      |
| Phase 6 | 13    | Conversational Robotics    |

---

## 4. Detailed Week-by-Week Structure

### Weeks 1–2: Physical AI Foundations

**Week 1**

* Chapter 1.1: What Is Physical AI?
* Chapter 1.2: Embodied Intelligence Explained
* Chapter 1.3: Physical Laws vs Digital AI

**Week 2**

* Chapter 2.1: Sensors Overview (LiDAR, Cameras, IMUs)
* Chapter 2.2: Perception in the Physical World
* Chapter 2.3: Sensor Data and Noise

---

### Weeks 3–5: ROS 2 Fundamentals

**Week 3**

* Chapter 3.1: ROS 2 Architecture
* Chapter 3.2: Nodes and Topics
* Chapter 3.3: Message Types and Data Flow

**Week 4**

* Chapter 4.1: Services and Actions
* Chapter 4.2: Parameters and Configuration
* Chapter 4.3: Launch Files

**Week 5**

* Chapter 5.1: Python ROS 2 Packages (`rclpy`)
* Chapter 5.2: Robot Control via ROS 2
* Chapter 5.3: URDF for Humanoid Robots

---

### Weeks 6–7: Simulation & Digital Twins

**Week 6**

* Chapter 6.1: Gazebo Setup
* Chapter 6.2: World Building in Gazebo
* Chapter 6.3: Physics Simulation Basics

**Week 7**

* Chapter 7.1: Sensor Simulation in Gazebo
* Chapter 7.2: Validating Sensor Streams
* Chapter 7.3: Unity Visualization for HRI

---

### Weeks 8–10: NVIDIA Isaac Platform

**Week 8**

* Chapter 8.1: Introduction to NVIDIA Isaac
* Chapter 8.2: Isaac Sim Environment Setup
* Chapter 8.3: Synthetic Data Generation

**Week 9**

* Chapter 9.1: Isaac ROS Acceleration
* Chapter 9.2: Visual SLAM (VSLAM)
* Chapter 9.3: Sensor Fusion Pipelines

**Week 10**

* Chapter 10.1: Navigation with Nav2
* Chapter 10.2: Path Planning for Humanoids
* Chapter 10.3: Sim-to-Real Transfer Concepts

---

### Weeks 11–12: Humanoid Intelligence

**Week 11**

* Chapter 11.1: Humanoid Kinematics
* Chapter 11.2: Dynamics and Balance
* Chapter 11.3: Bipedal Locomotion

**Week 12**

* Chapter 12.1: Manipulation and Grasping
* Chapter 12.2: Human–Robot Interaction
* Chapter 12.3: Safety and Constraints

---

### Week 13: Conversational Robotics

**Week 13**

* Chapter 13.1: Language Models in Robotics
* Chapter 13.2: Speech Recognition Pipelines
* Chapter 13.3: Vision–Language–Action Integration

---

## 5. Capstone Structure

**Capstone:** The Autonomous Humanoid

Required Chapters:

* C.1: Capstone System Architecture
* C.2: Perception → Planning → Action Pipeline
* C.3: Voice Command to Execution
* C.4: Navigation and Object Interaction
* C.5: Validation, Logging, and Demo

---

## 6. Spec File Mapping Rule

Each chapter MUST have:

* A matching spec file:

  * `/specs/week-X/chapter-Y.sp.md`
* Defined inputs, outputs, constraints, and validation

Missing specs invalidate the chapter.

---

## 7. Enforcement

Any deviation from this structure results in:

* Chapter rejection
* Build failure
* Hackathon non-compliance

---

**This document defines WHERE content lives. The constitution defines HOW it must be created.**
