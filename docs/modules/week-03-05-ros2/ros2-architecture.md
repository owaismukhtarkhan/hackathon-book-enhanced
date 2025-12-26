---
title: ROS 2 Architecture
sidebar_label: ROS 2 Architecture
---

# ROS 2 Architecture

<div className="neon-border" style={{ padding: '20px', borderRadius: '8px', backgroundColor: 'var(--ifm-color-emphasis-100)' }}>

## Introduction to ROS 2

Robot Operating System 2 (ROS 2) is the next-generation middleware for robotics development. Unlike its predecessor, ROS 2 is designed for production environments with improved security, real-time capabilities, and better support for commercial applications.

</div>

<div className="neon-text" style={{ fontSize: '1.2em', margin: '15px 0' }}>

> **Core Principle**: ROS 2 bridges the gap between digital AI and physical systems by providing a robust communication framework for embodied intelligence.

</div>

### Why ROS 2?

ROS 2 addresses key limitations of ROS 1:
- **Production readiness**: Designed for commercial and industrial applications
- **Security**: Built-in security features for safe deployment
- **Real-time support**: Better timing guarantees for critical applications
- **Multi-language support**: Improved support for languages beyond C++ and Python
- **DDS integration**: Modern data distribution service for robust communication

## Core Architecture Concepts

### Nodes

Nodes are the fundamental building blocks of ROS 2 applications. Each node:
- Performs a specific task (e.g., sensor processing, control, visualization)
- Communicates with other nodes through topics, services, or actions
- Can be written in different programming languages
- Runs as an independent process

### Topics and Publishers/Subscribers

Communication in ROS 2 is primarily message-based:

- **Topics**: Named buses over which nodes exchange messages
- **Publishers**: Nodes that send messages to topics
- **Subscribers**: Nodes that receive messages from topics
- **Message types**: Strongly typed data structures defined in `.msg` files

### Services and Actions

- **Services**: Request/response communication pattern for synchronous operations
- **Actions**: Goal-oriented communication for long-running tasks with feedback

## ROS 2 Middleware (DDS)

ROS 2 uses Data Distribution Service (DDS) as its underlying communication middleware:
- Provides publish/subscribe communication
- Handles discovery and connection management
- Supports quality of service (QoS) settings
- Enables communication across different DDS implementations

## Quality of Service (QoS)

QoS profiles allow fine-tuning communication behavior:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. transient local data
- **History**: Keep last N messages vs. keep all messages
- **Deadline**: Time bounds for message delivery

## Learning Objectives

By the end of this module, you will be able to:
- Explain the core concepts of ROS 2 architecture
- Identify the key differences between ROS 1 and ROS 2
- Understand the role of nodes, topics, services, and actions
- Describe the DDS middleware and QoS profiles

## Prerequisites

- Basic understanding of Physical AI principles
- Familiarity with programming concepts (no specific language required yet)

## Duration

Estimated completion time: 6-8 hours

## What's Next

After understanding the architecture, you'll learn about nodes, topics, and services in detail, followed by hands-on Python-based ROS package development.