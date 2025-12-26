# Quickstart Guide: Physical AI & Humanoid Robotics Book

## Prerequisites

Before starting with the Physical AI & Humanoid Robotics book, ensure you have:

1. **Hardware Requirements**:
   - Digital Twin Workstation with RTX 4070 Ti (12GB VRAM) or higher
   - Intel i7 (13th Gen+) or AMD Ryzen 9 CPU
   - 64GB DDR5 RAM (32GB absolute minimum)
   - Ubuntu 22.04 LTS OS
   - RTX GPU required for Isaac Sim (non-negotiable)

2. **Additional Hardware** (as needed):
   - NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB)
   - Intel RealSense D435i or D455
   - USB IMU (BNO055 or equivalent)
   - USB Microphone Array

3. **Software Requirements**:
   - Node.js (for Docusaurus)
   - Git
   - Python 3.8+
   - ROS 2 (Humble Hawksbill or later)

## Getting Started

### 1. Environment Setup

1. Clone the repository:
   ```bash
   git clone https://github.com/owaismukhtarkhan/hackathon-book.git
   cd hackathon-book
   ```

2. Install Docusaurus:
   ```bash
   npm install
   ```

3. Install project dependencies:
   ```bash
   npm run install
   ```

### 2. Local Development

1. Start the development server:
   ```bash
   npm start
   ```

2. Open your browser to [http://localhost:3000](http://localhost:3000) to see the documentation.

### 3. Curriculum Structure

The book follows a 13-week progressive learning model:

- **Weeks 1-2**: Physical AI Foundations
- **Weeks 3-5**: ROS 2 Fundamentals
- **Weeks 6-7**: Robot Simulation
- **Weeks 8-10**: NVIDIA Isaac Platform
- **Weeks 11-12**: Humanoid Development
- **Week 13**: Conversational Robotics

### 4. Module Completion Requirements

Each module has specific requirements that must be completed before advancing:

1. Complete all content sections
2. Execute and validate all code examples
3. Pass the module assessment with minimum required score
4. Submit for automated code review and execution tests (where applicable)

### 5. Assessment Validation

Assessments are validated through automated code review and execution tests as required by the constitution. You must achieve the minimum success thresholds:

- ROS 2 fundamentals: 90% success rate
- Simulation validation: 85% accuracy
- VLA pipeline implementation: 80% success rate
- Environment setup: 95% completion rate
- Capstone project: All 6 requirements fulfilled

### 6. Capstone Project

The final capstone project "The Autonomous Humanoid" requires implementing all 6 specific requirements:

1. Receive a voice command
2. Interpret intent via language model (Google Gemini as required)
3. Plan a sequence of actions
4. Navigate obstacles
5. Identify an object using computer vision
6. Manipulate or interact with the object

All steps must be spec-defined, logged, and validated.

## Theme and Accessibility

The documentation uses a Purple + Neon theme consistently across all pages as required by the constitution. All images include meaningful alt text, and the design follows mobile-first principles tested across viewports from 320px to 2560px.

## Technology Stack

- **Documentation**: Docusaurus (Mandatory per constitution)
- **Language**: TypeScript (Required per constitution)
- **Deployment**: GitHub Pages (Mandatory per constitution)
- **AI APIs**: Google Gemini (Required per constitution - OpenAI prohibited)

## Troubleshooting

If you encounter technical issues with required hardware during practical exercises, consult the specific module's troubleshooting section or reach out to the support team.