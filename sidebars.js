// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'index',
    {
      type: 'category',
      label: 'Physical AI Foundations (Weeks 1-2)',
      items: [
        'modules/week-01-02-fundamentals/physical-ai-principles',
        'modules/week-01-02-fundamentals/embodied-intelligence',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'ROS 2 Fundamentals (Weeks 3-5)',
      items: [
        'modules/week-03-05-ros2/ros2-architecture',
        'modules/week-03-05-ros2/nodes-topics-services',
        'modules/week-03-05-ros2/python-ros-packages',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Robot Simulation (Weeks 6-7)',
      items: [
        'modules/week-06-07-simulation/gazebo-setup',
        'modules/week-06-07-simulation/unity-visualization',
        'modules/week-06-07-simulation/sensor-simulation',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Conversational Robotics (Week 13)',
      items: [
        'modules/week-13-conversational/language-models-robotics',
        'modules/week-13-conversational/speech-recognition',
        'modules/week-13-conversational/multi-modal-interaction',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Assessments',
      items: [
        'modules/week-03-05-ros2/ros2-assessment',
        'modules/week-06-07-simulation/simulation-assessment',
        'modules/week-13-conversational/vla-assessment',
      ],
      collapsed: true,
    },
  ],
};

export default sidebars;