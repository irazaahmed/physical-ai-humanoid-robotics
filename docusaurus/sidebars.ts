import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Module 1: Robotic Nervous System (ROS 2)',
      items: [
        'module1/intro',
        'module1/ros-overview',
        'module1/nodes-topics-services',
        'module1/rclpy-basics',
        'module1/urdf-intro',
        'module1/lab1-pubsub',
        'module1/lab2-controller',
        'module1/quiz',
        'module1/references',
        'module1/appendix-hardware-lite'
      ],
    },
    {
      type: 'category',
      label: 'Module 2 - The Digital Twin (Gazebo & Unity)',
      items: [
        'module2/intro',
        'module2/gazebo-fundamentals',
        'module2/physics-simulation'
      ],
    },
    {
      type: 'category',
      label: 'Module 3 - The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module3/intro',
        'module3/isaac-sim-architecture',
        'module3/synthetic-data-generation',
        'module3/isaac-ros-hardware-acceleration',
        'module3/vslam-perception-pipelines',
        'module3/navigation-nav2',
        'module3/sim-to-real-transfer'
      ],
    },
    {
      type: 'category',
      label: 'Module 4 - Vision-Language-Action (VLA)',
      items: [
        'module4/intro-vla',
        'module4/whisper-voice-to-action',
        'module4/language-understanding-parsing',
        'module4/llm-cognitive-planning',
        'module4/vision-language-fusion',
        'module4/ros2-action-execution',
        'module4/error-handling-safety',
        'module4/capstone-autonomous-humanoid'
      ],
    },
  ],
};

export default sidebars;
