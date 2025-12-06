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
  // Manual sidebar configuration for the Physical AI & Humanoid Robotics course
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsed: false,
      link: {type: 'doc', id: 'module-1/index'},
      items: [
        'module-1/chapter-1/index',
        'module-1/chapter-2/index',
        'module-1/chapter-3/index',
        'module-1/chapter-4/index'
      ]
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsed: false,
      link: {type: 'doc', id: 'module-2/index'},
      items: [
        'module-2/chapter-5/index',
        'module-2/chapter-6/index',
        'module-2/chapter-7/index',
        'module-2/chapter-8/index'
      ]
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      collapsed: false,
      link: {type: 'doc', id: 'module-3/index'},
      items: [
        'module-3/chapter-9/index',
        'module-3/chapter-10/index',
        'module-3/chapter-11/index',
        'module-3/chapter-12/index'
      ]
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsed: false,
      link: {type: 'doc', id: 'module-4/index'},
      items: [
        'module-4/chapter-13/index',
        'module-4/chapter-14/index',
        'module-4/chapter-15/index',
        'module-4/chapter-16/index'
      ]
    },
    'module-5/index',
    'glossary'
  ]
};

export default sidebars;
