// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  curriculumSidebar: [
    {
      type: 'category',
      label: 'Getting Started',
      items: ['intro', 'setup', 'quickstart'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'modules/ros2-fundamentals/index',
        'modules/ros2-fundamentals/exercises'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      items: [
        'modules/simulation-environments/index',
        'modules/simulation-environments/exercises'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: AI Perception & Navigation',
      items: [
        'modules/ai-perception/index',
        'modules/ai-perception/exercises'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Integration',
      items: [
        'modules/vla-integration/index',
        'modules/vla-integration/exercises'
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone-project-guidelines'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Assessment & Resources',
      items: [
        'assessment-rubrics',
        'learning-resources',
        'cross-module-references'
      ],
      collapsed: false,
    }
  ],
};

export default sidebars;