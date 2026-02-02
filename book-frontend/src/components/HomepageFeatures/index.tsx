import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import Link from '@docusaurus/Link';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  emoji: string;
  link: string;
  description: React.JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Module 1: The Robotic Nervous System',
    emoji: '🤖',
    link: '/docs/module-1',
    description: (
      <>
        Build the foundation with ROS 2 (Robot Operating System). Learn nodes, topics, services, and actions to create responsive robotic systems.
      </>
    ),
  },
  {
    title: 'Module 2: The Digital Twin',
    emoji: '🌐',
    link: '/docs/module-2',
    description: (
      <>
        Master simulation environments with Gazebo and Unity. Create physics-accurate digital twins to test algorithms safely before deployment.
      </>
    ),
  },
  {
    title: 'Module 3: The AI-Robot Brain',
    emoji: '🧠',
    link: '/docs/module-3',
    description: (
      <>
        Leverage NVIDIA Isaac™ for advanced perception and control. Integrate deep learning models for object detection and navigation.
      </>
    ),
  },
  {
    title: 'Module 4: Vision-Language-Action',
    emoji: '👁️',
    link: '/docs/module-4',
    description: (
      <>
        Implement state-of-the-art VLA models. Enable robots to understand natural language commands and execute complex physical tasks.
      </>
    ),
  },
];

function Feature({title, emoji, link, description}: FeatureItem) {
  return (
    <div className={clsx('col col--3')}> {/* Changed from col--4 (3 cols) to col--3 (4 cols) */}
      <Link to={link} className={styles.featureLink}>
        <div className={styles.featureCard}>
          <div className={styles.featureIconWrapper}>
              {emoji}
          </div>
          <Heading as="h3" className={styles.featureHeading}>
            {title}
          </Heading>
          <p className={styles.featureDescription}>{description}</p>
        </div>
      </Link>
    </div>
  );
}

export default function HomepageFeatures(): React.JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <Heading as="h2" className={styles.sectionHeading}>
          Explore the Textbook Modules
        </Heading>
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
