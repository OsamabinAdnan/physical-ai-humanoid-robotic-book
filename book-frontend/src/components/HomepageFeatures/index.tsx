import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  emoji: string;
  description: React.JSX.Element;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI & Robotics',
    emoji: '🤖',
    description: (
      <>
        Explore the convergence of artificial intelligence and physical systems. Learn how AI agents interact with the real world through sensors, actuators, and robotic platforms.
      </>
    ),
  },
  {
    title: 'Humanoid Robotics',
    emoji: '🦾',
    description: (
      <>
        Master the design and control of humanoid robots with advanced locomotion, manipulation, and human-robot interaction capabilities.
      </>
    ),
  },
  {
    title: 'Vision-Language-Action',
    emoji: '👁️',
    description: (
      <>
        Implement cutting-edge Vision-Language-Action systems that enable robots to understand natural language commands and execute complex tasks.
      </>
    ),
  },
];

function Feature({title, emoji, description}: FeatureItem) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <div className={styles.featureSvg}>
          <span style={{ fontSize: '3rem', display: 'block' }}>{emoji}</span>
        </div>
        <Heading as="h3" className={styles.featureHeading}>
          {title}
        </Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures(): React.JSX.Element {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
