import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <h1 className={styles.heroTitle}>
          Build the Future of <br />
          <span style={{ color: 'var(--ifm-color-primary)' }}>Physical AI</span> & <span>Robotics</span>
        </h1>
        <p className={styles.heroSubtitle}>
          A comprehensive modern guide to humanoid robotics, integrating ROS 2,
          Simulations, NVIDIA Isaac™, and Vision-Language-Action models.
        </p>
        <div className={styles.buttons}>
          <Link
            className={clsx(styles.button, styles.buttonPrimary)}
            to="/docs/intro">
            Start Learning
          </Link>
          <Link
            className={clsx(styles.button, styles.buttonSecondary)}
            to="https://github.com/osamabinadnan/physical-ai-humanoid-robotic-book">
            View on GitHub
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Comprehensive textbook on Physical AI and Humanoid Robotics with Vision-Language-Action integration">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
