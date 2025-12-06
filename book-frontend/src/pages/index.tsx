import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning 🤖
          </Link>
          <Link
            className="button button--primary button--lg"
            to="/docs/module-1/">
            Begin Module 1 🚀
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): React.JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Comprehensive textbook on Physical AI and Humanoid Robotics with Vision-Language-Action integration">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <section className={styles.features}>
          <div className="container padding-horiz--md">
            <div className="row">
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>🎯 Practical Learning</h3>
                  <p>Hands-on exercises and real-world projects that build practical robotics skills.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>🧠 AI Integration</h3>
                  <p>Learn how to integrate advanced AI with physical robotic systems.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className={styles.featureCard}>
                  <h3>🛠️ Industry Ready</h3>
                  <p>Skills applicable to healthcare, manufacturing, service, and research robotics.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
