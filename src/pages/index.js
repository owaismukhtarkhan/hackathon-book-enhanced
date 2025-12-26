import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className={styles.heroContent}>
              <h1 className={clsx('hero__title', styles.title)}>
                {siteConfig.title}
              </h1>
              <p className={clsx('hero__subtitle', styles.subtitle)}>
                {siteConfig.tagline}
              </p>
              <div className={styles.buttons}>
                <Link
                  className="button button--primary button--lg button--modern"
                  to="/docs/">
                  Start Learning
                </Link>
                <Link
                  className="button button--secondary button--outline button--lg"
                  to="/docs/modules/week-01-02-fundamentals/physical-ai-principles">
                  Explore Curriculum
                </Link>
              </div>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Educational resource for Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}