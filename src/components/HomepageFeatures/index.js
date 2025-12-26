import React from 'react';
import clsx from 'clsx';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'Physical AI & Humanoid Robotics',
    description: (
      <>
        Comprehensive educational resource for <strong>Physical AI & Humanoid Robotics</strong>.
        Learn to build AI systems that understand physics, perception, motion, and real-world interaction.
      </>
    ),
  },
  {
    title: 'Embodied Intelligence',
    description: (
      <>
        Focus on <strong>Embodied Intelligence</strong> - AI systems that comprehend and interact with the physical world,
        bridging the gap between digital AI and physical systems.
      </>
    ),
  },
  {
    title: 'Production-Grade Implementation',
    description: (
      <>
        Prioritizes <strong>production-grade implementation over theory</strong>.
        Learn to apply AI knowledge to control humanoid robots in simulated and real-world environments.
      </>
    ),
  },
];

function Feature({Svg, title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
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