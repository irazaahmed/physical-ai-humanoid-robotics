import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={clsx(styles.heroContent, styles.centeredContent)}>
          <Heading as="h1" className={clsx("hero__title", styles.heroTitle)}>
            Physical AI & Humanoid Robotics
          </Heading>
          <p className="hero__subtitle">Bridging AI intelligence with humanoid robots in the physical world</p>
          <p className={clsx('hero__author', styles.authorLine)}>Written by Ahmed Raza (Founder & CEO, Cybrum Solutions)</p>

          <div className={clsx(styles.buttons, styles.centeredButtons)}>
            <Link
              className="button button--secondary button--lg header-open-book-button"
              to="/docs/module1/intro">
              Open the Book
            </Link>
            <Link
              className="button button--primary button--outline button--lg secondaryCta"
              to="https://www.linkedin.com/in/irazaahmed">
              Explore the Author
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ title, description }: { title: string, description: string }) {
  // Get an appropriate icon based on the title
  const getIcon = (title: string) => {
    switch(title) {
      case "AI-Native Textbook":
        return "🧠";
      case "Hands-On Robotics":
        return "🤖";
      case "End-to-End Coverage":
        return "🔄";
      default:
        return "✨";
    }
  };

  return (
    <div className={clsx('col col--4', styles.featureCard)}>
      <div className={styles.cardContent}>
        <h3>
          <span className={styles.cardIcon}>{getIcon(title)}</span>
          {title}
        </h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

function ModuleCard({ title, description }: { title: string, description: string }) {
  // Get an appropriate icon based on the title
  const getIcon = (title: string) => {
    switch(title) {
      case "Module 1: ROS 2":
        return "📡";
      case "Module 2: Digital Twin":
        return "🏗️";
      case "Module 3: NVIDIA Isaac":
        return "⚡";
      case "Module 4: Vision-Language-Action":
        return "👁️";
      default:
        return "📚";
    }
  };

  return (
    <div className={clsx('col col--3', styles.moduleCard)}>
      <div className={styles.moduleCardContent}>
        <h3>
          <span className={styles.cardIcon}>{getIcon(title)}</span>
          {title}
        </h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Textbook - Learn with interactive content and practical labs">
      <HomepageHeader />
      <main>
        <section className={styles.featuresSection} id="features">
          <div className="container padding-vert--lg">
            <div className="row">
              <FeatureCard
                title="AI-Native Textbook"
                description="Built with Spec-Kit Plus and Claude Code for dynamic, AI-driven content generation."
              />
              <FeatureCard
                title="Hands-On Robotics"
                description="Practical labs with ROS 2, Gazebo, Unity, and NVIDIA Isaac simulation environments."
              />
              <FeatureCard
                title="End-to-End Coverage"
                description="From ROS 2 fundamentals to VLA systems for autonomous humanoid robots."
              />
            </div>
          </div>
        </section>

        <section className={styles.modulesSection}>
          <div className="container padding-vert--lg">
            <div className="text--center padding-bottom--lg">
              <Heading as="h2">Course Modules</Heading>
            </div>
            <div className="row">
              <ModuleCard
                title="Module 1: Robotic Nervous System 2"
                description="Robotic nervous system, nodes, URDF, controllers"
              />
              <ModuleCard
                title="Module 2: Digital Twin"
                description="Gazebo physics, Unity simulation, sensors"
              />
              <ModuleCard
                title="Module 3: NVIDIA Isaac"
                description="Perception, navigation, sim-to-real"
              />
              <ModuleCard
                title="Module 4: Vision-Language-Action"
                description="Voice to action, LLM planning, humanoid capstone"
              />
            </div>
          </div>
        </section>

        <section className={styles.ctaSection}>
          <div className="container text--center padding-vert--xl">
            <Heading as="h2">Ready to dive into Physical AI & Robotics?</Heading>
            <Link
              className={clsx("button button--primary button--lg", styles.ctaButton)}
              to="/docs/module1/intro">
              Start Reading Now
            </Link>
          </div>
        </section>
      </main>
    </Layout>
  );
}
