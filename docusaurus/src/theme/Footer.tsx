import React from 'react';
import clsx from 'clsx';
import {ThemeClassNames} from '@docusaurus/theme-common';
import Link from '@docusaurus/Link';
import styles from './footer.module.css';

// Replace the default Footer if it exists, or create a new one
function Footer() {
  return (
    <footer className={clsx(styles.footer, ThemeClassNames.wrapper.docsPages)}>
      <div className={clsx("container", styles.container)}>
        <div className={clsx("row", styles.row)}>
          {/* Modules Column */}
          <div className={clsx("col", styles.col)}>
            <h4 className={styles.h4}>Modules</h4>
            <ul className={clsx("clean-list", styles.cleanList)}>
              <li>
                <Link to="/docs/module1/intro" className={styles.link}>
                  Module 1: ROS 2
                </Link>
              </li>
              <li>
                <Link to="/docs/module2/intro" className={styles.link}>
                  Module 2: Digital Twin
                </Link>
              </li>
              <li>
                <Link to="/docs/module3/intro" className={styles.link}>
                  Module 3: NVIDIA Isaac
                </Link>
              </li>
              <li>
                <Link to="/docs/module4/intro-vla" className={styles.link}>
                  Module 4: Vision-Language-Action
                </Link>
              </li>
            </ul>
          </div>

          {/* Author Column */}
          <div className={clsx("col", styles.col)}>
            <h4 className={styles.h4}>Author</h4>
            <ul className={clsx("clean-list", styles.cleanList)}>
              <li>
                <Link to="https://wa.me/923130221118" className={styles.link}>
                  Ahmed Raza
                </Link>
              </li>
              <li>
                <Link to="https://www.linkedin.com/in/irazaahmed" className={styles.link}>
                  LinkedIn
                </Link>
              </li>
              <li>
                <Link to="https://www.facebook.com/irazaahmed" className={styles.link}>
                  Facebook
                </Link>
              </li>
              <li>
                <Link to="https://www.instagram.com/irazaahmed" className={styles.link}>
                  Instagram
                </Link>
              </li>
            </ul>
          </div>

          {/* About Column */}
          <div className={clsx("col", styles.col)}>
            <h4 className={styles.h4}>About</h4>
            <p className={styles.aboutText}>
              An AI-native textbook created under Cybrum Solutions for the Physical AI & Humanoid Robotics Hackathon.
            </p>
          </div>
        </div>
        <div className={clsx("text--center", styles.copyright)}>
          Copyright © 2025 Cybrum Solutions. All rights reserved.
        </div>
      </div>
    </footer>
  );
}

export default Footer;