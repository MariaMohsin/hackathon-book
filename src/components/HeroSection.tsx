import React from 'react';
import styles from './HeroSection.module.css';

export default function HeroSection() {
  return (
    <div className={styles.hero}>
      {/* Animated gradient background */}
      <div className={styles.gradientBg}></div>
      
      {/* Floating shapes for visual interest */}
      <div className={styles.floatingShapes}>
        <div className={styles.shape1}></div>
        <div className={styles.shape2}></div>
        <div className={styles.shape3}></div>
      </div>

      <div className={styles.container}>
        {/* Abstract Logo */}
        <div className={styles.logoWrapper}>
          <svg
            className={styles.abstractLogo}
            viewBox="0 0 200 200"
            xmlns="http://www.w3.org/2000/svg"
          >
            {/* Outer circle */}
            <circle cx="100" cy="100" r="95" fill="none" stroke="url(#grad1)" strokeWidth="3" />
            
            {/* Inner geometric shapes */}
            <g opacity="0.8">
              <polygon points="100,30 150,80 100,130 50,80" fill="url(#grad2)" />
              <circle cx="100" cy="100" r="30" fill="url(#grad3)" opacity="0.6" />
              <path d="M 100 70 Q 130 100 100 130 Q 70 100 100 70" fill="url(#grad4)" opacity="0.7" />
            </g>

            {/* Gradient definitions */}
            <defs>
              <linearGradient id="grad1" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#FF006E" stopOpacity="1" />
                <stop offset="100%" stopColor="#FB5607" stopOpacity="1" />
              </linearGradient>
              <linearGradient id="grad2" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#FFBE0B" stopOpacity="1" />
                <stop offset="100%" stopColor="#FB5607" stopOpacity="1" />
              </linearGradient>
              <linearGradient id="grad3" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#8338EC" stopOpacity="1" />
                <stop offset="100%" stopColor="#3A86FF" stopOpacity="1" />
              </linearGradient>
              <linearGradient id="grad4" x1="0%" y1="0%" x2="100%" y2="100%">
                <stop offset="0%" stopColor="#FF006E" stopOpacity="1" />
                <stop offset="100%" stopColor="#8338EC" stopOpacity="1" />
              </linearGradient>
            </defs>
          </svg>
        </div>

        {/* Content */}
        <div className={styles.content}>
          <h1 className={styles.title}>
            Physical AI & <br />
            <span className={styles.gradient}>Humanoid Robotics</span>
          </h1>
          
          <p className={styles.tagline}>
            Master the future of robotics with AI-powered intelligent systems
          </p>

          <div className={styles.buttonGroup}>
            <a href="/docs/intro" className={styles.primaryBtn}>
              Start Learning
              <span className={styles.arrow}>→</span>
            </a>
            <a href="/docs/intro" className={styles.secondaryBtn}>
              Explore Docs
            </a>
          </div>

          {/* Stats */}
          <div className={styles.stats}>
            <div className={styles.stat}>
              <span className={styles.statNumber}>15+</span>
              <span className={styles.statLabel}>Modules</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>50+</span>
              <span className={styles.statLabel}>Code Examples</span>
            </div>
            <div className={styles.stat}>
              <span className={styles.statNumber}>100%</span>
              <span className={styles.statLabel}>Practical</span>
            </div>
          </div>
        </div>
      </div>

      {/* Scroll indicator */}
      <div className={styles.scrollIndicator}>
        <span>Scroll to explore</span>
        <div className={styles.scrollArrow}></div>
      </div>
    </div>
  );
}
