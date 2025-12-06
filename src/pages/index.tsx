import React from 'react';
import Layout from '@theme/Layout';
import HeroSection from '../components/HeroSection';

export default function Home() {
  return (
    <Layout title="Physical AI & Humanoid Robotics" description="Practical guides and insights into robotics and AI">
      <main>
        <HeroSection />
      </main>
    </Layout>
  );
}
