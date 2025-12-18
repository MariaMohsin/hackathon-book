
     import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import ChatbotWidget from '../ChatbotWidget';

export default function Layout(props) {
  return (
    <>
      <OriginalLayout {...props} />
      <ChatbotWidget />
    </>
  );
}