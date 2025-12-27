import React from 'react';
import OriginalDocSidebar from '@theme-original/DocSidebar';

// Export the original component to use default Docusaurus behavior
export default function DocSidebar(props) {
  return <OriginalDocSidebar {...props} />;
}