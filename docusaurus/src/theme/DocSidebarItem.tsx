import React from 'react';
import OriginalDocSidebarItem from '@theme-original/DocSidebarItem';

// Export the original component to use default Docusaurus behavior
export default function DocSidebarItem(props) {
  return <OriginalDocSidebarItem {...props} />;
}