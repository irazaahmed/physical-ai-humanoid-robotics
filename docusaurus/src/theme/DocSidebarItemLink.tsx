import React from 'react';
import OriginalDocSidebarItemLink from '@theme-original/DocSidebarItemLink';

// Export the original component to use default Docusaurus behavior
export default function DocSidebarItemLink(props) {
  return <OriginalDocSidebarItemLink {...props} />;
}