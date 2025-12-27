import React, {useState} from 'react';
import clsx from 'clsx';
import {
  useThemeConfig,
  useAnnouncementBar,
  useScrollPosition,
} from '@docusaurus/theme-common';
import {useFilteredAndTreeifiedSidebar} from '@docusaurus/theme-common/internal';
import {renderReportingError} from '@docusaurus/theme-error';
import ErrorBoundary from '@docusaurus/ErrorBoundary';
import DocSidebarItems from '@theme/DocSidebarItems';
import type {Props} from '@theme/DocSidebar';
import {useActiveDocContext} from '@docusaurus/plugin-content-docs/client';

import './styles.css'; // Custom styles for the sidebar

function DocSidebar({
  path,
  sidebar,
  sidebarCollapsible = true,
  onCollapse,
  isHidden,
}: Props): JSX.Element | null {
  const {navbar: {hideOnScroll}} = useThemeConfig();
  const [hidden, setHidden] = useState(false);
  const {activeDoc} = useActiveDocContext();
  const {announcementBar: {prepend, append}} = useAnnouncementBar();

  useScrollPosition(
    ({scrollY}) => {
      if (hideOnScroll) {
        setHidden(scrollY > 100);
      }
    },
    [hideOnScroll],
  );

  const filteredSidebar = useFilteredAndTreeifiedSidebar({
    sidebar,
    path,
  });

  if (filteredSidebar.length === 0) {
    return null;
  }

  // prettier-ignore
  return (
    <aside
      className={clsx(
        'sidebar',
        'thin-scrollbar',
        hideOnScroll && 'doc-sidebar--hidden',
        !isHidden && hidden && 'doc-sidebar--collapsed',
        'shadow--lw'
      )}
      onTransitionEnd={(e) => {
        if (e.propertyName === 'width' && onCollapse) {
          onCollapse(hidden);
        }
      }}>
      {prepend}
      <nav
        role="complementary"
        className={clsx('menu', 'menu--responsive', {
          'menu--show': isHidden,
        })}
        aria-label="Sidebar navigation">
        <div className="menu__list">
          <DocSidebarItems
            items={filteredSidebar}
            activePath={activeDoc?.frontMatter.sidebar_active_path ?? path}
            level={1}
          />
        </div>
      </nav>
      {append}
    </aside>
  );
}

export default function DocSidebarWrapper(props: Props): JSX.Element {
  return (
    <ErrorBoundary
      fallback={({error, tryReset}) => (
        <div className="margin-vert--xl">
          <div className="container">
            <div className="row">
              <div className="col col--6 col--offset-3">
                {renderReportingError(error, tryReset)}
              </div>
            </div>
          </div>
        </div>
      )}>
      <DocSidebar {...props} />
    </ErrorBoundary>
  );
}