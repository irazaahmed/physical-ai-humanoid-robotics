import React from 'react';
import clsx from 'clsx';
import {
  useThemeConfig,
  useDocSidebarItemsExpandedState,
  isActiveSidebarItem,
} from '@docusaurus/theme-common';
import useIsBrowser from '@docusaurus/useIsBrowser';
import DocSidebarItemCategory from '@theme/DocSidebarItem/Category';
import DocSidebarItemLink from '@theme/DocSidebarItem/Link';
import type {Props} from '@theme/DocSidebarItem';

export default function DocSidebarItem({item, ...props}: Props): JSX.Element {
  const {expandedItem, setExpandedItem} = useDocSidebarItemsExpandedState();
  const {collapsible: themeCollapsible} = useThemeConfig().sidebar;
  const collapsible = item.collapsible ?? themeCollapsible;
  const isBrowser = useIsBrowser();
  const isActive = isActiveSidebarItem(item, isBrowser);

  if (item.type === 'category') {
    return (
      <DocSidebarItemCategory
        {...props}
        item={item}
        collapsible={collapsible}
        expanded={item.href ? expandedItem === item.href : expandedItem === item.label}
        onExpand={setExpandedItem}
      />
    );
  }

  return (
    <DocSidebarItemLink
      {...props}
      item={item}
      className={clsx(
        'menu__list-item',
        {
          'menu__list-item--collapsed': !collapsible,
        },
        isActive && 'menu__list-item--active'
      )}
    />
  );
}