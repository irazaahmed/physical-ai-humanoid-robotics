import React, {useState, useEffect} from 'react';
import clsx from 'clsx';
import {
  useLocation,
  useCollapsible,
  Collapsible,
  usePrevious,
  isActiveSidebarItem,
} from '@docusaurus/theme-common';
import Link from '@docusaurus/Link';
import {translate} from '@docusaurus/Translate';
import type {Props} from '@theme/DocSidebarItem/Category';

function DocSidebarItemCategory({
  item,
  onItemClick,
  activePath,
  level,
  expanded,
  onExpand,
  index,
  ...props
}: Props): JSX.Element {
  const {items, href, label, collapsible} = item;
  const [justClicked, setJustClicked] = useState(false);
  const {
    collapsed,
    setCollapsed,
    toggleCollapsed,
  } = useCollapsible({
    // Active categories are always initialized as expanded
    initialState: () => {
      if (expanded) {
        return false;
      }
      return collapsible && !isActiveSidebarItem(item, activePath);
    },
  });

  // If we navigate to a category, it should automatically expand itself
  // We use the previous path to avoid a double update on initial load
  const previousActivePath = usePrevious(activePath);
  useEffect(() => {
    const isActive = isActiveSidebarItem(item, activePath);
    if (isActive && previousActivePath !== activePath) {
      setCollapsed(false);
      setJustClicked(true);
    }
  }, [activePath, previousActivePath, item, setCollapsed]);

  useEffect(() => {
    if (justClicked) {
      const timeout = setTimeout(() => setJustClicked(false), 1000);
      return () => clearTimeout(timeout);
    }
    return undefined;
  }, [justClicked]);

  const menuListClassName = clsx('menu__list', {
    'menu__list--show': !collapsed,
  });

  const handleItemClick = (e: React.MouseEvent<Element, MouseEvent>) => {
    e.preventDefault();
    if (level === 1) {
      onItemClick?.(item);
    }
    if (collapsible) {
      toggleCollapsed();
      onExpand && onExpand(justClicked ? null : href);
    }
  };

  const isActive = isActiveSidebarItem(item, activePath);

  return (
    <li
      className={clsx('menu__list-item', {
        'menu__list-item--collapsed': collapsed,
      })}>
      <div
        className={clsx('menu__list-item-collapsible', {
          'menu__list-item-collapsible--active': isActive,
        })}
        {...props}>
        <Link
          className={clsx('menu__link', {
            'menu__link--sublist': level > 1,
            'menu__link--sublist-caret': collapsible,
            'menu__link--active': isActive,
          })}
          onClick={handleItemClick}
          href={href ?? '#'}
          aria-label={
            collapsible
              ? translate(
                  {
                    id: 'theme.DocSidebarItem.toggleCollapsedCategoryAriaLabel',
                    message: 'Toggle the collapsible sidebar category \'{label}\'',
                    description: 'The ARIA label to toggle the collapsible sidebar category',
                  },
                  {label},
                )
              : undefined
          }>
          {label}
        </Link>
        {href && (
          <Link
            className="menu__link menu__link--sublist-caret"
            onClick={(e) => {
              e.preventDefault();
              if (level === 1) {
                onItemClick?.(item);
              }
              if (collapsible) {
                toggleCollapsed();
                onExpand && onExpand(justClicked ? null : href);
              }
            }}
            href="#"
            aria-hidden="true">
            <svg className="menu__caret" width="20" height="20" viewBox="0 0 24 24">
              <path fill="currentColor" d="M8.59 16.58L13.17 12L8.59 7.41L10 6L16 12L10 18L8.59 16.58Z"/>
            </svg>
          </Link>
        )}
      </div>

      <Collapsible
        lazy
        as="ul"
        className={menuListClassName}
        collapsed={collapsed}>
        {items.map((childItem, childIndex) => (
          <DocSidebarItem
            key={childIndex}
            activePath={activePath}
            index={childIndex}
            item={childItem}
            level={level + 1}
            onItemClick={onItemClick}
          />
        ))}
      </Collapsible>
    </li>
  );
}

export default React.memo(DocSidebarItemCategory);