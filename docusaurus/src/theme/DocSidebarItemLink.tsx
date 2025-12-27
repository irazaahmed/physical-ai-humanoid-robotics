import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {isActiveSidebarItem} from '@docusaurus/theme-common';
import type {Props} from '@theme/DocSidebarItem/Link';

export default function DocSidebarItemLink({
  item,
  onItemClick,
  activePath,
  level,
  index,
  ...props
}: Props): JSX.Element {
  const {href, label, className} = item;
  const isActive = isActiveSidebarItem(item, activePath);

  return (
    <li className="menu__list-item">
      <Link
        className={clsx(
          'menu__link',
          {
            'menu__link--active': isActive,
            'menu__link--sublist': level > 1,
          },
          className,
        )}
        aria-current={isActive ? 'page' : undefined}
        to={href}
        {...(level === 1
          ? {
              onClick: onItemClick
                ? () => onItemClick(item)
                : undefined,
            }
          : {})}
        {...props}>
        {label}
      </Link>
    </li>
  );
}