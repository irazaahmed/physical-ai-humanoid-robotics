import React from 'react';
import OriginalComponent from '@theme-original/NavbarItem/DefaultNavbarItem';

export default function DefaultNavbarItemWrapper(props) {
  return (
    <>
      <OriginalComponent {...props} />
    </>
  );
}