/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

// See https://docusaurus.io/docs/site-config for all the possible
// site configuration options.

// List of projects/orgs using your project for the users page.
const users = [
  {
    caption: 'alphaville',
    // You will need to prepend the image path with your baseUrl
    // if it is not '/', like: '/test-site/img/docusaurus.svg'.
    image: '/optimization-engine/img/box.png',
    infoLink: 'https://alphaville.github.io',
    pinned: true,
  },
];

const siteConfig = {
  title: 'OpEn', // Title for your website.
  tagline: 'Fast and Accurate Nonconvex Optimization',
  url: 'https://alphaville.github.io', // Your website URL
  baseUrl: '/optimization-engine/', // Base URL for your project */
  // For github.io type URLs, you would set the url and baseUrl like:
  //   url: 'https://facebook.github.io',
  //   baseUrl: '/test-site/',

  // Used for publishing and more
  projectName: 'optimization-engine',
  organizationName: 'alphaville',
  // For top-level user or org sites, the organization is still the same.
  // e.g., for the https://JoelMarcey.github.io site, it would be set like...
  //   organizationName: 'JoelMarcey'

  // For no header links in the top nav bar -> headerLinks: [],
  headerLinks: [
    { doc: 'open-intro', label: 'Docs' },
    { blog: true, label: 'Blog' },
    { href: 'https://docs.rs/optimization_engine/*/optimization_engine/', label: 'API' },
    { href: '/optimization-engine/blog/2019/03/06/talk-to-us', label: 'Chat' },
    { href: 'https://www.github.com/alphaville/optimization-engine', label: 'Github' },
  ],

  // If you have users set above, you add it here:
  users,

  /* path to images for header/footer */
  headerIcon: 'img/box.png',
  footerIcon: 'img/box.png',
  favicon: 'img/box.png',

  /* Colors for website */
  colors: {
    primaryColor: "#800000",
    secondaryColor: "#AAAAAA",
    accentColor1: "#717171",
    accentColor2: "#F3EAFF",
    accentColor3: "#D2B9F3",
    accentColor4: "#ECF4F9",
    accentColor5: "#CBDDEA",
    accentColor6: "#2F5773"
  },


  /* Custom fonts for website */
  /*
  fonts: {
    myFont: [
      "Times New Roman",
      "Serif"
    ],
    myOtherFont: [
      "-apple-system",
      "system-ui"
    ]
  },
  */

  // This copyright info is used in /core/Footer.js and blog RSS/Atom feeds.
  copyright: `Copyright © ${new Date().getFullYear()} Pantelis Sopasakis and Emil Fresk`,

  highlight: {
    // Highlight.js theme to use for syntax highlighting in code blocks.
    theme: 'default',
  },

  // Add custom scripts here that would be placed in <script> tags.
  scripts: ['https://buttons.github.io/buttons.js'],

  // On page navigation for the current documentation page.
  onPageNav: 'separate',
  // No .html extensions for paths.
  cleanUrl: true,

  // Open Graph and Twitter card images.
  ogImage: 'img/docusaurus2.svg',
  twitterImage: 'img/docusaurus.png',

  // Show documentation's last contributor's name.
  // enableUpdateBy: true,

  // Show documentation's last update time.
  // enableUpdateTime: true,

  // You may provide arbitrary config keys to be used as needed by your
  // template. For example, if you need your repo's URL...
  repoUrl: 'https://github.com/alphaville/optimization-engine',

  docsSideNavCollapsible: true,
};

module.exports = siteConfig;

