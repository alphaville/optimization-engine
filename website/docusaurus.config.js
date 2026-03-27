const {themes} = require('prism-react-renderer');

const baseUrl = '/optimization-engine/';

const users = [
  {
    caption: 'alphaville',
    image: 'img/box.png',
    infoLink: 'https://alphaville.github.io',
    pinned: true,
  },
];

function preprocessLegacyMarkdown({fileContent}) {
  return fileContent
    .replace(/<script\b[\s\S]*?<\/script>\s*/gi, '')
    .replace(/<link\b[^>]*font-awesome[^>]*>\s*/gi, '')
    .replace(/(!\[[^\]]*]\()\/optimization-engine\//g, '$1pathname:///optimization-engine/')
    .replace(
      /onclick="toggleCollapseExpand\('([^']+)',\s*'([^']+)',\s*'([^']+)'\)"/gi,
      'data-legacy-toggle-button="$1" data-legacy-toggle-target="$2" data-legacy-toggle-label="$3"',
    )
    .replace(/target=(['"])blank\1/gi, 'target="_blank"')
    .replace(/\n{3,}/g, '\n\n');
}

module.exports = {
  title: 'OpEn',
  tagline: 'Fast and Accurate Nonconvex Optimization',
  favicon: 'img/box.png',
  url: 'https://alphaville.github.io',
  baseUrl,
  organizationName: 'alphaville',
  projectName: 'optimization-engine',
  deploymentBranch: 'gh-pages',
  onBrokenLinks: 'warn',
  trailingSlash: false,
  baseUrlIssueBanner: true,
  markdown: {
    format: 'detect',
    hooks: {
      onBrokenMarkdownLinks: 'warn',
    },
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
    preprocessor: preprocessLegacyMarkdown,
  },
  themes: [],
  presets: [
    [
      'classic',
      {
        docs: {
          path: '../docs',
          routeBasePath: 'docs',
          sidebarPath: require.resolve('./sidebars.js'),
        },
        blog: {
          path: './blog',
          showReadingTime: true,
          onInlineAuthors: 'ignore',
          onUntruncatedBlogPosts: 'ignore',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
  stylesheets: [
    'https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css',
  ],
  scripts: [
    `${baseUrl}js/mathjax-config.js`,
    {
      src: 'https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js',
      defer: true,
    },
    {
      src: `${baseUrl}js/legacy-docs.js`,
      defer: true,
    },
    {
      src: 'https://buttons.github.io/buttons.js',
      async: true,
    },
  ],
  customFields: {
    users,
  },
  themeConfig: {
    image: 'img/open-functionality.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: true,
      respectPrefersColorScheme: false,
    },
    prism: {
      theme: themes.github,
      additionalLanguages: ['bash', 'matlab'],
    },
    navbar: {
      title: 'Optimization Engine',
      logo: {
        alt: 'OpEn logo',
        src: 'img/box.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'docsSidebar',
          position: 'left',
          label: 'Docs',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://docs.rs/optimization_engine/*/optimization_engine/',
          label: 'Rust API',
          position: 'right',
        },
        {
          href: 'https://alphaville.github.io/optimization-engine/api-dox/html/index.html',
          label: 'Opengen API',
          position: 'right',
        },
        {
          to: '/blog/2019/03/06/talk-to-us',
          label: 'Chat',
          position: 'right',
        },
        {
          href: 'https://github.com/alphaville/optimization-engine',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {label: 'Getting Started', to: '/docs/open-intro'},
            {label: 'Python Interface', to: '/docs/python-interface'},
            {label: 'MATLAB Interface', to: '/docs/matlab-interface'},
            {label: 'Docker', to: '/docs/docker'},
          ],
        },
        {
          title: 'Community',
          items: [
            {label: 'User Showcase', to: '/users'},
            {label: 'Discord community', href: 'https://discord.gg/mfYpn4V'},
            {label: 'Chat on Gitter', href: 'https://gitter.im/alphaville/optimization-engine'},
          ],
        },
        {
          title: 'More',
          items: [
            {label: 'Blog', to: '/blog'},
            {label: 'GitHub', href: 'https://github.com/alphaville/optimization-engine'},
            {label: 'OpenHub', href: 'https://www.openhub.net/p/optimization-engine'},
            {
              html: '<a href="https://github.com/alphaville/optimization-engine" aria-label="GitHub stars for alphaville/optimization-engine"><img src="https://img.shields.io/github/stars/alphaville/optimization-engine?style=social" alt="GitHub stars for alphaville/optimization-engine" /></a>',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Pantelis Sopasakis and Emil Fresk<br />Box icon made by <a href="https://www.flaticon.com/authors/freepik">Freepik</a> from <a href="https://www.flaticon.com/">Flaticon</a>.`,
    },
  },
};
