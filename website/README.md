This website now uses Docusaurus v3.

# Development

Install dependencies:

```sh
yarn
```

Start the local dev server:

```sh
yarn start
```

Build the production site:

```sh
yarn build
```

Preview the production build locally:

```sh
yarn serve
```

Deploy to GitHub Pages:

```sh
yarn deploy
```

# Project Layout

```
optimization-engine/
  docs/                  # documentation markdown files
  website/
    blog/                # blog posts
    src/
      css/
      pages/
    static/
      img/
      js/
    docusaurus.config.js
    sidebars.js
    package.json
```

# Notes

- The docs remain in the repository root under `/docs`.
- Legacy inline MathJax and widget scripts are stripped at build time, and equivalent site-wide support is loaded from `website/static/js`.
- Sidebar ordering now lives in `website/sidebars.js`.
