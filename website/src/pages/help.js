import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';

const helpLinks = [
  {
    title: 'Browse Docs',
    description: 'Start with installation, the Python interface, and the optimal control tutorials.',
    to: '/docs/open-intro',
  },
  {
    title: 'Join the Community',
    description: 'Reach out on Discord or Gitter if you are stuck or want feedback.',
    href: 'https://discord.gg/mfYpn4V',
  },
  {
    title: 'Stay Up to Date',
    description: 'Follow the blog for release notes, examples, and project updates.',
    to: '/blog',
  },
];

export default function Help() {
  return (
    <Layout title="Help" description="Ways to get help with OpEn.">
      <main className="simplePage">
        <section className="simplePage__hero">
          <p className="homeSection__eyebrow">Need help?</p>
          <h1>Support for docs, examples, and integration questions</h1>
          <p>
            OpEn is maintained by a small team and community. These are the
            fastest routes to get unstuck.
          </p>
        </section>

        <section className="simplePage__content helpGrid">
          {helpLinks.map((item) => (
            <article key={item.title} className="homeCard homeCard--wide">
              <h2>{item.title}</h2>
              <p>{item.description}</p>
              <Link className="button button--secondary" {...(item.to ? {to: item.to} : {href: item.href})}>
                Open
              </Link>
            </article>
          ))}
        </section>
      </main>
    </Layout>
  );
}
