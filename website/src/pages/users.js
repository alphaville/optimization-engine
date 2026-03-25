import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import useBaseUrl from '@docusaurus/useBaseUrl';

export default function Users() {
  const {siteConfig} = useDocusaurusContext();
  const baseUrl = useBaseUrl('/');
  const assetUrl = (path) => `${baseUrl}${path.replace(/^\//, '')}`;
  const users = siteConfig.customFields?.users ?? [];

  return (
    <Layout title="Users" description="Organizations and teams using OpEn.">
      <main className="simplePage">
        <section className="simplePage__hero">
          <p className="homeSection__eyebrow">User showcase</p>
          <h1>Who is using OpEn?</h1>
          <p>
            OpEn is used in robotics, optimization, and control engineering
            workflows where fast numerical solvers need to reach real systems.
          </p>
        </section>

        <section className="simplePage__content">
          {users.length === 0 ? (
            <p>No user showcase entries have been added yet.</p>
          ) : (
            <div className="usersGrid">
              {users.map((user) => (
                <a
                  key={user.infoLink}
                  className="usersCard"
                  href={user.infoLink}
                  target="_blank"
                  rel="noreferrer"
                >
                  <img src={assetUrl(user.image)} alt={user.caption} />
                  <strong>{user.caption}</strong>
                </a>
              ))}
            </div>
          )}
        </section>
      </main>
    </Layout>
  );
}
