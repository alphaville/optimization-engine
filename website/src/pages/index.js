import React, {useEffect, useState} from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import CodeBlock from '@theme/CodeBlock';
import useBaseUrl from '@docusaurus/useBaseUrl';

const codeExample = String.raw`import opengen as og
import casadi.casadi as cs

u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)
c = 1.5 * cs.cos(u[0]) - u[1]
bounds = og.constraints.Ball2(None, 1.5)

problem = og.builder.Problem(u, p, phi) \
    .with_penalty_constraints(c) \
    .with_constraints(bounds)

build_config = og.config.BuildConfiguration() \
    .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE) \
    .with_tcp_interface_config()

meta = og.config.OptimizerMeta() \
    .with_optimizer_name("my_optimizer")

solver_config = og.config.SolverConfiguration() \
    .with_tolerance(1e-5)

builder = og.builder.OpEnOptimizerBuilder(
    problem,
    meta,
    build_config,
    solver_config,
)
builder.build()`;

const heroStats = [
  {label: 'Core language', value: 'Rust'},
  {label: 'Primary uses', value: 'MPC, MHE, Robotics'},
  {label: 'Interfaces', value: 'Python, MATLAB, C/C++, ROS, TCP'},
];

const featureCards = [
  {
    title: 'Embeddable',
    image: 'img/microchip.svg',
    body:
      'All numerical routines are written in Rust, making OpEn a strong fit for embedded targets where speed, determinism, and memory safety matter.',
  },
  {
    title: 'Accurate',
    image: 'img/bullseye.svg',
    body:
      'OpEn combines fast convergence with a practical problem formulation for nonconvex optimization, including augmented Lagrangian and penalty updates.',
  },
  {
    title: 'Fast',
    image: 'img/rocket.svg',
    body:
      'Benchmarks and applications show sub-millisecond performance in the right settings, enabling demanding control and estimation loops.',
  },
];

const ecosystemCards = [
  {
    title: 'Design and Deploy',
    image: 'img/about-open.png',
    imageClassName: 'homeCard__image--feature',
    zoomAlt: 'Detailed OpEn design and deploy workflow diagram',
    body:
      'Formulate your problem in Python or MATLAB, generate a Rust optimizer, and consume it over TCP, C/C++, ROS, or native Rust.',
  },
  {
    title: 'Research-Ready',
    image: 'img/openbenchmark.png',
    imageClassName: 'homeCard__image--feature homeCard__image--benchmark',
    zoomAlt: 'Detailed OpEn benchmark comparison figure',
    body:
      'OpEn is built for real optimization workflows, from reproducible academic experiments to embedded deployments and hardware-in-the-loop tests.',
  },
];

const documentationCard = {
  title: 'Well Documented',
  image: 'img/saturn.png',
  body:
    'The documentation covers installation, interfaces, optimal control tutorials, and end-to-end examples for robotics and autonomous systems.',
};

const mpcCard = {
  title: 'Model Predictive Control',
  image: 'img/mpc56.png',
  body: [
    'Model Predictive Control (MPC) is a powerful optimization-based control methodology. MPC has become the golden standard in control engineering as it can deal with nonlinear dynamics and state/input constraints. At its core, there is an optimization problem that needs to be solved in real time and within the ever so often stringent runtime requirements of modern applications (robotics, aerospace, automotive and more).',
    'When the system dynamics is nonlinear, or there exist nonconvex constraints (e.g., set avoidance constraints), the MPC optimization problem poses significant challenges towards the implementation and deployment of fast and reliable predictive controllers. This is where OpEn comes in: it offers a toolkit of extremely fast and robust numerical optimization methods, especially tailored for embedded applications where both speed and memory usage are of the essense.',
  ],
};

const mheCard = {
  title: 'Moving Horizon Estimation',
  image: 'img/mhe.png',
  body: [
    "Moving Horizon Estimation (MHE) is the bee's knees of nonlinear estimation: it is an optimization-based estimator for constrained nonlinear systems. MHE is backed by a strong theoretical bedrock that combines Bayesian estimation and dynamic programming; however, its applicability has been hampered by the associated computational burden and has limited its use to slow or linear dynamical systems.",
    'OpEn can unlock the huge potential of MHE and facilitate its use in robotics, automotive, aerospace and other applications with high sampling frequencies.',
  ],
};

export default function Home() {
  const baseUrl = useBaseUrl('/');
  const assetUrl = (path) => `${baseUrl}${path.replace(/^\//, '')}`;
  const promoGif = assetUrl('img/open-promo.gif');
  const boxLogo = assetUrl('img/box.png');
  const ocpStatesImage = assetUrl('img/ocp-states.png');
  const [zoomedImage, setZoomedImage] = useState(null);

  useEffect(() => {
    if (!zoomedImage) {
      return undefined;
    }

    function onKeyDown(event) {
      if (event.key === 'Escape') {
        setZoomedImage(null);
      }
    }

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [zoomedImage]);

  return (
    <Layout
      title="Optimization Engine"
      description="Fast and accurate embedded nonconvex optimization for robotics and autonomous systems."
    >
      <main className="homePage">
        <section className="homeHero">
          <div className="homeHero__content">
            <div className="homeHero__headline">
              <img className="homeHero__logo" src={boxLogo} alt="OpEn logo" />
              <div>
                <p className="homeHero__eyebrow">Optimization Engine</p>
                <h1>Fast and accurate embedded nonconvex optimization</h1>
              </div>
            </div>
            <p className="homeHero__lead">
              Build high-performance optimizers for next-generation robotics,
              autonomous vehicles, and other cyber-physical systems without
              hand-writing solver infrastructure.
            </p>
            <div className="homeHero__actions">
              <Link className="button button--primary button--lg" to="/docs/open-intro">
                Get Started
              </Link>
              <Link
                className="button button--secondary button--lg"
                href="https://arxiv.org/abs/2003.00292"
              >
                Read the Paper
              </Link>
            </div>
            <div className="homeHero__stats">
              {heroStats.map((item) => (
                <div key={item.label} className="homeHero__stat">
                  <span>{item.label}</span>
                  <strong>{item.value}</strong>
                </div>
              ))}
            </div>
          </div>
        </section>

        <section className="homeSection homeSection--alt">
          <div className="homeSection__header">
            <p className="homeSection__eyebrow">Why people use OpEn</p>
            <h2>Fast embedded optimization</h2>
          </div>
          <div className="homeCardGrid">
            {featureCards.map((card) => (
              <article key={card.title} className="homeCard">
                <img src={assetUrl(card.image)} alt="" aria-hidden="true" />
                <h3>{card.title}</h3>
                <p>{card.body}</p>
              </article>
            ))}
          </div>
        </section>

        <section className="homeSection">
          <div className="homeSplit">
            <div className="homeSplit__copy">
              <p className="homeSection__eyebrow">Easy code generation</p>
              
              <p></p>
              <p>
                Install OpEn in Python with <code>pip</code>, model your
                optimization problem with CasADi, and generate a solver that you
                can run through TCP, C/C++, ROS, or Rust.
              </p>
              <p>
                The docs in <Link to="/docs/installation">Installation</Link> and{' '}
                <Link to="/docs/python-interface">Python Interface</Link> walk
                through the flow end to end.
              </p>
              <p>
                <a
                  href="https://colab.research.google.com/drive/17vbVUbqcah9seIg17aN6bW0-T15FWrBo?usp=sharing"
                  target="_blank"
                  rel="noreferrer noopener"
                >
                  <img
                    src="https://colab.research.google.com/assets/colab-badge.svg"
                    alt="Try it In Colab"
                    title="Run this example"
                  />
                </a>
              </p>
            </div>
            <div className="homeSplit__media">
              <img
                className="homeMediaCard"
                src={promoGif}
                alt="Animated overview of OpEn code generation"
              />
            </div>
          </div>
        
        </section>

        <section className="homeSection homeSection--alt">
          <div className="homeSection__header">
            <p className="homeSection__eyebrow">Capabilities</p>
            <h2>Blazingly fast embedded optimization</h2>
          </div>
          <div className="homeFeatureGrid">
            {ecosystemCards.map((card) => (
              <article key={card.title} className="homeCard homeCard--feature">
                {card.zoomAlt ? (
                  <button
                    type="button"
                    className="homeCard__zoomButton"
                    onClick={() =>
                      setZoomedImage({
                        src: assetUrl(card.image),
                        alt: card.zoomAlt,
                      })
                    }
                    aria-label={`Open a larger view of the ${card.title} figure`}
                  >
                    <img
                      className={card.imageClassName}
                      src={assetUrl(card.image)}
                      alt={card.zoomAlt}
                    />
                  </button>
                ) : (
                  <img
                    className={card.imageClassName}
                    src={assetUrl(card.image)}
                    alt=""
                    aria-hidden="true"
                  />
                )}
                <h3>{card.title}</h3>
                <p>{card.body}</p>
              </article>
            ))}
          </div>
          <div className="homeFeatureFooter">
            <article className="homeCard homeCard--topic">
              <img src={assetUrl(mpcCard.image)} alt="" aria-hidden="true" />
              <h3>{mpcCard.title}</h3>
              {mpcCard.body.map((paragraph) => (
                <p key={paragraph}>{paragraph}</p>
              ))}
            </article>
            <article className="homeCard homeCard--topic">
              <img src={assetUrl(mheCard.image)} alt="" aria-hidden="true" />
              <h3>{mheCard.title}</h3>
              {mheCard.body.map((paragraph) => (
                <p key={paragraph}>{paragraph}</p>
              ))}
            </article>
          </div>
          <div className="homeFeatureDocsRow">
            <article className="homeCard homeCard--documented">
              <img src={assetUrl(documentationCard.image)} alt="" aria-hidden="true" />
              <h3>{documentationCard.title}</h3>
              <p>{documentationCard.body}</p>
              <Link className="button button--secondary" to="/docs/open-intro">
                Browse the Docs
              </Link>
            </article>
          </div>
        </section>

        <section className="homeSection">
          <div className="homeVideoSection">
            <div className="homeVideoSection__card">
              <div className="homeVideoCard">
                <h3>Presentation at IFAC 2020</h3>
                <p>
                  A short introduction to what OpEn does, how it works, and how
                  to use it in practice.
                </p>
                <div className="homeVideoFrame">
                  <iframe
                    src="https://www.youtube.com/embed/bHZ6eyhj3LM"
                    title="OpEn IFAC 2020 presentation"
                    allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
                    allowFullScreen
                  />
                </div>
              </div>
            </div>
          </div>
        </section>

        <section className="homeSection homeSection--alt">
          <div className="homeOcpPromo">
            <div className="homeOcpPromo__content">
              <p className="homeSection__eyebrow">Python OCP package</p>
              <h2>Design optimal control problems and MPC in Python</h2>
              <p>
                OpEn comes with a Python OCP module that facilitates the design
                of optimal control problems in an intuitive and straightforward
                way. You define the key ingredients of the problem, including
                stage and terminal costs, dynamics, and state or input
                constraints.
              </p>
              <p>
                It is a practical starting point for building nonlinear optimal
                control and MPC formulations directly in Python before
                generating an embedded optimizer.
              </p>
              <div className="homeHero__actions">
                <Link className="button button--primary" to="/docs/python-ocp-1">
                  Explore the OCP docs
                </Link>
                <Link className="button button--secondary" to="/docs/python-ocp-2">
                  See the OCP workflow
                </Link>
              </div>
            </div>
            <div className="homeOcpPromo__visual">
              <img
                className="homeMediaCard homeOcpPromo__image"
                src={ocpStatesImage}
                alt="State trajectories from an optimal control problem built with the Python OCP package"
              />
            </div>
          </div>
        </section>
      </main>
      {zoomedImage ? (
        <div
          className="imageZoomOverlay"
          role="dialog"
          aria-modal="true"
          aria-label="Magnified image preview"
          onClick={() => setZoomedImage(null)}
        >
          <div
            className="imageZoomOverlay__content"
            onClick={(event) => event.stopPropagation()}
          >
            <button
              type="button"
              className="imageZoomOverlay__close"
              onClick={() => setZoomedImage(null)}
              aria-label="Close magnified image"
            >
              Close
            </button>
            <img src={zoomedImage.src} alt={zoomedImage.alt} />
          </div>
        </div>
      ) : null}
    </Layout>
  );
}
