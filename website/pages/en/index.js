/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

const React = require("react");

const CompLibrary = require("../../core/CompLibrary.js");

const { MarkdownBlock, GridBlock, Container } = CompLibrary; /* Used to read markdown */

const siteConfig = require(`${process.cwd()}/siteConfig.js`);

function docUrl(doc) {
  return `${siteConfig.baseUrl}docs/${doc}`;
}

function imgUrl(img) {
  return `${siteConfig.baseUrl}img/${img}`;
}
class Button extends React.Component {
  render() {
    return (
      <div className="pluginWrapper buttonWrapper">
        <a className="button hero" href={this.props.href} target={this.props.target}>
          {this.props.children}
        </a>
      </div>
    );
  }
}

Button.defaultProps = {
  target: "_self"
};

const SplashContainer = props => (
  <div className="homeContainer">
    <div className="homeSplashFade">
      <div className="wrapper homeWrapper">{props.children}</div>
    </div>
  </div>
);

const Logo = props => (
  <div className="projectLogo">
    <img src={props.img_src} alt="Project Logo" />
  </div>
);

const ProjectTitle = () => (
  <React.Fragment>
    <div style={{ display: "flex", justifyContent: "center", alignItems: "center" }}>
      <img src={"img/box.png"} alt="OpEn logo" width={100} height={100} />
      <h1 className="projectTitle">{siteConfig.title}</h1>
    </div>
    <h2 style={{ marginTop: "0.5em" }}>
      Optimization Engine
    </h2>
    <p>
      Fast &amp; Accurate Embedded Optimization<br/> for next-generation Robotics and Autonomous Systems
    </p>
  </React.Fragment>
);

const PromoSection = props => (
  <div className="section promoSection">
    <div className="promoRow">
      <div className="pluginRowBlock">{props.children}</div>
    </div>
  </div>
);

class HomeSplash extends React.Component {
  render() {
    return (
      <SplashContainer>
        <div className="inner">
          <ProjectTitle />
<h2><a href="https://twitter.com/hashtag/BlackLivesMatter">#BlackLivesMatter</a> - Optimization Engine stands in solidarity with the black community in the US and worldwide and all	 people's struggle for justice and freedom from oppression.</h2>
<h2>Racism cannot be tolerated!</h2>
          <PromoSection>
            <Button href={docUrl("open-intro")}>
              Get Started
            </Button>
          </PromoSection>
        </div>
      </SplashContainer>
    );
  }
}

const Block = props => (
  <Container
    id={props.id}
    background={props.background}
    className={props.className}
  >
    <GridBlock align="center" contents={props.children} layout={props.layout} />
  </Container>
);

const AboutOpen = props => (
  <Block className="aboutBlock">
    {[
      {
        content: "<div style='text-align:left'><p><b>Design &amp; Deploy</b> your high-performance embedded optimizer in no time... <ul><li>Formulate your problem in Python or MATLAB</li><li>Build an optimizer (Rust)</li><li>Consume it over a TCP interface or</li><li>Call it in C/C++ (and ROS), or Rust</li></ul></p> Focus on your design, not numerical optimization!</div>",
        image: imgUrl("about-open.png"),
        imageAlign: 'left',
        imageAlt: "organisation map of Optimization Engine (OpEn) and how it works: design interfaces, solver in Rust, core solver engine, TCP/IP socket interface and C/C++ bindings",
        title: "Embedded Optimization Made Easy"
      }
    ]}
  </Block>

);

const FeaturesTop = props => (
  <Block layout="threeColumn" className="featureBlock">
    {[
      {
        content: "All numerical routines are written in **Rust**: a **fast** and **safe** programming language, which is ideal for embedded applications.",
        image: imgUrl("microchip.svg"),
        imageAlt: "microchip icon black and white",
        imageAlign: 'top',
        title: "Embeddable"
      },
      {
        content: "**Optimization Engine** can produce solutions of high accuracy thanks to the fast convergence properties of its numerical algorithm (PANOC).",
        image: imgUrl("bullseye.svg"),
        imageAlt: "bullseye icon black and white",
        imageAlign: 'top',
        title: "Accurate"
      },
      {
        content: "**Sub-millisecond** fast numerical nonconvex optimization tested on several platforms: check out our <a href='https://arxiv.org/abs/2003.00292' target='_blank'>benchmarks</a>.",
        image: imgUrl("rocket.svg"),
        imageAlt: "small rocket icon black and white",
        imageAlign: 'top',
        title: "Fast"
      }
    ]}
  </Block>
);

const FeaturesTopTwo = props => (
  <Block layout="threeColumn" className="featureBlock">
    {[
      {
        content: "**OpEn** is **seriously** easy to use! You can call if from Rust, MATLAB, Python and other programming languages - even over the Internet! A <a href='/optimization-engine/docs/docker'>Docker Image</a> is available!",
        image: imgUrl("happy.png"),
        imageAlt: "smiley face icon black and white",
        imageAlign: 'top',
        title: "User Friendly"
      },
      {
        content: "**OpEn is open**: it is a free, open source, MIT/Apachev2-licensed software with a growing community that actively supports its development. Reach us [here](/optimization-engine/blog/2019/03/06/talk-to-us).",
        image: imgUrl("open.png"),
        imageAlt: "group of people icon black and white",
        imageAlign: 'top',
        title: "Community"
      },
      {
        content: "**Well documented** with lots of examples. Among 10% best documented open-source Rust projects according to <a href='https://www.openhub.net/p/optimization-engine/factoids#FactoidCommentsVeryHigh' target='_blank'>openhub</a>.",
        image: imgUrl("saturn.png"),
        imageAlt: "saturn icon black and white",
        imageAlign: 'top',
        title: "Documented"
      }
    ]}
  </Block>

);

const SuperFastBlock = props => (
  <Block className="oneColumn">
    {[
      {
        content: "<div style='text-align:left'><p><b>Blazingly Fast Numerical Optimization</b>: OpEn combines extremely fast numerical optimization methods (<a href='docs/algorithm'>see details</a>) with Rust - a fast and safe programming language, which is ideal for embedded applications. OpEn implements numerical fast state-of-the-art optimization methods with low memory requirements. Our benchmarks have shown that OpEn can outperform other methods, such as interior point and sequential quadratic/convex programming by 1-2 orders of magnitude. This way, OpEn paves the way for the use of optimization-based methods, such as model predictive control and moving horizon estimation, to highly dynamical nonlinear systems.</div><img width=500 src='img/openbenchmark.png' alt='benchmark results of OpEn; comparison with IPOPT'/>",
        title: "Blazingly Fast"
      }
    ]}
  </Block>

);

const ModelPredictiveControl = props => (
  <Block className="mpcBlock">
    {[
      {
        content: "<div style='text-align:left'><p><a href='https://en.wikipedia.org/wiki/Model_predictive_control' target='_blank'><b>Model Predictive Control</b></a> (MPC) is a powerful optimization-based control methodology. MPC has become the golden standard in control engineering as it can deal with nonlinear dynamics and state/input constraints. At its core, there is an optimization problem that needs to be solved in real time and within the ever so often stringent runtime requirements of modern applications (robotics, aerospace, automotive and more).</p></div><img width=500 src='img/mpc56.png' alt='model predictive control for UAV obstacle avoidance'/><div style='text-align:left'><p>When the system dynamics is nonlinear, or there exist nonconvex constraints (e.g., set avoidance constraints), the MPC optimization problem poses significant challenges towards the implementation and deployment of fast and reliable predictive controllers. This is where OpEn comes in: it offers a toolkit of extremely fast and robust numerical optimization methods, especially tailored for embedded applications where both speed and memory usage are of the essense.</p></div>",
        title: "Model Predictive Control"
      }
    ]}
  </Block>

);

const MovingHorizonEstimation = props => (
  <Block className="oneColumn">
    {[
      {
        content: "<div style='text-align:left'><p><a href='https://en.wikipedia.org/wiki/Moving_horizon_estimation' target='_blank'><b>Moving Horizon Estimation</b></a> (MHE) is the bee's knees of nonlinear estimation: it is an optimization-based estimator for constrained nonlinear systems. MHE is backed by a strong theoretical bedrock that combines Bayesian estimation and dynamic programming; however, its applicability has been hampered by the associated computational burden and has limited its use to slow or linear dynamical systems. OpEn can unlock the huge potential of MHE and facilitate its use in robotics, automotive, aerospace and other applications with high sampling frequencies.</p></div><img width=500 src='img/mhe.png' alt='estimation of the state of Lorenz system with nonlinear moving horizon estimation'/>",
        title: "Moving Horizon Estimation"
      }
    ]}
  </Block>

);



const pre = '```';
const codeExample = `${pre}python
import opengen as og
import casadi.casadi as cs

u = cs.SX.sym("u", 5)                     # -- decision variables
p = cs.SX.sym("p", 2)                     # -- parameters
phi = og.functions.rosenbrock(u, p)       # -- cost
c = 1.5 * cs.cos(u[0]) - u[1]             # -- constraints
bounds = og.constraints.Ball2(None, 1.5)  # -- bounds on u
problem = og.builder.Problem(u, p, phi) \\
    .with_penalty_constraints(c) \\
    .with_constraints(bounds)             # -- construct problem
build_config = og.config.BuildConfiguration()  \\
    .with_build_mode(og.config.BuildConfiguration.DEBUG_MODE)  \\
    .with_tcp_interface_config()          # -- build configuration
meta = og.config.OptimizerMeta()  \\
    .with_optimizer_name("my_optimizer")
solver_config = og.config.SolverConfiguration()  \\
    .with_tolerance(1e-5)
builder = og.builder.OpEnOptimizerBuilder(problem, meta,
                                          build_config,
                                          solver_config)
builder.build()
    `;

class Index extends React.Component {
  render() {
    const language = this.props.language || "en";

    return (
      <div>
        <HomeSplash language={language} />
        <div className="mainContainer">
          <AboutOpen />
          <div className="productShowcaseSection">
              <div>
                  <h3>Easy Code Generation</h3>
                  <p>You can install OpEn in Python using <code>pip</code> (read the <a href="docs/installation">installation instructions</a>) and generate your first optimizer in a few minutes!</p>
                  <img src="img/open-promo.gif" alt="showcase of OpEn functionality in Python via a code generation example"/>
                  <p/>
              </div>
            <Container background="light">
              <FeaturesTop />
              <FeaturesTopTwo />
            </Container>
          </div>
          <Container>
              <SuperFastBlock />
              <ModelPredictiveControl />
              <MovingHorizonEstimation />
          </Container>
        </div>
      </div>
    );
  }
}

module.exports = Index;
