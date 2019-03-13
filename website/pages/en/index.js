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

function docUrl(doc, language) {
  return `${siteConfig.baseUrl}${language ? `${language}/` : ""}${doc}`;
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
      <img src={"img/docusaurus2.svg"} alt="OpEn logo" width={100} height={100} />
      <h1 className="projectTitle">{siteConfig.title}</h1>
    </div>

    <h2 style={{ marginTop: "0.5em" }}>
      Fast &amp; Accurate Embedded Optimization
    </h2>
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
    const language = this.props.language || "";
    return (
      <SplashContainer>
        <div className="inner">
          <ProjectTitle />
          <PromoSection>
            <Button href={docUrl("docs/open-intro", language)}>
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

const FeaturesTop = props => (
  <Block layout="threeColumn" className="featureBlock">
    {[
      {
        content: "All numerical routines are written in **Rust** -- a **fast** and **safe** programming language, which is ideal for embedded applications.",
        image: imgUrl("microchip.svg"),
        imageAlign: 'top',
        title: "Embeddable"
      },
      {
        content: "**Optimization Engine** can produce solutions of high accuracy thanks to the fast convergence properties of its numerical algorithm (PANOC).",
        image: imgUrl("bullseye.svg"),
        imageAlign: 'top',
        title: "Accurate"
      },
      {
        content: "**Sub-millisecond** fast numerical nonconvex optimization tested on several platforms.",
        image: imgUrl("rocket.svg"),
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
        content: "**OpEn** is **seriously** easy to use! You can call if from Rust, MATLAB, Python and other programming languages - even over the Internet!",
        image: imgUrl("happy.png"),
        imageAlign: 'top',
        title: "User Friendly"
      },
      {
        content: "**OpEn is open**: it is a free, open source, MIT/Apachev2-licensed software with a growing community that actively supports its development. Reach us [here](/optimization-engine/blog/2019/03/06/talk-to-us).",
        image: imgUrl("open.png"),
        imageAlign: 'top',
        title: "Community"
      },
      {
        content: "**Well documented** with lots of examples.",
        image: imgUrl("saturn.png"),
        imageAlign: 'top',
        title: "Documented"
      }
    ]}
  </Block>

);


class Index extends React.Component {
  render() {
    const language = this.props.language || "";

    return (
      <div>
        <HomeSplash language={language} />
        <div className="mainContainer">
          <div className="productShowcaseSection">
            <Container background="light">
              <FeaturesTop />
              <FeaturesTopTwo />
            </Container>
          </div>
        </div>
      </div>
    );
  }
}

module.exports = Index;

