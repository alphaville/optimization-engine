/**
 * Copyright (c) 2017-present, Facebook, Inc.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

const React = require('react');

class Footer extends React.Component {
  docUrl(doc, language) {
    const baseUrl = this.props.config.baseUrl;
    const docsUrl = this.props.config.docsUrl;
    const docsPart = `${docsUrl ? `${docsUrl}/` : ''}`;
    const langPart = `${language ? `${language}/` : ''}`;
    return `${baseUrl}${docsPart}${langPart}${doc}`;
  }

  pageUrl(doc, language) {
    const baseUrl = this.props.config.baseUrl;
    return baseUrl + (language ? `${language}/` : '') + doc;
  }

  render() {
    return (
      <footer className="nav-footer" id="footer">
        <section className="sitemap">
          <a href={this.props.config.baseUrl} className="nav-home">
            {this.props.config.footerIcon && (
              <img
                src={this.props.config.baseUrl + this.props.config.footerIcon}
                alt={this.props.config.title}
                width="66"
                height="58"
              />
            )}
          </a>
          <div>
            <h5>Docs</h5>
            <a href={this.docUrl('open-intro.html')}>
              Getting Started
            </a>
            <a href={this.docUrl('python-interface.html')}>
              Python interface
            </a>
            <a href={this.docUrl('matlab-interface.html')}>
              MATLAB interface
            </a>
            <a href={this.docUrl('docker')}>
              Docker
            </a>
          </div>
          <div>
            <h5>Community</h5>
            <a href={this.pageUrl('users.html')}>
              User Showcase
            </a>
            <a href="https://discord.gg/mfYpn4V" target="_blank" rel="noreferrer noopener">Discord community</a>
            <a href="https://gitter.im/alphaville/optimization-engine" target="_blank" rel="noreferrer noopener">Chat on Gitter</a>
            <a
              href="https://twitter.com/isToxic"
              target="_blank"
              rel="noreferrer noopener">
              Twitter
            </a>
          </div>
          <div>
            <h5>More</h5>
            <a href={`${this.props.config.baseUrl}blog`}>Blog</a>
            <a href="https://github.com/alphaville/optimization-engine" target="_blank">GitHub</a>
            <a href="https://www.openhub.net/p/optimization-engine" target="_blank">Openhub</a>
            <a
              className="github-button"
              href={this.props.config.repoUrl}
              data-icon="octicon-star"
              data-count-href="/alphaville/optimization-engine/stargazers"
              data-show-count="true"
              data-count-aria-label="# stargazers on GitHub"
              aria-label="Star this project on GitHub">
              Star
            </a>
            <br />
            <br />
            <a href="https://twitter.com/share?ref_src=twsrc%5Etfw&via=isToxic"
              className="twitter-share-button"
              data-show-count="false"
              data-text="Fast and accurate embedded nonconvex optimization with #OptimizationEngine">Tweet</a>
            <script async src="https://platform.twitter.com/widgets.js" charSet="utf-8"></script>
          </div>
        </section>


        <section className="copyright">{this.props.config.copyright}</section>
	<section className="copyright"><div>Box Icon made by <a href="https://www.flaticon.com/authors/freepik" title="Freepik">Freepik</a> from <a href="https://www.flaticon.com/" title="Flaticon">www.flaticon.com</a></div></section>
      </footer>
    );
  }
}

module.exports = Footer;
