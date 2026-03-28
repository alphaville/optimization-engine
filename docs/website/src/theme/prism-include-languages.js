import siteConfig from '@generated/docusaurus.config';

function registerMsgLanguage(PrismObject) {
  PrismObject.languages.msg = {
    comment: {
      pattern: /#.*/,
      greedy: true,
    },
    keyword: /\b(?:bool|byte|char|float32|float64|int8|int16|int32|int64|string|uint8|uint16|uint32|uint64)\b/,
    number: /\b\d+(?:\.\d+)?\b/,
    operator: /=/,
    punctuation: /[[\]]/,
  };
}

export default function prismIncludeLanguages(PrismObject) {
  const {
    themeConfig: {prism},
  } = siteConfig;
  const {additionalLanguages} = prism;

  const PrismBefore = globalThis.Prism;
  globalThis.Prism = PrismObject;

  additionalLanguages.forEach((lang) => {
    if (lang === 'php') {
      require('prismjs/components/prism-markup-templating.js');
    }
    require(`prismjs/components/prism-${lang}`);
  });

  registerMsgLanguage(PrismObject);

  delete globalThis.Prism;
  if (typeof PrismBefore !== 'undefined') {
    globalThis.Prism = PrismObject;
  }
}
