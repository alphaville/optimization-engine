#!/bin/bash
GIT_USER=alphaville \
  CURRENT_BRANCH=master \
  USE_SSH=true \
  yarn run publish-gh-pages
