(function () {
  function updateMath() {
    if (!window.MathJax || !window.MathJax.typesetPromise) {
      return;
    }

    window.MathJax.typesetPromise().catch(function () {
      return undefined;
    });
  }

  function initializeLegacyContainers() {
    document.querySelectorAll('.mycontainer').forEach(function (element) {
      if (!element.dataset.legacyReady) {
        element.style.display = 'none';
        element.dataset.legacyReady = 'true';
      }
    });
  }

  function bindLegacyToggleButtons() {
    document
      .querySelectorAll('[data-legacy-toggle-button][data-legacy-toggle-target][data-legacy-toggle-label]')
      .forEach(function (button) {
        if (button.dataset.legacyToggleBound === 'true') {
          return;
        }

        button.addEventListener('click', function () {
          window.toggleCollapseExpand(
            button.dataset.legacyToggleButton,
            button.dataset.legacyToggleTarget,
            button.dataset.legacyToggleLabel,
          );
        });

        button.dataset.legacyToggleBound = 'true';
      });
  }

  function refreshLegacyContent() {
    initializeLegacyContainers();
    bindLegacyToggleButtons();
    window.setTimeout(updateMath, 80);
  }

  function scheduleRefresh() {
    window.setTimeout(refreshLegacyContent, 150);
  }

  window.toggleCollapseExpand = function (buttonId, containerId, label) {
    var container = document.getElementById(containerId);
    var button = document.getElementById(buttonId);

    if (!container || !button) {
      return;
    }

    var isHidden = getComputedStyle(container).display === 'none';
    container.style.display = isHidden ? 'block' : 'none';
    button.innerHTML = isHidden
      ? '<i class="fa fa-angle-up"></i> Collapse ' + label
      : '<i class="fa fa-angle-down"></i> Expand ' + label;

    if (isHidden) {
      updateMath();
    }
  };

  var originalPushState = history.pushState;
  history.pushState = function () {
    var result = originalPushState.apply(this, arguments);
    scheduleRefresh();
    return result;
  };

  var originalReplaceState = history.replaceState;
  history.replaceState = function () {
    var result = originalReplaceState.apply(this, arguments);
    scheduleRefresh();
    return result;
  };

  window.addEventListener('popstate', scheduleRefresh);
  window.addEventListener('hashchange', scheduleRefresh);
  window.addEventListener('load', refreshLegacyContent);

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', refreshLegacyContent);
  } else {
    refreshLegacyContent();
  }
})();
