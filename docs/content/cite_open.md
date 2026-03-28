---
id: cite_open
title: Cite OpEn
sidebar_label: Cite OpEn
description: How to cite OpEn
---

<script>
  ((window.gitter = {}).chat = {}).options = {
    room: 'alphaville/optimization-engine'
  };
</script>
<script src="https://sidecar.gitter.im/dist/sidecar.v1.js" async defer></script>
<script type="text/javascript" src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.1/MathJax.js?config=TeX-AMS-MML_HTMLorMML"></script>
<link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css">
<script type="text/javascript">
function toggleCollapseExpand(buttonId, containerId, theText) {
    conditionsElement = document.getElementById(containerId);
    techhConditionsButtonElement = document.getElementById(buttonId);
    conditionsDisplay = getComputedStyle(conditionsElement, null).display
    if (conditionsDisplay === "none") {
        conditionsElement.style.display = "block";
        techhConditionsButtonElement.innerHTML = '<i class="fa fa-angle-up"></i> Collapse ' + theText;
    } else {
        conditionsElement.style.display = "none";
        techhConditionsButtonElement.innerHTML = '<i class="fa fa-angle-down"></i> Expand ' + theText;
    }
}
</script>

### Main Paper

Please, cite OpEn as follows (read the paper on <a href="https://arxiv.org/abs/2003.00292">arXiv</a>):
```bibtex
@inproceedings{open2020,
  author    = "P. Sopasakis and E. Fresk and P. Patrinos",
  title     = "{OpEn}: Code Generation for Embedded Nonconvex Optimization",
  booktitle = "IFAC World Congress",
  year      = "2020",
  address   = "Berlin, Germany"
}
```

<button onclick="toggleCollapseExpand('videoOpenButton', 'videoOpen', 'Video')" id="videoOpenButton">
  <i class="fa fa-cog fa-spin"></i>
  Show Video
</button>

<div class="mycontainer" id="videoOpen">
<iframe width='600'
	height='315'
	src='https://www.youtube.com/embed/bHZ6eyhj3LM'
	title='YouTube video player'
	frameborder='0'
	allow='accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture'
	allowfullscreen></iframe>
</div>
<br/><br/>

Cite the original PANOC paper as follows (read on <a href="https://arxiv.org/abs/1709.06487">arXiv</a>):
```bibtex
@inproceedings{panoc2017,
  author    = "Stella, L. and Themelis, A. and Sopasakis, P. and Patrinos, P.",
  title     = "A simple and efficient algorithm for
               nonlinear model predictive control",
  booktitle = "IEEE Conference on Decision and Control (CDC)",
  year      = "2017",
  month     = "Dec",
  pages     = "1939-1944"
}
```


### Application papers

You may also cite the following papers where we use PANOC in lab experiments - first, for obstacle avoidance on an autonomous ground vehicle carrying a trailer:

```bibtex
@inproceedings{agv2018,
  author    = "Sathya, A. and Sopasakis, P. and
               Van Parys, R. and Themelis, A. and
               Pipeleers, G. and Patrinos, P.",
  title     = "Embedded nonlinear model predictive control for
               obstacle avoidance using PANOC",
  booktitle = "European Control Conference (ECC)",
  year      = "2018",
  month     = "June",
  pages     = "1523-1528"
}
```

```bibtex
@inproceedings{sina:ifac20,
  author    = "Mansouri, S. S. and Kanellakis, C. and
               Fresk, E. and Lindqvist, B. and
               Kominiak, D. and Koval, A. and
               Sopasakis, P. and Nikolakopoulos, G.",
  title     = "Subterranean MAV Navigation based on Nonlinear
               {MPC} with Collision Avoidance Constraints",
  booktitle = "IFAC World Congress",
  year      = "2020",
  address   = "Berlin, Germany"
}
```

<button onclick="toggleCollapseExpand('videoSubterraneanButton', 'videoSubterranean', 'Video')" id="videoSubterraneanButton">
  <i class="fa fa-cog fa-spin"></i>
  Show Video
</button>

<div class="mycontainer" id="videoSubterranean">
<iframe width="560" height="315" src="https://www.youtube.com/embed/-MP4Sn6Q1uo" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>
<br/><br/>

and our recent work on obstacle avoidance of an autonomous micro-aerial vehicle (read on <a href="https://arxiv.org/abs/1812.04755">arXiv</a>):

```bibtex
@inproceedings{mav2019,
  author    = "Small, E. and Sopasakis, P. and Fresk, E. and
               Patrinos, P. and Nikolakopoulos, G.",
  title     = "Aerial navigation in obstructed environments with
               embedded nonlinear model predictive control",
  booktitle = "European Control Conference (ECC)",
  year      = "2019",
  month     = "June",
  pages     = "3556-3563"
}
```

<button onclick="toggleCollapseExpand('videoAerialObstructedButton', 'videoAerialObstructed', 'Video')" id="videoAerialObstructedButton">
  <i class="fa fa-cog fa-spin"></i>
  Show Video
</button>

<div class="mycontainer" id="videoAerialObstructed">
<iframe width="560" height="315" src="https://www.youtube.com/embed/E4vCSJw97FQ" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>
<br/><br/>

and our work for controlling multiple UAVs (read on <a href="https://arxiv.org/abs/2104.03783">arXiv</a>):

```bibtex
@inproceedings{iros2021,
  author    = "Lindqvist, B. and Sopasakis, P. and Nikolakopoulos, G.",
  title     = "A Scalable Distributed Collision Avoidance Scheme
               for Multi-agent {UAV} systems",
  booktitle = "Int Conf on Intelligent Robots and Systems (IROS)",
  year      = "2021",
  month     = "Sep-Oct"
}
```

<button onclick="toggleCollapseExpand('videoMultiAgentButton', 'videoMultiAgent', 'Video')" id="videoMultiAgentButton">
  <i class="fa fa-cog fa-spin"></i>
  Show Video
</button>

<div class="mycontainer" id="videoMultiAgent">
<iframe width="560" height="315" src="https://www.youtube.com/embed/3kyiL6MZaag" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>

### Papers using OpEn (Other authors)

OpEn is used in an NMPC for controlling a small-scale car (<a href="https://giuseppesilano.net/publications/SMC22.pdf">preprint</a>)

```bibtex
@inproceedings{smc2022,
  author    = "Cataffo, V. and Silano, G. and Iannelli, L. and Puig, V. and Glielmo, L.",
  title     = "A Nonlinear Model Predictive Control Strategy 
                for Autonomous Racing of Scale Vehicles",
  booktitle = "IEEE International Conference on Systems, Man and Cybernetics (SMC)",
  year      = "2022",
  month     = "Oct",
  preprint  = "https://giuseppesilano.net/publications/SMC22.pdf"
}
```

<button onclick="toggleCollapseExpand('videoRacingButton', 'videoRacing', 'Video')" id="videoRacingButton">
  <i class="fa fa-cog fa-spin"></i>
  Show Video
</button>

<div class="mycontainer" id="videoRacing">
<iframe width="560" height="315" src="https://www.youtube.com/embed/w5c328rQmX4" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>
</div>
