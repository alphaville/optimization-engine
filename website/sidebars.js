module.exports = {
  docsSidebar: [
    {
      type: 'category',
      label: 'OpEn Guide',
      items: ['open-intro', 'installation', 'cite_open'],
    },
    {
      type: 'category',
      label: 'Python',
      items: [
        'python-interface',
        'python-advanced',
        'python-c',
        'python-bindings',
        'python-tcp-ip',
        'python-ros',
        'python-examples',
      ],
    },
    {
      type: 'category',
      label: 'Optimal Control',
      items: ['python-ocp-1', 'python-ocp-2', 'python-ocp-3', 'python-ocp-4'],
    },
    {
      type: 'category',
      label: 'Rust',
      items: ['openrust-basic', 'openrust-alm', 'openrust-features'],
    },
    {
      type: 'category',
      label: 'MATLAB',
      items: ['matlab-interface', 'matlab-examples'],
    },
    {
      type: 'category',
      label: 'Examples',
      items: [
        'example_rosenbrock_py',
        'example_navigation_py',
        'example_estimation_py',
        'example_bnp_py',
        'example_tanks_py',
        'example_invpend_py',
        'example_navigation_ros_codegen',
        'example-nav',
        'example-nmpc',
      ],
    },
    {
      type: 'category',
      label: 'Extras',
      items: ['docker', 'udp-sockets', 'irc', 'algorithm', 'faq', 'contributing'],
    },
  ],
};
