class OptimizerMeta:

    def __init__(self,
                 build_name='MyOptimizer',
                 version='0.0.0',
                 licence='MIT',
                 authors=['Me', 'You']):
        self._build_name = build_name
        self._version = version
        self._licence = licence
        self._authors = authors

    def with_version(self, version):
        self._version = version
        return self

    def with_authors(self, authors):
        self._authors = authors
        return self

    def with_build_name(self, build_name):
        self._build_name = build_name
        return self

    def with_licence(self, licence):
        self._licence = licence
        return self

    def build_name(self):
        return self._build_name

    def version(self):
        return self._version

    def authors(self):
        return self._authors

    def licence(self):
        return self._licence