class OptimizerMeta:

    def __init__(self,
                 optimizer_name='MyOptimizer',
                 version='0.0.0',
                 licence='MIT',
                 authors=['Me', 'You']):
        self._optimizer_name = optimizer_name
        self._version = version
        self._licence = licence
        self._authors = authors

    def with_version(self, version):
        self._version = version
        return self

    def with_authors(self, authors):
        self._authors = authors
        return self

    def with_optimizer_name(self, build_name):
        self._optimizer_name = build_name
        return self

    def with_licence(self, licence):
        self._licence = licence
        return self

    def optimizer_name(self):
        return self._optimizer_name

    def version(self):
        return self._version

    def authors(self):
        return self._authors

    def licence(self):
        return self._licence