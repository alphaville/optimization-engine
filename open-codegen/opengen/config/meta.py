class OptimizerMeta:
    """Metadata of auto-generated parametric optimizer

    General metadata for the auto-generated optimizer

    The most important piece of information is the name of the
    optimizer. The optimizer will be stored in a namesake folder
    inside the target build directory.

    """

    def __init__(self,
                 optimizer_name_='open_optimizer',
                 version_='0.0.0',
                 licence_='MIT',
                 authors_=['John Smith']):
        """Constructor of OptimizerMeta

        Args:
            optimizer_name_: Optimizer name (default: "open_optimizer")
            version_:        version (default: "0.0.0")
            licence_:        licence name or URL (default: "MIT")
            authors_:        List of authors, as list of strings (default: ["John Smith"])

        Returns:
            New instance of OptimizerMeta

        Examples:
            >>> import opengen as og
            >>> meta = og.config.OptimizerMeta()                \
            >>>     .with_version("0.0.2")                      \
            >>>     .with_authors(["P. Sopasakis", "E. Fresk"]) \
            >>>     .with_licence("CC4.0-By")                   \
            >>>     .with_optimizer_name("wow_optimizer")

        """
        self._optimizer_name = optimizer_name_
        self._version = version_
        self._licence = licence_
        self._authors = authors_

    def with_version(self, version_):
        """Specify version

        Specify the version of the auto-generated optimizer.

        Args:
            version_: version

        Returns:
            The current instance of OptimizerMeta
        """
        self._version = version_
        return self

    def with_authors(self, authors_):
        """Specify list of authors

        Args:
            authors_: list of authors

        Returns:
            The current instance of OptimizerMeta
        """
        self._authors = authors_
        return self

    def with_optimizer_name(self, build_name):
        """Specify the name of the optimizer

        Args:
            build_name: name of build

        Returns:
            The current instance of OptimizerMeta
        """
        self._optimizer_name = build_name
        return self

    def with_licence(self, licence):
        """Specify licence of auto-generated code

        Args:
            licence: licence name (e.g., MIT) or licence URL

        Returns:
            The current instance of OptimizerMeta
        """
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
