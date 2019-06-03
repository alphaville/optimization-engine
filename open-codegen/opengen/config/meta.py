class OptimizerMeta:
    """Metadata of auto-generated parametric optimizer

    General metadata for the auto-generated optimizer

    The most important piece of information is the name of the
    optimizer. The optimizer will be stored in a namesake folder
    inside the target build directory.

    """

    def __init__(self,
                 optimizer_name='open_optimizer',
                 optimizer_version='0.0.0',
                 optimizer_licence='MIT',
                 optimizer_authors=['John Smith']):
        """Constructor of OptimizerMeta

        Args:
            optimizer_name:     optimizer name (default: <code>"open_optimizer"</code>)
            optimizer_version:  version (default: <code>"0.0.0"</code>)
            optimizer_licence:  licence name or URL (default: <code>"MIT"</code>)
            optimizer_authors:  list of authors, as list of strings (default: <code>["John Smith"]</code>)

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
        self.__optimizer_name = optimizer_name
        self.__optimizer_version = optimizer_version
        self.__optimizer_licence = optimizer_licence
        self.__optimizer_author_list = optimizer_authors

    def with_version(self, optimizer_version):
        """Specify version

        Specify the version of the auto-generated optimizer.

        Args:
            optimizer_version: version of auto-generated optimizer

        Returns:
            The current instance of OptimizerMeta
        """
        self.__optimizer_version = optimizer_version
        return self

    def with_authors(self, optimizer_authors):
        """Specify list of authors

        Args:
            optimizer_authors: list of authors

        Returns:
            The current instance of OptimizerMeta
        """
        self.__optimizer_author_list = optimizer_authors
        return self

    def with_optimizer_name(self, optimizer_name):
        """Specify the name of the optimizer

        Args:
            optimizer_name: name of build

        Returns:
            The current instance of OptimizerMeta
        """
        self.__optimizer_name = optimizer_name
        return self

    def with_licence(self, optimizer_licence):
        """Specify licence of auto-generated code

        Args:
            optimizer_licence: licence name (e.g., MIT) or licence URL

        Returns:
            The current instance of OptimizerMeta
        """
        self.__optimizer_licence = optimizer_licence
        return self

    @property
    def optimizer_name(self):
        """Name of optimizer"""
        return self.__optimizer_name

    @property
    def version(self):
        """Version of optimizer"""
        return self.__optimizer_version

    @property
    def authors(self):
        """List of authors of optimizer"""
        return self.__optimizer_author_list

    @property
    def licence(self):
        """Licence of optimizer"""
        return self.__optimizer_licence
