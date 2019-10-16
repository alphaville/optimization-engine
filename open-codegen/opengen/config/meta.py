import re # regular expressions


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

        :param optimizer_name:     optimizer name (default: `open_optimizer`)
        :param optimizer_version:  version (default: `0.0.0`)
        :param optimizer_licence:  licence name or URL (default: `MIT`)
        :param optimizer_authors:  list of authors, as list of strings (default: `["John Smith"]`)

        :returns: The current instance of OptimizerMeta

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

        :param optimizer_version: version of auto-generated optimizer

        :returns: The current instance of OptimizerMeta
        """
        self.__optimizer_version = optimizer_version
        return self

    def with_authors(self, optimizer_authors):
        """Specify list of authors

        :param optimizer_authors: list of authors

        :returns: The current instance of OptimizerMeta
        """
        self.__optimizer_author_list = optimizer_authors
        return self

    def with_optimizer_name(self, optimizer_name):
        """Specify the name of the optimizer

        :param optimizer_name: name of build, may only contain letters,
        numbers and underscores, and may not start with a number

        :returns: The current instance of OptimizerMeta
        """

        if re.match(r"^[a-zA-Z_]+[\w]*$", optimizer_name):
            self.__optimizer_name = optimizer_name
            return self
        else:
            raise ValueError("invalid optimizer name")

    def with_licence(self, optimizer_licence):
        """Specify licence of auto-generated code

        :param optimizer_licence: licence name (e.g., MIT) or licence URL

        :returns: The current instance of OptimizerMeta
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
