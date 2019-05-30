class BuildConfiguration:
    """Build configuration

    Configuration for the code generator

    """

    def __init__(self, build_dir=None):
        """
        Construct an instance of BuildConfiguration

        Args:
            build_dir: Target directory

        Returns:
            A new instance of BuildConfiguration

        """
        self._target_system = 'default'
        self._build_mode = 'release'
        self._cost_function_name = 'phi'
        self._grad_cost_function_name = 'grad_phi'
        self._constraint_penalty_function = 'constraints_penalty'
        self._rebuild = False
        self._build_dir = build_dir
        self._open_version = '*'

    # ---------- GETTERS ---------------------------------------------

    @property
    def rebuild(self):
        """Whether to re-build the optimizer from scratch"""
        return self._rebuild

    @property
    def cost_function_name(self):
        return self._cost_function_name

    @property
    def grad_function_name(self):
        return self._grad_cost_function_name

    @property
    def constraint_penalty_function_name(self):
        return self._constraint_penalty_function

    @property
    def target_system(self):
        """Target system"""
        return self._target_system

    @property
    def build_mode(self):
        """Build mode (release or debug)"""
        return self._build_mode

    @property
    def build_dir(self):
        """Directory in which the auto-generated optimizer will be stored"""
        return self._build_dir

    @property
    def open_version(self):
        """OpEn version used with the auto-generated solver"""
        return self._open_version

    # ---------- SETTERS ---------------------------------------------

    def with_rebuild(self, do_rebuild):
        """Whether to clean and rebuild the code generator, if it already exists

        Args:
            do_rebuild: if set to True, the target code generator
            will be cleaned and rebuilt from scratch

        Returns:
            The current instance of BuildConfiguration (self)
        """
        self._rebuild = do_rebuild
        return self

    def with_target_system(self, _target_system):
        """Not implemented yet"""
        raise NotImplementedError

    def with_build_mode(self, build_mode):
        """Set the build mode (debug/release)

        Args:
            build_mode: Choose either 'debug' or 'release'; the former is
            fast, but suboptimal, while the later may take a while to compile,
            but the generated binary is significantly faster

        Returns:
            The current instance of  BuildConfiguration (self)

        """
        self._build_mode = build_mode
        return self

    def with_build_directory(self, build_dir):
        """Specify the build directory"""
        self._build_dir = build_dir
        return self

    def with_open_version(self, open_version):
        """Specify the version of OpEn to link to"""
        self._open_version = open_version
        return self
