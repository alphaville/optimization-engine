from opengen.config.tcp_server_config import TcpServerConfiguration
import random
import string


class BuildConfiguration:
    """Build configuration

    Configuration for the code generator

    """

    def __init__(self, build_dir="."):
        """
        Construct an instance of BuildConfiguration

        Args:
            build_dir: Target directory, defaults to the current directory

        Returns:
            A new instance of BuildConfiguration

        """
        random_string = ''.join(random.choice(string.ascii_letters) for i in range(20))

        self.__target_system = None
        self.__build_mode = 'release'
        self.__id = random_string
        self.__cost_function_name = 'phi_' + random_string
        self.__grad_cost_function_name = 'grad_phi_' + random_string
        self.__constraint_penalty_function = 'mapping_f2_' + random_string
        self.__alm_constraints_mapping_f1 = 'mapping_f1_' + random_string
        self.__rebuild = False
        self.__build_dir = build_dir
        self.__open_version = None
        self.__build_c_bindings = False
        self.__tcp_interface_config = None

    # ---------- GETTERS ---------------------------------------------

    @property
    def rebuild(self):
        """Whether to re-build the optimizer from scratch"""
        return self.__rebuild

    @property
    def id(self):
        return self.__id

    @property
    def cost_function_name(self):
        return self.__cost_function_name

    @property
    def grad_function_name(self):
        return self.__grad_cost_function_name

    @property
    def constraint_penalty_function_name(self):
        return self.__constraint_penalty_function

    @property
    def alm_mapping_f1_function_name(self):
        return self.__alm_constraints_mapping_f1

    @property
    def target_system(self):
        """Target system"""
        return self.__target_system

    @property
    def build_mode(self):
        """Build mode (release or debug)"""
        return self.__build_mode

    @property
    def build_dir(self):
        """Directory in which the auto-generated optimizer will be stored"""
        return self.__build_dir

    @property
    def open_version(self):
        """OpEn version used with the auto-generated solver"""
        return self.__open_version

    @property
    def build_c_bindings(self):
        return self.__build_c_bindings

    @property
    def tcp_interface_config(self):
        return self.__tcp_interface_config

    # ---------- SETTERS ---------------------------------------------

    def with_rebuild(self, do_rebuild):
        """Whether to clean and rebuild the code generator, if it already exists

        Args:
            do_rebuild: if set to True, the target code generator
            will be cleaned and rebuilt from scratch

        Returns:
            The current instance of BuildConfiguration (self)
        """
        self.__rebuild = do_rebuild
        return self

    def with_target_system(self, target_system):
        self.__target_system = target_system
        return self

    def with_build_mode(self, build_mode):
        """Set the build mode (debug/release)

        Args:
            build_mode: Choose either 'debug' or 'release'; the former is
            fast, but suboptimal, while the later may take a while to compile,
            but the generated binary is significantly faster

        Returns:
            The current instance of  BuildConfiguration (self)

        """
        self.__build_mode = build_mode
        return self

    def with_build_directory(self, build_dir):
        """Specify the build directory"""
        self.__build_dir = build_dir
        return self

    def with_open_version(self, open_version):
        """Specify the version of OpEn to link to"""
        self.__open_version = open_version
        return self

    def with_build_c_bindings(self, build_c_bindings=True):
        self.__build_c_bindings = build_c_bindings
        return self

    def with_tcp_interface_config(self, tcp_interface_config=TcpServerConfiguration()):
        self.__tcp_interface_config = tcp_interface_config
        return self
