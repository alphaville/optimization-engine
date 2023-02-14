from opengen.config.tcp_server_config import TcpServerConfiguration
from opengen.config.ros_config import RosConfiguration
import random
import string
from enum import Enum


class RustAllocator(Enum):
    """Memory Allocator for the auto-generated project"""

    DefaultAllocator = 0
    """Default allocator"""

    JemAlloc = 1
    """Memory allocator: `jemalloc`
    
    Jemalloc is a generic implementation of malloc that emphasises fragmentation
    avoidance
    """

    RpAlloc = 2
    """Memory allocator: `rpmalloc`
    
    Rpmalloc is a very efficient lock-free thread caching 16-byte aligned 
    memory allocator implemented in C.
    """


class BuildConfiguration:
    """Build configuration

    Configuration for the code generator

    """

    #: Debug mode (fast compilation, worse performance)
    DEBUG_MODE = "debug"

    #: Release mode (great performance, very slow compilation)
    RELEASE_MODE = "release"

    def __init__(self, build_dir="."):
        """
        Construct an instance of BuildConfiguration

        :param build_dir: Target directory, defaults to the current directory

        :return: A new instance of BuildConfiguration

        """

        self.__target_system = None
        self.__build_mode = BuildConfiguration.RELEASE_MODE
        self.__rebuild = False
        self.__build_dir = build_dir
        self.__open_version = None
        self.__build_c_bindings = False
        self.__build_python_bindings = False
        self.__ros_config = None
        self.__tcp_interface_config = None
        self.__local_path = None
        self.__allocator = RustAllocator.DefaultAllocator

    # ---------- GETTERS ---------------------------------------------

    @property
    def rebuild(self):
        """Whether to re-build the optimizer from scratch"""
        return self.__rebuild

    @property
    def target_system(self):
        """Target system

        See also: :class:`~opengen.config.build_config.BuildConfiguration.with_target_system`
        """
        return self.__target_system

    @property
    def build_mode(self):
        """
        Build mode (:class:`~opengen.config.build_config.BuildConfiguration.RELEASE_MODE` 
        or :class:`~opengen.config.build_config.BuildConfiguration.DEBUG_MODE`)
        """
        return self.__build_mode

    @property
    def build_dir(self):
        """Directory in which the auto-generated optimizer will be stored"""
        return self.__build_dir

    @property
    def open_version(self):
        """
        OpEn version used with the auto-generated solver

        :return: The method returns either a specific version of OpEn,
            which will be used with the auto-generated optimizer, or `None`,
            in which case, the latest version will be used. You may set your
            preferred version of OpEn with `with_open_version`
        """
        return self.__open_version

    @property
    def local_path(self):
        """Local path of OpEn (if any)"""
        return self.__local_path

    @property
    def build_c_bindings(self):
        """
        Whether to build C bindings 
        """
        return self.__build_c_bindings

    @property
    def build_python_bindings(self):
        """
        Whether to build Python bindings
        """
        return self.__build_python_bindings

    @property
    def tcp_interface_config(self):
        """
        Whether to build a TCP interface
        """
        return self.__tcp_interface_config

    @property
    def ros_config(self) -> RosConfiguration:
        """ROS package configuration

        :return: instance of RosConfiguration
        """
        return self.__ros_config

    @property
    def allocator(self) -> RustAllocator:
        """
        Memory allocator for generated Rust solver
        """
        return self.__allocator

    # ---------- SETTERS ---------------------------------------------

    def with_rebuild(self, do_rebuild):
        """Whether to clean and rebuild the code generator, if it already exists

        :param do_rebuild: if set to True, the target code generator
            will be cleaned and rebuilt from scratch

        :return: current instance of BuildConfiguration
        """
        self.__rebuild = do_rebuild
        return self

    def with_target_system(self, target_system):
        """
        Specify the target system

        :param target_system: target system as string (e.g., use
            "arm-unknown-linux-gnueabihf" or "rpi" for Raspberry Pi).
            Note that you must have installed the target using `rustup`
            if you need to cross-compile.

        :return: current instance of BuildConfiguration
        """
        if target_system.lower() == "rpi":
            self.__target_system = "arm-unknown-linux-gnueabihf"
        else:
            self.__target_system = target_system
        return self

    def with_build_mode(self, build_mode):
        """
        Set the build mode (debug/release)

        :param build_mode: Choose either 'debug' or 'release'; the former is
            fast, but suboptimal, while the latter may take a while to compile,
            but the generated binary is significantly faster

        :return: current instance of BuildConfiguration

        """
        self.__build_mode = build_mode
        return self

    def with_build_directory(self, build_dir):
        """
        Specify the build directory

        :param build_dir: build directory as string

        :return: current instance of BuildConfiguration
        """
        self.__build_dir = build_dir
        return self

    def with_open_version(self, open_version="*", local_path=None):
        """
        Specify the version of OpEn to link to

        :param open_version: version of OpEn (in case you want to
            compile with an older version of OpEn; if not, the latest
            version of OpEn will be used)

        :param local_path: you can compile using a local version
            of OpEn. In that case, you need to provide the full absolute
            path to that local OpEn directory. This option is intended for 
            developers.

        :return: current instance of BuildConfiguration
        """
        self.__open_version = open_version
        self.__local_path = local_path
        return self

    def with_build_c_bindings(self, build_c_bindings=True):
        """
        If activated, OpEn will generate C/C++ bindings for the
        auto-generated solver

        :param build_c_bindings: whether to build C/C++ bindings for
            auto-generated solver; default: `True`, i.e., it suffices
            to call `build_config.with_build_c_bindings()` instead of
            `build_config.with_build_c_bindings(True)`

        :return: current instance of BuildConfiguration
        """
        self.__build_c_bindings = build_c_bindings
        return self

    def with_build_python_bindings(self, build_python_bindings=True):
        """
        If activated, OpEn will generate python bindings for the
        auto-generated solver

        :param build_python_bindings: whether to build python bindings for
            auto-generated solver; default: `True`, i.e., it suffices
            to call `build_config.with_build_python_bindings()` instead of
            `build_config.with_build_python_bindings(True)`

        :return: current instance of BuildConfiguration
        """
        self.__build_python_bindings = build_python_bindings
        return self

    def with_ros(self, ros_config: RosConfiguration):
        """
        Activates the generation of a ROS package. The caller must provide an
        instance of RosConfiguration

        :param ros_config: Configuation of ROS package

        :return: current instance of BuildConfiguration
        """
        self.__build_c_bindings = True  # no C++ bindings, no ROS package mate
        self.__ros_config = ros_config
        return self

    def with_tcp_interface_config(self, tcp_interface_config=TcpServerConfiguration()):
        """
        Specify a TCP server configuration object

        :param tcp_interface_config: Custom TCP server configuration

        :return: current instance of BuildConfiguration
        """
        self.__tcp_interface_config = tcp_interface_config
        return self

    def with_allocator(self, allocator: RustAllocator):
        """Specify a Rust memory allocator. 

        With this method the user can choose an alternative memory allocator
        such as `jemalloc` and `rpalloc`. For example, if you choose `jemalloc`
        as your memory allocator, the autogenerated project will have a 
        `Cargo.toml` file where `optimization-engine` is loaded as a dependency
        with the "jem" feature.

        :param allocator: allocator; instance of `RustAllocator`

        :return: current instance of BuildConfiguration
        """
        self.__allocator = allocator
        return self

    def to_dict(self):
        build_dict = {
            "target_system": self.__target_system,
            "build_mode": self.__build_mode,
            "rebuild": self.__rebuild,
            "build_directory": self.__build_dir,
            "open_version": self.__open_version,
            "build_c_bindings": self.__build_c_bindings,
            "build_python_bindings": self.__build_python_bindings,
        }
        if self.__tcp_interface_config is not None:
            build_dict["tcp_interface_config"] = self.__tcp_interface_config.to_dict()
        if self.__ros_config is not None:
            build_dict["ros_config"] = self.__ros_config.to_dict()
        return build_dict
