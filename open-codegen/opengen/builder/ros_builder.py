import opengen.definitions as og_dfn

import datetime
import logging
import os
import shutil

import jinja2


def make_dir_if_not_exists(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


def get_ros_template(template_subdir, name):
    file_loader = jinja2.FileSystemLoader(og_dfn.templates_subdir(template_subdir))
    env = jinja2.Environment(loader=file_loader, autoescape=True)
    return env.get_template(name)


class _BaseRosBuilder:
    """
    Shared code generation for ROS-related packages

    For internal use
    """

    _template_subdir = None
    _logger_name = None
    _logger_tag = None
    _launch_file_name = None

    def __init__(self, meta, build_config, solver_config):
        self._meta = meta
        self._build_config = build_config
        self._solver_config = solver_config
        self._logger = logging.getLogger(self._logger_name)
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(1)
        c_format = logging.Formatter(
            f'[%(levelname)s] <<{self._logger_tag}>> %(message)s')
        stream_handler.setFormatter(c_format)
        self._logger.setLevel(1)
        self._logger.handlers.clear()
        self._logger.addHandler(stream_handler)
        self._logger.propagate = False

    @property
    def _ros_config(self):
        raise NotImplementedError

    def _template(self, name):
        return get_ros_template(self._template_subdir, name)

    def _target_dir(self):
        return os.path.abspath(
            os.path.join(
                self._build_config.build_dir,
                self._meta.optimizer_name))

    def _ros_target_dir(self):
        return os.path.abspath(
            os.path.join(
                self._build_config.build_dir,
                self._meta.optimizer_name,
                self._ros_config.package_name))

    def _generate_ros_dir_structure(self):
        self._logger.info("Generating directory structure")
        target_ros_dir = self._ros_target_dir()
        make_dir_if_not_exists(target_ros_dir)
        for directory_name in ('include', 'extern_lib', 'src', 'msg', 'config', 'launch'):
            make_dir_if_not_exists(os.path.abspath(
                os.path.join(target_ros_dir, directory_name)))

    def _generate_ros_package_xml(self):
        self._logger.info("Generating package.xml")
        target_ros_dir = self._ros_target_dir()
        template = self._template('package.xml')
        output_template = template.render(meta=self._meta, ros=self._ros_config)
        target_rospkg_path = os.path.join(target_ros_dir, "package.xml")
        with open(target_rospkg_path, "w") as fh:
            fh.write(output_template)

    def _generate_ros_cmakelists(self):
        self._logger.info("Generating CMakeLists")
        target_ros_dir = self._ros_target_dir()
        template = self._template('CMakeLists.txt')
        output_template = template.render(meta=self._meta, ros=self._ros_config)
        target_rospkg_path = os.path.join(target_ros_dir, "CMakeLists.txt")
        with open(target_rospkg_path, "w") as fh:
            fh.write(output_template)

    def _copy_ros_files(self):
        self._logger.info("Copying external dependencies")
        target_ros_dir = self._ros_target_dir()

        header_file_name = self._meta.optimizer_name + '_bindings.hpp'
        target_include_filename = os.path.abspath(
            os.path.join(target_ros_dir, 'include', header_file_name))
        original_include_file = os.path.abspath(
            os.path.join(self._target_dir(), header_file_name))
        shutil.copyfile(original_include_file, target_include_filename)

        lib_file_name = 'lib' + self._meta.optimizer_name + '.a'
        target_lib_file_name = os.path.abspath(
            os.path.join(target_ros_dir, 'extern_lib', lib_file_name))
        original_lib_file = os.path.abspath(
            os.path.join(
                self._target_dir(),
                'target',
                self._build_config.build_mode,
                lib_file_name))
        shutil.copyfile(original_lib_file, target_lib_file_name)

        for message_name in ('OptimizationParameters.msg', 'OptimizationResult.msg'):
            original_message = os.path.abspath(
                os.path.join(
                    og_dfn.templates_dir(),
                    self._template_subdir,
                    message_name))
            target_message = os.path.abspath(
                os.path.join(target_ros_dir, 'msg', message_name))
            shutil.copyfile(original_message, target_message)

    def _generate_ros_params_file(self):
        self._logger.info("Generating open_params.yaml")
        target_ros_dir = self._ros_target_dir()
        template = self._template('open_params.yaml')
        output_template = template.render(meta=self._meta, ros=self._ros_config)
        target_yaml_fname = os.path.join(target_ros_dir, "config", "open_params.yaml")
        with open(target_yaml_fname, "w") as fh:
            fh.write(output_template)

    def _generate_ros_node_header(self):
        self._logger.info("Generating open_optimizer.hpp")
        target_ros_dir = self._ros_target_dir()
        template = self._template('open_optimizer.hpp')
        output_template = template.render(
            meta=self._meta,
            ros=self._ros_config,
            solver_config=self._solver_config)
        target_rosnode_header_path = os.path.join(
            target_ros_dir, "include", "open_optimizer.hpp")
        with open(target_rosnode_header_path, "w") as fh:
            fh.write(output_template)

    def _generate_ros_node_cpp(self):
        self._logger.info("Generating open_optimizer.cpp")
        target_ros_dir = self._ros_target_dir()
        template = self._template('open_optimizer.cpp')
        output_template = template.render(
            meta=self._meta,
            ros=self._ros_config,
            timestamp_created=datetime.datetime.now())
        target_rosnode_cpp_path = os.path.join(target_ros_dir, "src", "open_optimizer.cpp")
        with open(target_rosnode_cpp_path, "w") as fh:
            fh.write(output_template)

    def _generate_ros_launch_file(self):
        self._logger.info("Generating %s", self._launch_file_name)
        target_ros_dir = self._ros_target_dir()
        template = self._template(self._launch_file_name)
        output_template = template.render(meta=self._meta, ros=self._ros_config)
        target_rosnode_launch_path = os.path.join(
            target_ros_dir, "launch", self._launch_file_name)
        with open(target_rosnode_launch_path, "w") as fh:
            fh.write(output_template)

    def _generate_ros_readme_file(self):
        self._logger.info("Generating README.md")
        target_ros_dir = self._ros_target_dir()
        template = self._template('README.md')
        output_template = template.render(ros=self._ros_config)
        target_readme_path = os.path.join(target_ros_dir, "README.md")
        with open(target_readme_path, "w") as fh:
            fh.write(output_template)

    def _symbolic_link_info_message(self):
        raise NotImplementedError

    def build(self):
        """
        Build ROS-related files
        """
        self._generate_ros_dir_structure()
        self._generate_ros_package_xml()
        self._generate_ros_cmakelists()
        self._copy_ros_files()
        self._generate_ros_params_file()
        self._generate_ros_node_header()
        self._generate_ros_node_cpp()
        self._generate_ros_launch_file()
        self._generate_ros_readme_file()
        self._symbolic_link_info_message()


class RosBuilder(_BaseRosBuilder):
    """
    Code generation for ROS-related files

    For internal use
    """

    _template_subdir = 'ros'
    _logger_name = 'opengen.builder.RosBuilder'
    _logger_tag = 'ROS'
    _launch_file_name = 'open_optimizer.launch'

    @property
    def _ros_config(self):
        return self._build_config.ros_config

    def _symbolic_link_info_message(self):
        target_ros_dir = self._ros_target_dir()
        self._logger.info("ROS package was built successfully. Now run:")
        self._logger.info("ln -s %s  ~/catkin_ws/src/", target_ros_dir)
        self._logger.info("cd ~/catkin_ws/; catkin_make")


class ROS2Builder(_BaseRosBuilder):
    """
    Code generation for ROS2-related files

    For internal use
    """

    _template_subdir = 'ros2'
    _logger_name = 'opengen.builder.ROS2Builder'
    _logger_tag = 'ROS2'
    _launch_file_name = 'open_optimizer.launch.py'

    @property
    def _ros_config(self):
        return self._build_config.ros2_config

    def _symbolic_link_info_message(self):
        target_ros_dir = self._ros_target_dir()
        self._logger.info("ROS2 package was built successfully. Now run:")
        self._logger.info("ln -s %s  ~/ros2_ws/src/", target_ros_dir)
        self._logger.info("cd ~/ros2_ws/; colcon build --packages-select %s",
                          self._ros_config.package_name)
