import opengen.definitions as og_dfn

import os
import logging
import jinja2
import shutil
import datetime

_ROS_PREFIX = 'ros_node_'


def make_dir_if_not_exists(directory):
    if not os.path.exists(directory):
        os.makedirs(directory)


def get_template(name):
    file_loader = jinja2.FileSystemLoader(og_dfn.templates_dir())
    env = jinja2.Environment(loader=file_loader, autoescape=True)
    return env.get_template(name)


def get_ros_template(name):
    file_loader = jinja2.FileSystemLoader(og_dfn.templates_subdir('ros'))
    env = jinja2.Environment(loader=file_loader, autoescape=True)
    return env.get_template(name)


class RosBuilder:
    """
    Code generation for ROS-related files

    For internal use
    """

    def __init__(self, meta, build_config, solver_config):
        self.__meta = meta
        self.__build_config = build_config
        self.__solver_config = solver_config
        self.__logger = logging.getLogger('opengen.builder.RosBuilder')
        stream_handler = logging.StreamHandler()
        stream_handler.setLevel(1)
        c_format = logging.Formatter('[%(levelname)s] <<ROS>> %(message)s')
        stream_handler.setFormatter(c_format)
        self.__logger.setLevel(1)
        self.__logger.addHandler(stream_handler)

    def __target_dir(self):
        return os.path.abspath(
            os.path.join(
                self.__build_config.build_dir,
                self.__meta.optimizer_name))

    def __ros_target_dir(self):
        ros_config = self.__build_config.ros_config
        ros_target_dir_name = ros_config.package_name
        return os.path.abspath(
            os.path.join(
                self.__build_config.build_dir,
                self.__meta.optimizer_name, ros_target_dir_name))

    def __generate_ros_dir_structure(self):
        self.__logger.info("Generating directory structure")
        target_ros_dir = self.__ros_target_dir()
        make_dir_if_not_exists(target_ros_dir)
        make_dir_if_not_exists(os.path.abspath(
            os.path.join(target_ros_dir, 'include')))
        make_dir_if_not_exists(os.path.abspath(
            os.path.join(target_ros_dir, 'extern_lib')))
        make_dir_if_not_exists(os.path.abspath(
            os.path.join(target_ros_dir, 'src')))
        make_dir_if_not_exists(os.path.abspath(
            os.path.join(target_ros_dir, 'msg')))
        make_dir_if_not_exists(os.path.abspath(
            os.path.join(target_ros_dir, 'config')))
        make_dir_if_not_exists(os.path.abspath(
            os.path.join(target_ros_dir, 'launch')))

    def __generate_ros_package_xml(self):
        self.__logger.info("Generating package.xml")
        target_ros_dir = self.__ros_target_dir()
        template = get_ros_template('package.xml')
        output_template = template.render(
            meta=self.__meta, ros=self.__build_config.ros_config)
        target_rospkg_path = os.path.join(target_ros_dir, "package.xml")
        with open(target_rospkg_path, "w") as fh:
            fh.write(output_template)

    def __generate_ros_cmakelists(self):
        self.__logger.info("Generating CMakeLists")
        target_ros_dir = self.__ros_target_dir()
        template = get_ros_template('CMakeLists.txt')
        output_template = template.render(meta=self.__meta,
                                          ros=self.__build_config.ros_config)
        target_rospkg_path = os.path.join(target_ros_dir, "CMakeLists.txt")
        with open(target_rospkg_path, "w") as fh:
            fh.write(output_template)

    def __copy__ros_files(self):
        self.__logger.info("Copying external dependencies")
        # 1. --- copy header file
        target_ros_dir = self.__ros_target_dir()
        header_file_name = self.__meta.optimizer_name + '_bindings.hpp'
        target_include_filename = os.path.abspath(
            os.path.join(
                target_ros_dir, 'include', header_file_name))
        original_include_file = os.path.abspath(
            os.path.join(self.__target_dir(), header_file_name))
        shutil.copyfile(original_include_file, target_include_filename)

        # 2. --- copy library file
        lib_file_name = 'lib' + self.__meta.optimizer_name + '.a'
        target_lib_file_name = \
            os.path.abspath(
                os.path.join(
                    target_ros_dir, 'extern_lib', lib_file_name))
        original_lib_file = os.path.abspath(
            os.path.join(
                self.__target_dir(),
                'target',
                self.__build_config.build_mode,
                lib_file_name))
        shutil.copyfile(original_lib_file, target_lib_file_name)

        # 3. --- copy msg file OptimizationParameters.msg
        original_params_msg = os.path.abspath(
            os.path.join(
                og_dfn.templates_dir(), 'ros', 'OptimizationParameters.msg'))
        target_params_msg = \
            os.path.abspath(
                os.path.join(
                    target_ros_dir, 'msg', 'OptimizationParameters.msg'))
        shutil.copyfile(original_params_msg, target_params_msg)

        # 4. --- copy msg file OptimizationResult.msg
        original_result_msg = os.path.abspath(
            os.path.join(
                og_dfn.templates_dir(), 'ros', 'OptimizationResult.msg'))
        target_result_msg = \
            os.path.abspath(
                os.path.join(
                    target_ros_dir, 'msg', 'OptimizationResult.msg'))
        shutil.copyfile(original_result_msg, target_result_msg)

    def __generate_ros_params_file(self):
        self.__logger.info("Generating open_params.yaml")
        target_ros_dir = self.__ros_target_dir()
        template = get_ros_template('open_params.yaml')
        output_template = template.render(meta=self.__meta,
                                          ros=self.__build_config.ros_config)
        target_yaml_fname \
            = os.path.join(target_ros_dir, "config", "open_params.yaml")
        with open(target_yaml_fname, "w") as fh:
            fh.write(output_template)

    def __generate_ros_node_header(self):
        self.__logger.info("Generating open_optimizer.hpp")
        target_ros_dir = self.__ros_target_dir()
        template = get_ros_template('open_optimizer.hpp')
        output_template = template.render(meta=self.__meta,
                                          ros=self.__build_config.ros_config,
                                          solver_config=self.__solver_config)
        target_rosnode_header_path \
            = os.path.join(target_ros_dir, "include", "open_optimizer.hpp")
        with open(target_rosnode_header_path, "w") as fh:
            fh.write(output_template)

    def __generate_ros_node_cpp(self):
        self.__logger.info("Generating open_optimizer.cpp")
        target_ros_dir = self.__ros_target_dir()
        template = get_ros_template('open_optimizer.cpp')
        output_template = template.render(meta=self.__meta,
                                          ros=self.__build_config.ros_config,
                                          timestamp_created=datetime.datetime.now())
        target_rosnode_cpp_path \
            = os.path.join(target_ros_dir, "src", "open_optimizer.cpp")
        with open(target_rosnode_cpp_path, "w") as fh:
            fh.write(output_template)

    def __generate_ros_launch_file(self):
        self.__logger.info("Generating open_optimizer.launch")
        target_ros_dir = self.__ros_target_dir()
        template = get_ros_template('open_optimizer.launch')
        output_template = template.render(meta=self.__meta,
                                          ros=self.__build_config.ros_config)
        target_rosnode_launch_path \
            = os.path.join(target_ros_dir, "launch", "open_optimizer.launch")
        with open(target_rosnode_launch_path, "w") as fh:
            fh.write(output_template)

    def __generate_ros_readme_file(self):
        self.__logger.info("Generating README.md")
        target_ros_dir = self.__ros_target_dir()
        template = get_ros_template('README.md')
        output_template = template.render(
            ros=self.__build_config.ros_config)
        target_readme_path \
            = os.path.join(target_ros_dir, "README.md")
        with open(target_readme_path, "w") as fh:
            fh.write(output_template)

    def __symbolic_link_info_message(self):
        target_ros_dir = self.__ros_target_dir()
        self.__logger.info("ROS package was built successfully. Now run:")
        self.__logger.info("ln -s %s  ~/catkin_ws/src/", target_ros_dir)
        self.__logger.info("cd ~/catkin_ws/; catkin_make")

    def build(self):
        """
        Build ROS-related files
        """
        self.__generate_ros_dir_structure()   # generate necessary folders
        self.__generate_ros_package_xml()     # generate package.xml
        self.__generate_ros_cmakelists()      # generate CMakeLists.txt
        self.__copy__ros_files()              # Copy certain files
        #                                     #   - C++ bindings, library, msg
        self.__generate_ros_params_file()     # generate params file
        self.__generate_ros_node_header()     # generate node .hpp file
        self.__generate_ros_node_cpp()        # generate main node .cpp file
        self.__generate_ros_launch_file()     # generate launch file
        self.__generate_ros_readme_file()     # final touch: create README.md
        self.__symbolic_link_info_message()   # Info: create symbolic link
