import re


class RosConfiguration:
    """
    Configuration of auto-generated ROS package
    """

    def __init__(self):
        """
        Constructor of an instance of RosConfiguration
        """
        self.__package_name = "open_ros"
        self.__node_name = "ros_node_optimizer"
        self.__description = "parametric optimization with OpEn"
        self.__rate = 10.0
        self.__result_topic_queue_size = 100
        self.__params_topic_queue_size = 100
        self.__publisher_subtopic = "result"
        self.__subscriber_subtopic = "parameters"

    @property
    def package_name(self):
        """
        Package name
        :return: package name (default: 'open_ros')
        """
        return self.__package_name

    @property
    def node_name(self):
        """
        Node name (default: ros_node_optimizer)
        :return:
        """
        return self.__node_name

    @property
    def publisher_subtopic(self):
        """
        Name of publisher sub-topic (default: "result")
        :return:
        """
        return self.__publisher_subtopic

    @property
    def subscriber_subtopic(self):
        """
        Name of subscriber sub-topic (default: "parameters")
        :return:
        """
        return self.__subscriber_subtopic

    @property
    def description(self):
        """
        Description of ROS package (in package.xml)
        :return: description
        """
        return self.__description

    @property
    def rate(self):
        """
        ROS node rate in Hz (default: 10)
        :return: rate
        """
        return self.__rate

    @property
    def result_topic_queue_size(self):
        """
        Size of "result" topic (default: 100)
        :return: result topic name
        """
        return self.__result_topic_queue_size

    @property
    def params_topic_queue_size(self):
        """
        Size of "parameter" topic queue (default: 100)
        :return: parameter topic name
        """
        return self.__params_topic_queue_size

    def with_package_name(self, pkg_name):
        """
        Set the package name, which is the same as the name
        of the folder that will store the auto-generated ROS node.
        The node name can contain lowercase and uppercase
        characters and underscores, but not spaces or other symbols
        :param pkg_name: package name
        :return: current object
        :raises: ValueError if pkg_name is not a legal package name
        """
        if re.match(r"^[a-zA-Z_]+[\w]*$", pkg_name):
            self.__package_name = pkg_name
            return self
        raise ValueError("invalid package name")

    def with_node_name(self, node_name):
        """
        Set the node name. The node name can contain lowercase
        and uppercase characters and underscores, but not spaces
        or other symbols
        :param node_name:
        :return: current object
        :raises: ValueError if node_name is not a legal node name
        """
        if re.match(r"^[a-zA-Z_]+[\w]*$", node_name):
            self.__node_name = node_name
            return self
        raise ValueError("invalid node name")

    def with_rate(self, rate):
        """
        Set the rate of the ROS node
        :param rate: rate in Hz
        :return: current object
        """
        self.__rate = rate
        return self

    def with_description(self, description):
        """
        Set the description of the ROS package
        :param description: description (string)
        :return: current object
        """
        self.__description = description
        return self

    def with_queue_sizes(self,
                         result_topic_queue_size=100,
                         parameter_topic_queue_size=100):
        """
        Set queue sizes for ROS node
        :param result_topic_queue_size:
        :param parameter_topic_queue_size:
        :return: current object
        """
        self.__result_topic_queue_size = result_topic_queue_size
        self.__params_topic_queue_size = parameter_topic_queue_size
        return self

    def with_publisher_subtopic(self, publisher_subtopic):
        """
        The auto-generated node will output its results to the topic
        `~/{publisher_subtopic}`. The subtopic (publisher_subtopic)
        can be specified using this method. The default subtopic name
        is 'result'. This can be configured after the package is
        generated, in `config/open_params.yaml`.

        :param publisher_subtopic: publisher sub-topic name
        :return: current object
        """
        self.__publisher_subtopic = publisher_subtopic
        return self

    def with_subscriber_subtopic(self, subscriber_subtopic):
        """
        The auto-generated node will listen for input at
        `~/{subscriber_subtopic}`. The subtopic (subscriber_subtopic)
        can be specified using this method. The default subtopic name
        is 'parameters'. This can be configured after the package is
        generated, in `config/open_params.yaml`.

        :param subscriber_subtopic: subscriber sub-topic name
        :return: :return: current object
        """
        self.__subscriber_subtopic = subscriber_subtopic
        return self
