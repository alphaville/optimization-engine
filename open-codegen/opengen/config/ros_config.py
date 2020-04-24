from .meta import OptimizerMeta


class RosConfiguration:

    def __init__(self, meta: OptimizerMeta):
        self.__package_name = "ros_node_" + meta.optimizer_name
        self.__node_name = "ros_node" + meta.optimizer_name + "_optimizer"
        self.__description = "parametric optimization with OpEn"
        self.__rate = 10.0
        self.__solution_topic_queue_size = 100
        self.__params_topic_queue_size = 100
        self.__publisher_subtopic = "solution"
        self.__subscriber_subtopic = "parameters"

    @property
    def package_name(self):
        return self.__package_name

    @property
    def node_name(self):
        return self.__node_name

    @property
    def publisher_subtopic(self):
        return self.__publisher_subtopic

    @property
    def subscriber_subtopic(self):
        return self.__subscriber_subtopic

    @property
    def description(self):
        return self.__description

    @property
    def rate(self):
        return self.__rate

    @property
    def solution_topic_queue_size(self):
        return self.__solution_topic_queue_size

    @property
    def params_topic_queue_size(self):
        return self.__params_topic_queue_size

    def with_package_name(self, pkg_name):
        self.__package_name = pkg_name
        return self

    def with_node_name(self, node_name):
        self.__node_name = node_name
        return self

    def with_rate(self, rate):
        self.__rate = rate
        return self
