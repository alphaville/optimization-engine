class Rectangle:

    def __init__(self, xmin, xmax):
        self.__xmin = xmin
        self.__xmax = xmax

    @property
    def xmin(self):
        return self.__xmin

    @property
    def xmax(self):
        return self.__xmax

    @xmin.setter
    def xmin(self, xmin):
        self.__xmin = xmin

    @xmax.setter
    def xmin(self, xmax):
        self.__xmax = xmax
