from enum import Enum


class FormulationType(Enum):
    SINGLE_SHOOTING = 1
    MULTIPLE_SHOOTING = 2


class OcpInterfaceType(Enum):
    DIRECT = 1
    TCP = 2


class ConstraintMethod(Enum):
    ALM = 1
    PM = 2
