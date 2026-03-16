import casadi.casadi as cs


class ParameterDefinition:
    """Named parameter block used to pack the flat OpEn parameter vector."""

    def __init__(self, name, size, default=None):
        if not isinstance(name, str) or not name:
            raise ValueError("parameter name must be a non-empty string")
        if not isinstance(size, int) or size <= 0:
            raise ValueError("parameter size must be a positive integer")

        self.__name = name
        self.__size = size
        self.__default = ParameterDefinition._normalize_default(default, size)

    @staticmethod
    def _normalize_default(default, size):
        if default is None:
            return None

        if size == 1 and not isinstance(default, (list, tuple)):
            return [float(default)]

        if len(default) != size:
            raise ValueError("default value has incompatible dimension")

        return [float(value) for value in default]

    @property
    def name(self):
        return self.__name

    @property
    def size(self):
        return self.__size

    @property
    def default(self):
        return None if self.__default is None else list(self.__default)

    def has_default(self):
        return self.__default is not None


class ParameterView:
    """Dictionary-like accessor for symbolic parameter slices."""

    def __init__(self, packed_symbol, slices):
        self.__packed_symbol = packed_symbol
        self.__slices = dict(slices)

    def __getitem__(self, name):
        if name not in self.__slices:
            raise KeyError(f"unknown parameter '{name}'")
        start, end = self.__slices[name]
        return self.__packed_symbol[start:end]

    def get(self, name, default=None):
        if name not in self.__slices:
            return default
        return self[name]

    def keys(self):
        return self.__slices.keys()

    @property
    def packed(self):
        return self.__packed_symbol


class ParameterPack:
    """Parameter registry and utilities for symbolic and numeric packing."""

    def __init__(self, symbol_type=cs.SX.sym):
        self.__definitions = []
        self.__definitions_by_name = {}
        self.__symbol_type = symbol_type

    def add(self, name, size, default=None):
        if name in self.__definitions_by_name:
            raise ValueError(f"parameter '{name}' already exists")

        definition = ParameterDefinition(name, size, default=default)
        self.__definitions.append(definition)
        self.__definitions_by_name[name] = definition
        return self

    def total_size(self):
        return sum(definition.size for definition in self.__definitions)

    def symbol(self, name="p"):
        return self.__symbol_type(name, self.total_size())

    def slices(self):
        offset = 0
        slices = {}
        for definition in self.__definitions:
            next_offset = offset + definition.size
            slices[definition.name] = (offset, next_offset)
            offset = next_offset
        return slices

    def view(self, packed_symbol):
        return ParameterView(packed_symbol, self.slices())

    def pack(self, values=None):
        values = {} if values is None else dict(values)
        packed = []
        missing = []

        for definition in self.__definitions:
            value = values.get(definition.name, definition.default)
            if value is None:
                missing.append(definition.name)
                continue

            if definition.size == 1 and not isinstance(value, (list, tuple)):
                value = [value]

            if len(value) != definition.size:
                raise ValueError(
                    f"parameter '{definition.name}' has incompatible dimension"
                )

            packed.extend(float(item) for item in value)

        if missing:
            raise ValueError(
                "missing values for parameters: " + ", ".join(sorted(missing))
            )

        return packed

    def definitions(self):
        return list(self.__definitions)
