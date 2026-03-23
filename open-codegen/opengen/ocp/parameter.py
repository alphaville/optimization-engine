"""Utilities for named OCP parameters and flat OpEn parameter vectors."""

import casadi.casadi as cs


class ParameterDefinition:
    """Definition of a named parameter block.

    A parameter block has a name, a dimension and, optionally, a default
    numeric value. Multiple parameter blocks are later packed into the flat
    parameter vector expected by the low-level OpEn builder.
    """

    def __init__(self, name, size, default=None):
        """Construct a parameter definition.

        :param name: parameter name
        :param size: block dimension
        :param default: optional default numeric value
        :raises ValueError: if the name, size or default are invalid
        """
        if not isinstance(name, str) or not name:
            raise ValueError("parameter name must be a non-empty string")
        if not isinstance(size, int) or size <= 0:
            raise ValueError("parameter size must be a positive integer")

        self.__name = name
        self.__size = size
        self.__default = ParameterDefinition._normalize_default(default, size)

    @staticmethod
    def _normalize_default(default, size):
        """Validate and normalize a default value to a list of floats."""
        if default is None:
            return None

        if size == 1 and not isinstance(default, (list, tuple)):
            return [float(default)]

        if len(default) != size:
            raise ValueError("default value has incompatible dimension")

        return [float(value) for value in default]

    @property
    def name(self):
        """Name of the parameter block."""
        return self.__name

    @property
    def size(self):
        """Dimension of the parameter block."""
        return self.__size

    @property
    def default(self):
        """Default value as a list of floats, or ``None`` if absent."""
        return None if self.__default is None else list(self.__default)

    def has_default(self):
        """Whether this parameter block has a default value."""
        return self.__default is not None


class ParameterView:
    """Dictionary-like accessor for symbolic parameter slices.

    Instances of this class are passed to user-defined callbacks so they can
    access named symbolic parameters using expressions such as
    ``param["xref"]``.
    """

    def __init__(self, packed_symbol, slices):
        """Construct a parameter view.

        :param packed_symbol: flat symbolic parameter vector
        :param slices: dictionary mapping parameter names to index pairs
        """
        self.__packed_symbol = packed_symbol
        self.__slices = dict(slices)

    def __getitem__(self, name):
        """Return the symbolic slice associated with a named parameter."""
        if name not in self.__slices:
            raise KeyError(f"unknown parameter '{name}'")
        start, end = self.__slices[name]
        return self.__packed_symbol[start:end]

    def get(self, name, default=None):
        """Return the symbolic slice associated with ``name`` if it exists."""
        if name not in self.__slices:
            return default
        return self[name]

    def keys(self):
        """Return all known parameter names."""
        return self.__slices.keys()

    @property
    def packed(self):
        """The underlying flat symbolic parameter vector."""
        return self.__packed_symbol


class ParameterPack:
    """Registry and packer for named parameters.

    This class stores the declared parameter blocks of an OCP and provides
    helpers to:

    - construct a flat symbolic parameter vector,
    - create named symbolic views into that vector, and
    - pack concrete numeric values for calls to the generated solver.
    """

    def __init__(self, symbol_type=cs.SX.sym):
        """Construct an empty parameter registry.

        :param symbol_type: CasADi symbol constructor, typically ``cs.SX.sym``
            or ``cs.MX.sym``
        """
        self.__definitions = []
        self.__definitions_by_name = {}
        self.__symbol_type = symbol_type

    def add(self, name, size, default=None):
        """Add a named parameter block.

        :param name: parameter name
        :param size: block dimension
        :param default: optional default numeric value
        :return: current instance
        :raises ValueError: if a parameter with the same name already exists
        """
        if name in self.__definitions_by_name:
            raise ValueError(f"parameter '{name}' already exists")

        definition = ParameterDefinition(name, size, default=default)
        self.__definitions.append(definition)
        self.__definitions_by_name[name] = definition
        return self

    def total_size(self):
        """Return the total dimension of the packed parameter vector."""
        return sum(definition.size for definition in self.__definitions)

    def symbol(self, name="p"):
        """Construct the packed symbolic parameter vector.

        :param name: symbolic variable name
        :return: CasADi symbolic vector containing all parameter blocks
        """
        return self.__symbol_type(name, self.total_size())

    def slices(self):
        """Return the slice map of all parameter blocks.

        :return: dictionary ``name -> (start, stop)``
        """
        offset = 0
        slices = {}
        for definition in self.__definitions:
            next_offset = offset + definition.size
            slices[definition.name] = (offset, next_offset)
            offset = next_offset
        return slices

    def view(self, packed_symbol):
        """Create a named symbolic view of a packed parameter vector."""
        return ParameterView(packed_symbol, self.slices())

    def pack(self, values=None):
        """Pack numeric values into the flat parameter vector.

        :param values: dictionary mapping parameter names to values
        :return: flat list of floats
        :raises ValueError: if a required parameter is missing or dimensions are
            inconsistent
        """
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
        """Return the declared parameter definitions."""
        return list(self.__definitions)
