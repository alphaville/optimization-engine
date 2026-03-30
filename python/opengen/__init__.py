"""Top-level package for OpEn with lazy submodule imports.

This module defers importing heavy subpackages to attribute access
to avoid circular import problems during package initialization.

Lazy submodule imports defer the loading of Python modules and their 
attributes until they are first accessed, reducing startup time and 
memory usage.  This is achieved using PEP 562 (__getattr__ and __dir__) 
to intercept attribute access and load the underlying code only when 
necessary.
"""

from importlib import import_module

__all__ = [
	"definitions",
	"builder",
	"config",
	"functions",
	"constraints",
	"tcp",
	"ocp",
]


def __getattr__(name):
	"""Lazily import submodules on attribute access.

	Example: accessing ``opengen.builder`` will import
	``opengen.builder`` and cache it on the package module.

	This defers importing heavy subpackages until they're actually used 
	(lazy imports), reducing startup cost and helping avoid import-time 
	circular dependencies.
	"""
	if name in __all__:
		module = import_module(f"{__name__}.{name}")
		globals()[name] = module
		return module
	raise AttributeError(f"module {__name__!r} has no attribute {name!r}")


def __dir__():
	return sorted(list(__all__) + list(globals().keys()))
