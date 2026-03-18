"""Builders and runtime wrappers for OCP-generated optimizers."""

import importlib
import json
import os
import sys
import hashlib
import platform
from datetime import datetime, timezone
import casadi
import casadi.casadi as cs
from importlib.metadata import version, PackageNotFoundError

from opengen.builder.optimizer_builder import OpEnOptimizerBuilder
from opengen.builder.problem import Problem
from opengen.config.build_config import BuildConfiguration
from opengen.config.solver_config import SolverConfiguration
from opengen.constraints.cartesian import CartesianProduct
from opengen.constraints.no_constraints import NoConstraints
from opengen.tcp.optimizer_tcp_manager import OptimizerTcpManager

from .parameter import ParameterPack
from .problem import ShootingMethod
from .solution import OcpSolution


class GeneratedOptimizer:
    """High-level runtime wrapper around a generated optimizer.

    This class hides whether the underlying solver is consumed through direct
    Python bindings or through the TCP interface and exposes a uniform
    ``solve(x0=..., xref=...)`` API based on named parameters.
    """

    def __init__(
        self,
        optimizer_name,
        target_dir,
        backend,
        backend_kind,
        ocp=None,
        symbolic_model=None,
        metadata=None,
    ):
        """Construct a generated optimizer wrapper.

        :param optimizer_name: generated optimizer name
        :param target_dir: generated optimizer directory
        :param backend: low-level backend object
        :param backend_kind: backend type, e.g. ``"direct"`` or ``"tcp"``
        :param ocp: source OCP definition
        :param symbolic_model: precomputed symbolic model of the OCP
        :param metadata: serialized optimizer metadata used when reloading an
            optimizer from disk
        """
        self.__optimizer_name = optimizer_name
        self.__target_dir = os.path.abspath(target_dir)
        self.__backend = backend
        self.__backend_kind = backend_kind
        self.__started = False
        self.__rollout_function = None
        self.__input_slices = None
        self.__state_slices = None

        if metadata is not None:
            self.__initialize_from_metadata(metadata)
        elif ocp is not None:
            self.__initialize_from_ocp(ocp, symbolic_model=symbolic_model)
        else:
            raise ValueError("either ocp or metadata must be provided")

    def __initialize_from_ocp(self, ocp, symbolic_model=None):
        """Populate runtime metadata from an in-memory OCP object."""
        self.__shooting = ocp.shooting
        self.__nx = ocp.nx
        self.__nu = ocp.nu
        self.__horizon = ocp.horizon
        self.__parameters = ocp.parameters
        model = symbolic_model if symbolic_model is not None else ocp.build_symbolic_model()
        self.__input_slices = model.get("input_slices")
        self.__state_slices = model.get("state_slices")
        self.__rollout_function = self.__make_rollout_function(model)

    def __initialize_from_metadata(self, metadata):
        """Populate runtime metadata from a saved JSON manifest."""
        self.__shooting = ShootingMethod(metadata["shooting"])
        self.__nx = metadata["nx"]
        self.__nu = metadata["nu"]
        self.__horizon = metadata["horizon"]
        self.__parameters = ParameterPack()
        for definition in metadata["parameters"]:
            self.__parameters.add(
                definition["name"],
                definition["size"],
                default=definition["default"],
            )
        self.__input_slices = metadata.get("input_slices")
        self.__state_slices = metadata.get("state_slices")
        self.__rollout_function = metadata.get("rollout_function")

    @property
    def target_dir(self):
        """Directory of the generated optimizer project."""
        return self.__target_dir

    @property
    def optimizer_name(self):
        """Name of the generated optimizer."""
        return self.__optimizer_name

    @property
    def backend_kind(self):
        """Backend kind used by this optimizer wrapper."""
        return self.__backend_kind

    def start(self):
        """Start the backend if it is a local TCP server.

        :return: current instance
        """
        if self.__backend_kind == "tcp" and not self.__started:
            self.__backend.start()
            self.__started = True
        return self

    def kill(self):
        """Stop the backend if it is a local TCP server."""
        if self.__backend_kind == "tcp" and self.__started:
            self.__backend.kill()
            self.__started = False

    def __make_rollout_function(self, symbolic_model):
        """
        Create the state rollout function for single shooting
        
        The function is used in single shooting formualations only;
        a function is constructed to compute the sequence of states
        from the sequence of inputs.
        """
        if self.__shooting == ShootingMethod.MULTIPLE:
            return None
        states = cs.horzcat(*symbolic_model["state_trajectory"])
        return cs.Function("ocp_rollout", [symbolic_model["u"], symbolic_model["p"]], [states])

    def __pack_parameters(self, solve_kwargs):
        """Pack named keyword arguments into the flat solver parameter vector."""
        return self.__parameters.pack(solve_kwargs)

    def __extract_inputs(self, flat_solution):
        r"""Extract stage-wise inputs :math:`u_0, \ldots, u_{N-1}` from the flat solution."""
        if self.__shooting == ShootingMethod.SINGLE:
            nu = self.__nu
            return [
                list(flat_solution[stage_idx * nu:(stage_idx + 1) * nu])
                for stage_idx in range(self.__horizon)
            ]

        return [
            list(flat_solution[start:stop])
            for start, stop in self.__input_slices
        ]

    def __extract_states(self, flat_solution, packed_parameters):
        r"""Extract or reconstruct the state trajectory :math:`x_0, \ldots, x_N`."""
        if self.__shooting == ShootingMethod.MULTIPLE:
            x0_start, x0_stop = self.__parameters.slices()["x0"]
            x0 = packed_parameters[x0_start:x0_stop]
            states = [x0]
            states.extend(
                list(flat_solution[start:stop])
                for start, stop in self.__state_slices
            )
            return states

        state_matrix = self.__rollout_function(flat_solution, packed_parameters).full()
        return [state_matrix[:, idx].reshape((-1,)).tolist() for idx in range(state_matrix.shape[1])]

    def __metadata_dict(self):
        """Return the JSON-serializable optimizer manifest."""
        rollout_sha256 = None
        if self.__rollout_function is not None:
            rollout_sha256 = None

        return {
            "manifest_version": 1,
            "optimizer_name": self.__optimizer_name,
            "target_dir": self.__target_dir,
            "backend_kind": self.__backend_kind,
            "created_at": datetime.now(timezone.utc).isoformat(),
            "platform": platform.platform(),
            "python_version": sys.version,
            "opengen_version": self.__safe_package_version("opengen"),
            "casadi_version": self.__casadi_version(),
            "shooting": self.__shooting.value,
            "nx": self.__nx,
            "nu": self.__nu,
            "horizon": self.__horizon,
            "decision_dimension": self.__nu * self.__horizon
            if self.__shooting == ShootingMethod.SINGLE
            else self.__horizon * (self.__nu + self.__nx),
            "parameter_dimension": self.__parameters.total_size(),
            "parameters": [
                {
                    "name": definition.name,
                    "size": definition.size,
                    "default": definition.default,
                }
                for definition in self.__parameters.definitions()
            ],
            "input_slices": self.__input_slices,
            "state_slices": self.__state_slices,
            "rollout_file": "rollout.casadi" if self.__rollout_function is not None else None,
            "rollout_sha256": rollout_sha256,
        }

    @staticmethod
    def __safe_package_version(package_name):
        """Return the installed version of a package if available."""
        try:
            return version(package_name)
        except PackageNotFoundError:
            return None

    @staticmethod
    def __casadi_version():
        """Return the installed CasADi version if available."""
        casadi_version = getattr(casadi, "__version__", None)
        if casadi_version is not None:
            return casadi_version
        return GeneratedOptimizer.__safe_package_version("casadi")

    def save(self, json_path=None):
        """Save a manifest that can later recreate this optimizer.

        The manifest is stored in the generated optimizer directory by default.
        For single-shooting optimizers, the rollout function is stored in a
        separate ``rollout.casadi`` file next to the manifest.

        :param json_path: optional destination manifest path
        :return: current instance
        """
        if json_path is None:
            json_path = os.path.join(self.__target_dir, "optimizer_manifest.json")

        json_path = os.path.abspath(json_path)
        manifest_dir = os.path.dirname(json_path)
        os.makedirs(manifest_dir, exist_ok=True)

        metadata = self.__metadata_dict()
        rollout_file = metadata.get("rollout_file")
        if rollout_file is not None:
            rollout_path = os.path.join(manifest_dir, rollout_file)
            self.__rollout_function.save(rollout_path)
            with open(rollout_path, "rb") as fh:
                metadata["rollout_sha256"] = hashlib.sha256(fh.read()).hexdigest()

        with open(json_path, "w") as fh:
            json.dump(metadata, fh, indent=2)
        return self

    @staticmethod
    def __load_backend(target_dir, optimizer_name, backend_kind):
        """Create a backend object from saved metadata."""
        if backend_kind == "direct":
            if target_dir not in sys.path:
                sys.path.insert(0, target_dir)
            module = importlib.import_module(optimizer_name)
            return module.solver()
        if backend_kind == "tcp":
            return OptimizerTcpManager(target_dir)
        if backend_kind == "none":
            return None
        raise ValueError(f"unknown backend kind '{backend_kind}'")

    @classmethod
    def load(cls, json_path):
        """Load a previously saved optimizer manifest.

        :param json_path: path to a JSON manifest created by :meth:`save`
        :return: reconstructed :class:`GeneratedOptimizer`
        """
        json_path = os.path.abspath(json_path)
        with open(json_path, "r") as fh:
            metadata = json.load(fh)
        rollout_file = metadata.get("rollout_file")
        if rollout_file is not None:
            rollout_path = os.path.join(os.path.dirname(json_path), rollout_file)
            metadata["rollout_function"] = cs.Function.load(rollout_path)
            rollout_sha256 = metadata.get("rollout_sha256")
            if rollout_sha256 is not None:
                with open(rollout_path, "rb") as fh:
                    current_sha256 = hashlib.sha256(fh.read()).hexdigest()
                if current_sha256 != rollout_sha256:
                    raise ValueError("rollout.casadi checksum mismatch")
            else:
                raise ValueError("manifest is missing rollout_sha256")
        backend = cls.__load_backend(
            metadata["target_dir"],
            metadata["optimizer_name"],
            metadata["backend_kind"],
        )
        return cls(
            optimizer_name=metadata["optimizer_name"],
            target_dir=metadata["target_dir"],
            backend=backend,
            backend_kind=metadata["backend_kind"],
            metadata=metadata,
        )

    def solve(
        self,
        initial_guess=None,
        initial_lagrange_multipliers=None,
        initial_penalty=None,
        **parameter_values,
    ):
        r"""Solve the generated OCP optimizer.

        Named keyword arguments are packed according to the declared OCP
        parameters. For example, if the OCP declares ``x0`` and ``xref``, the
        solver can be called as ``optimizer.solve(x0=x0, xref=xref)`` to solve
        the OCP for a given initial condition :math:`x_0` and reference
        :math:`x^{\mathrm{ref}}`.

        :param initial_guess: optional initial decision-variable guess
        :param initial_lagrange_multipliers: optional initial ALM multipliers
        :param initial_penalty: optional initial penalty parameter
        :param parameter_values: named parameter values
        :return: :class:`OcpSolution`
        :raises RuntimeError: if the backend is unavailable or the low-level
            solve call fails
        """
        packed_parameters = self.__pack_parameters(parameter_values)

        if self.__backend_kind == "direct":
            raw = self.__backend.run(
                p=packed_parameters,
                initial_guess=initial_guess,
                initial_lagrange_multipliers=initial_lagrange_multipliers,
                initial_penalty=initial_penalty,
            )
            if raw is None:
                raise RuntimeError("solver failed")
        elif self.__backend_kind == "tcp":
            self.start()
            response = self.__backend.call(
                packed_parameters,
                initial_guess=initial_guess,
                initial_y=initial_lagrange_multipliers,
                initial_penalty=initial_penalty,
            )
            if not response.is_ok():
                raise RuntimeError(str(response.get()))
            raw = response.get()
        else:
            raise RuntimeError("optimizer backend is not available")

        inputs = self.__extract_inputs(raw.solution)
        states = self.__extract_states(raw.solution, packed_parameters)
        return OcpSolution(raw=raw, inputs=inputs, states=states)


class OCPBuilder:
    """Builder that lowers an OCP to the low-level OpEn builder stack."""

    def __init__(
        self,
        problem,
        metadata,
        build_configuration=BuildConfiguration(),
        solver_configuration=None,
    ):
        """Construct an OCP builder.

        :param problem: instance of :class:`OptimalControlProblem`
        :param metadata: optimizer metadata
        :param build_configuration: OpEn build configuration
        :param solver_configuration: OpEn solver configuration
        """
        self.__ocp = problem
        self.__metadata = metadata
        self.__build_configuration = build_configuration
        self.__solver_configuration = (
            solver_configuration
            if solver_configuration is not None
            else SolverConfiguration()
        )
        self.__generate_not_build = False

    def with_generate_not_build_flag(self, flag):
        """Generate code without compiling it.

        :param flag: whether to generate only
        :return: current instance
        """
        self.__generate_not_build = flag
        return self

    def __make_input_constraints(self):
        r"""Build the hard input constraint set for single shooting.

        This produces a stage-wise product set for the control inputs
        :math:`u_0, \ldots, u_{N-1}`.
        """
        stage_constraints = self.__ocp.input_constraints
        if stage_constraints is None:
            return NoConstraints()

        if self.__ocp.horizon == 1:
            return stage_constraints

        segments = [((idx + 1) * self.__ocp.nu) - 1 for idx in range(self.__ocp.horizon)]
        constraints = [stage_constraints] * self.__ocp.horizon
        return CartesianProduct(segments, constraints)

    def __make_multiple_shooting_constraints(self):
        """Build the hard decision-variable set for multiple shooting.

        Depending on the selected OCP options, this acts on input blocks
        :math:`u_t`, stage-wise state-input blocks :math:`(x_t, u_t)`, or the
        terminal state :math:`x_N`.
        """
        stage_constraints = self.__ocp.input_constraints
        if stage_constraints is None:
            stage_constraints = NoConstraints()

        hard_stage_constraints = self.__ocp.hard_stage_state_input_constraints
        hard_terminal_constraints = self.__ocp.hard_terminal_state_constraints

        if hard_stage_constraints is not None and self.__ocp.input_constraints is not None:
            raise ValueError(
                "cannot combine with_input_constraints with "
                "with_hard_stage_state_input_constraints automatically; "
                "encode the input bounds directly in the stage state-input set"
            )

        if hard_stage_constraints is not None and hard_terminal_constraints is not None:
            raise ValueError(
                "cannot combine with_hard_stage_state_input_constraints with "
                "with_hard_terminal_state_constraints automatically; "
                "their intersection on the terminal stage is not supported"
            )

        if hard_stage_constraints is not None:
            segments = [
                ((idx + 1) * (self.__ocp.nu + self.__ocp.nx)) - 1
                for idx in range(self.__ocp.horizon)
            ]
            constraints = [hard_stage_constraints] * self.__ocp.horizon
            return CartesianProduct(segments, constraints)

        if hard_terminal_constraints is not None:
            segments = []
            constraints = []
            offset = -1

            for _ in range(self.__ocp.horizon - 1):
                offset += self.__ocp.nu
                segments.append(offset)
                constraints.append(stage_constraints)

                offset += self.__ocp.nx
                segments.append(offset)
                constraints.append(NoConstraints())

            offset += self.__ocp.nu
            segments.append(offset)
            constraints.append(stage_constraints)

            offset += self.__ocp.nx
            segments.append(offset)
            constraints.append(hard_terminal_constraints)

            return CartesianProduct(segments, constraints)

        segments = []
        constraints = []
        offset = -1

        for _ in range(self.__ocp.horizon):
            offset += self.__ocp.nu
            segments.append(offset)
            constraints.append(stage_constraints)

            offset += self.__ocp.nx
            segments.append(offset)
            constraints.append(NoConstraints())

        return CartesianProduct(segments, constraints)

    def build_problem(self, symbolic_model=None):
        """Lower the OCP to a low-level :class:`opengen.builder.problem.Problem`.

        :return: low-level OpEn problem
        """
        model = symbolic_model if symbolic_model is not None else self.__ocp.build_symbolic_model()
        low_level_problem = Problem(model["u"], model["p"], model["cost"])

        if self.__ocp.shooting == ShootingMethod.SINGLE:
            constraints = self.__make_input_constraints()
        elif self.__ocp.shooting == ShootingMethod.MULTIPLE:
            constraints = self.__make_multiple_shooting_constraints()
        else:
            raise NotImplementedError("unsupported shooting method")

        low_level_problem = low_level_problem.with_constraints(constraints)

        if model["alm_mapping"] is not None:
            low_level_problem = low_level_problem.with_aug_lagrangian_constraints(
                model["alm_mapping"],
                model["alm_set_c"],
                model["alm_set_y"],
            )

        if model["penalty_mapping"] is not None:
            low_level_problem = low_level_problem.with_penalty_constraints(
                model["penalty_mapping"]
            )

        return low_level_problem

    def __make_backend(self, target_dir):
        """Create the runtime backend associated with the generated optimizer."""
        optimizer_name = self.__metadata.optimizer_name

        if self.__build_configuration.build_python_bindings:
            if target_dir not in sys.path:
                sys.path.insert(0, target_dir)
            module = importlib.import_module(optimizer_name)
            return module.solver(), "direct"

        if self.__build_configuration.tcp_interface_config is not None:
            return OptimizerTcpManager(target_dir), "tcp"

        return None, "none"

    def build(self):
        """Generate, compile, and wrap the optimizer.

        :return: :class:`GeneratedOptimizer`
        """
        symbolic_model = self.__ocp.build_symbolic_model()
        low_level_problem = self.build_problem(symbolic_model=symbolic_model)
        builder = OpEnOptimizerBuilder(
            low_level_problem,
            metadata=self.__metadata,
            build_configuration=self.__build_configuration,
            solver_configuration=self.__solver_configuration,
        )
        if self.__generate_not_build:
            builder.with_generate_not_build_flag(True)
        info = builder.build()
        target_dir = os.path.abspath(info["paths"]["target"])
        backend, backend_kind = self.__make_backend(target_dir)
        optimizer = GeneratedOptimizer(
            optimizer_name=self.__metadata.optimizer_name,
            target_dir=target_dir,
            backend=backend,
            backend_kind=backend_kind,
            ocp=self.__ocp,
            symbolic_model=symbolic_model,
        )
        optimizer.save()
        return optimizer
