"""Continuous-time dynamics discretization helpers for OCP problems."""

import casadi.casadi as cs


class DynamicsDiscretizer:
    r"""Discretizer for continuous-time dynamics.

    This helper wraps a continuous-time model of the form
    :math:`\dot{x} = f(x, u, p)` and produces a discrete-time callback
    compatible with :meth:`opengen.ocp.problem.OptimalControlProblem.with_dynamics`.
    """

    def __init__(self, continuous_dynamics, sampling_time):
        r"""Construct a dynamics discretizer.

        :param continuous_dynamics: callable implementing
            :math:`\dot{x} = f(x, u, p)`
        :param sampling_time: sampling time :math:`T_s`
        :raises ValueError: if the sampling time is not positive
        """
        if sampling_time <= 0:
            raise ValueError("sampling_time must be positive")

        self.__continuous_dynamics = continuous_dynamics
        self.__sampling_time = float(sampling_time)

    @property
    def sampling_time(self):
        """Sampling time :math:`T_s`."""
        return self.__sampling_time

    @property
    def continuous_dynamics(self):
        """Continuous-time dynamics callback."""
        return self.__continuous_dynamics

    def euler(self):
        r"""Return an explicit Euler discretization callback.

        The returned callback implements:

        :math:`x^+ = x + T_s f(x, u, p)`.
        """

        def discrete_dynamics(x, u, param):
            return x + self.__sampling_time * self.__continuous_dynamics(x, u, param)

        return discrete_dynamics

    def midpoint(self):
        r"""Return an explicit midpoint discretization callback.

        The returned callback implements the second-order Runge-Kutta rule:

        :math:`x^+ = x + T_s f(x + \tfrac{T_s}{2} k_1, u, p)`,

        where :math:`k_1 = f(x, u, p)`.
        """
        ts = self.__sampling_time
        f = self.__continuous_dynamics

        def discrete_dynamics(x, u, param):
            k1 = f(x, u, param)
            midpoint_state = x + 0.5 * ts * k1
            return x + ts * f(midpoint_state, u, param)

        return discrete_dynamics

    def heun(self):
        r"""Return a Heun discretization callback.

        The returned callback implements the explicit trapezoidal method:

        :math:`x^+ = x + \tfrac{T_s}{2}(k_1 + k_2)`,

        where :math:`k_1 = f(x, u, p)` and
        :math:`k_2 = f(x + T_s k_1, u, p)`.
        """
        ts = self.__sampling_time
        f = self.__continuous_dynamics

        def discrete_dynamics(x, u, param):
            k1 = f(x, u, param)
            k2 = f(x + ts * k1, u, param)
            return x + 0.5 * ts * (k1 + k2)

        return discrete_dynamics

    def rk4(self):
        r"""Return a classical fourth-order Runge-Kutta discretization callback.

        The returned callback implements:

        :math:`x^+ = x + \frac{T_s}{6}(k_1 + 2k_2 + 2k_3 + k_4)`,

        where the intermediate slopes are computed from the continuous-time
        dynamics.
        """
        ts = self.__sampling_time
        f = self.__continuous_dynamics

        def discrete_dynamics(x, u, param):
            k1 = f(x, u, param)
            k2 = f(x + 0.5 * ts * k1, u, param)
            k3 = f(x + 0.5 * ts * k2, u, param)
            k4 = f(x + ts * k3, u, param)
            return x + (ts / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

        return discrete_dynamics

    def multistep(self, method="rk4", num_steps=1):
        """Return a discrete-time callback with internal substeps.

        This helper subdivides the sampling interval :math:`T_s` into
        ``num_steps`` smaller steps and applies the chosen explicit method on
        each subinterval.

        Supported methods are ``"euler"``, ``"midpoint"``, ``"heun"``, and
        ``"rk4"``.

        :param method: base discretization method
        :param num_steps: number of internal substeps
        :return: discrete-time dynamics callback
        :raises ValueError: if ``num_steps`` is not a positive integer or the
            method is unknown
        """
        if not isinstance(num_steps, int) or num_steps <= 0:
            raise ValueError("num_steps must be a positive integer")

        if num_steps == 1:
            return self.discretize(method)

        substep = DynamicsDiscretizer(
            self.__continuous_dynamics,
            self.__sampling_time / float(num_steps),
        )
        base_step = substep.discretize(method)

        def discrete_dynamics(x, u, param):
            x_next = x
            for _ in range(num_steps):
                x_next = base_step(x_next, u, param)
            return x_next

        return discrete_dynamics

    def discretize(self, method="euler"):
        """Return a discrete-time dynamics callback for a chosen method.

        Supported methods are ``"euler"``, ``"midpoint"``, ``"heun"``, and
        ``"rk4"``.

        :param method: discretization method
        :return: discrete-time dynamics callback
        :raises ValueError: if the method is unknown
        """
        method = method.lower()
        if method == "euler":
            return self.euler()
        if method == "midpoint":
            return self.midpoint()
        if method == "heun":
            return self.heun()
        if method == "rk4":
            return self.rk4()
        raise ValueError(f"unknown discretization method '{method}'")
