import casadi as cs


class SqDistSOC(cs.Callback):

    def __init__(self, name, opts={}):
        cs.Callback.__init__(self)
        self.construct(name, opts)

    def get_n_in(self): return 1

    def get_n_out(self): return 1

    def init(self):
        pass

    def eval(self, arg):
        x = arg[0]
        f = 0.5 * cs.dot(x, x)
        return [f]

    def has_jacobian(self, *_args):
        return False

    def has_forward(self, *args):
        return True

    def get_forward(self, *args):
        x = cs.MX.sym('x', 1)
        z = cs.MX.sym('x', 1)
        return cs.Function('fw', [x, z], [1])

