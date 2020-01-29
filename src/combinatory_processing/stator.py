class Toggler:
    def __init__(self):
        self.prev_held = False
        self.state = False

    def __call__(self, value):
        if self.prev_held:
            self.prev_held = value
        elif value:
            self.state = not self.state
            self.prev_held = True
        return self.state

class RisingEdge:
    def __init__(self):
        self.state = False

    def __call__(self, value):
        if value:
            if not self.state:
                self.state = True
                return True
            return False
        self.state = False
        return False

class Integrator:
    def __init__(self, initial_value=0, lower_bound=None, upper_bound=None):
        self.state = initial_value
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound

    def __call__(self, value):
        self.state += value
        if self.state > self.upper_bound:
            self.state = self.upper_bound
        elif self.state < self.lower_bound:
            self.state = self.lower_bound
        return self.state

class ModularIntegrator:
    def __init__(self, modulo, initial_value=0):
        self.state = initial_value
        self.modulo = modulo

    def __call__(self, value):
        if value > 0:
            self.state = (self.state + value) % self.modulo
        elif value < 0:
            self.state += value
            while self.state < 0:
                self.state += self.modulo
        return self.state
