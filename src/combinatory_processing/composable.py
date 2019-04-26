class Composable(object):
    def __init__(self, parent=None):
        self.parent = parent

    def process(self, msg):
        if self.parent is None:
            return self.apply_mapping(msg)
        result = self.parent.process(msg)
        return None if result is None else self.apply_mapping(result)

    def apply_mapping(self, msg):
        return msg

class UnaryMappingComposable(Composable):
    def __init__(self, mapping_func, parent):
        super(UnaryMappingComposable, self).__init__(parent)
        self.mapping_func = mapping_func

    def apply_mapping(self, msg):
        return self.mapping_func(msg)

class MultiMappingComposable(Composable):
    def __init__(self, joining_func):
        super(MultiMappingComposable, self).__init__()
        self.joining_func = joining_func

    def apply_mapping(self, msg):
        return self.joining_func(*msg)
