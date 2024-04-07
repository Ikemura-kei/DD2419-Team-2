import py_trees_ros as ptr
import py_trees

class TemplateBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name="template_behavior"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name)

    def setup(self, **kwargs):
        self.node = kwargs.get('node')
        return super().setup(**kwargs)
    
    def register_value(self, key, read=True, write=True):
        if read:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.READ)
        if write:
            self.blackboard.register_key(key=key, access=py_trees.common.Access.WRITE)

# -- please copy this for defining new class --
class NewBehaviour(TemplateBehavior):
    def __init__(self, name="new_behaviour"):
        super().__init__(name)

    def initialise(self) -> None:
        return super().initialise()

    def update(self):
        pass