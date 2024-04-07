import os

class OBJECT_TYPES:
    CUBE = 'cube'
    DOLL = 'doll'
    SPHERE = 'sphere'

    def __init__(self):
        pass

def get_obj_type_and_id(name:str):
    fields = name.split('_')
    return fields[0], fields[1]