import carla

class CustomAgent(object):
    transform = carla.Transform()
    bounding_box = carla.BoundingBox()
    type_id = "vehicle.custom"
    def __init__(self, transform, bounding_box, type_id="vehicle.custom"):
        self.transform = transform
        self.bounding_box = bounding_box
        self.type_id = type_id

    def get_transform(self):
        return self.transform
    
    def get_location(self):
        return self.transform.location