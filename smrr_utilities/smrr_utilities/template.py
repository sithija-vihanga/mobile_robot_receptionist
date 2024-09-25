import rclpy

from rclpy.node import Node
from .include.transform import GeometricTransformations

class Template(Node):

    def __init__(self):
        super().__init__('template')
        self.point_arr = None 
        self.transform      = GeometricTransformations(self)
        
    def listener_callback_collision(self, msg):
        transformation   = self.transform.get_transform('map', 'base_link')          

        if transformation:
            transformed_points  = self.transform.transform_points(self.point_arr, transformation)
                
def main(args=None):
    
    rclpy.init(args=args)

    template = Template()

    rclpy.spin(template)

    template.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


