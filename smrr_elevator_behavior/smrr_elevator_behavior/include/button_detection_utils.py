import cv2
from std_msgs.msg import Int16MultiArray
from cv_bridge import CvBridge
import yaml

bridge = CvBridge()

class ButtonDetectionUtils():
    def __init__(self, node, target_button) :
        self.node          = node
        self.target_button = target_button


    def detect_target_button(self, results, target_button, model):
        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                c = box.cls

                if (model.names[int(c)] == target_button):
                    x_min = int(b[0])  
                    y_min = int(b[1])  
                    x_max = int(b[2])  
                    y_max = int(b[3])  

                    mid_point_x = int((x_min + x_max) / 2)
                    mid_point_y = int((y_min + y_max) / 2)

                    return [mid_point_x, mid_point_y]

            return None


    def camera_callback(self, msg, model, target_button):
        img      = bridge.imgmsg_to_cv2(msg, "bgr8")
        conf_val = 0.1
        results  = model(img, conf = conf_val)

        pixel_point = self.detect_target_button(results, target_button, model)

        if pixel_point is None:
            self.node.get_logger().warn("Failed to detect target button : %s" % self.target_button)
            return

        return pixel_point[0], pixel_point[1]


    def read_yaml(self, file_path):
        with open(file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
        
        return data
    

    def update_yaml(self, file_path, new_data):
        with open(file_path, 'w') as yaml_file:
            yaml.dump(new_data, yaml_file)