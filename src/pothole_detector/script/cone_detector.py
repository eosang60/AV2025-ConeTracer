#!/usr/bin/env python3
import os
import rospy
import torch
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge

IMAGE_TOPIC = "/usb_cam/image_raw"
BBOX_TOPIC = "/cone_bboxes"

class ConeDetector:
    def __init__(self, yolov5_dir, model_path, conf_thresh):
        self.model = torch.hub.load(yolov5_dir, 'custom', path=model_path, source='local')
        self.model.conf = conf_thresh

        if torch.cuda.is_available():
            self.model = self.model.cuda()
            self.model.half()  
            rospy.loginfo("YOLO 모델이 GPU(CUDA)에서 동작합니다.")
        else:
            rospy.logwarn("YOLO 모델이 CPU에서 동작합니다. (실시간성 저하 가능)")

        self.bridge = CvBridge()
        self.sub_img = rospy.Subscriber(IMAGE_TOPIC, Image, self.imgCallback, queue_size=5)
        self.pub_bbox = rospy.Publisher(BBOX_TOPIC, Float32MultiArray, queue_size=5)

    def imgCallback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
            with torch.no_grad():
                results = self.model(cv_img, size=640)
            bboxes = []

            for idx, (*box, conf, cls) in enumerate(results.xyxy[0]):
                class_id = int(cls)
                class_name = self.model.names[class_id if hasattr(self.model, 'names') else str(class_id)]
                x1, y1, x2, y2 = map(int, box)
                bboxes.extend([x1, y1, x2, y2, float(conf), int(cls)])

            # 바운딩박스만 publish (마커는 cone_3d에서 처리)
            arr = Float32MultiArray(data=bboxes)
            self.pub_bbox.publish(arr)

        except Exception as e:
            rospy.logerr(f"[ConeDetector] Image processing failed: {e}")

def main():
    rospy.init_node("cone_detector_node")
    yolov5_dir = os.path.expanduser(rospy.get_param("~yolov5_dir", "~/yolov5"))
    model_path = os.path.expanduser(rospy.get_param("~model_path", "~/models/best.pt"))
    conf_thresh = rospy.get_param("~conf_threshold", 0.65)

    ConeDetector(yolov5_dir, model_path, conf_thresh)
    rospy.spin()

if __name__ == "__main__":
    main()