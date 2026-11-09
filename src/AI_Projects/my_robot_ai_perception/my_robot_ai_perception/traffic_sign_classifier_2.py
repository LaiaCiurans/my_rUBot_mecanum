#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO


class TrafficSignClassifier(Node):

    def __init__(self):
        super().__init__('traffic_sign_classifier')

        self.declare_parameter('model', 'yolo11n_cls_g1.pt')
        self.declare_parameter('image_topic', '/camera/image')
        self.declare_parameter('confidence', 0.70)
        self.declare_parameter('stable_frames', 3)

        model_name = self.get_parameter('model').value
        image_topic = self.get_parameter('image_topic').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.stable_frames = int(self.get_parameter('stable_frames').value)

        self.bridge = CvBridge()
        self.model = YOLO(model_name)

        self.publisher = self.create_publisher(String, '/traffic_sign', 10)
        self.image_publisher = self.create_publisher(Image, '/traffic_sign/image', 10)
        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        self.last_class = None
        self.class_count = 0
        self.last_published_class = None

        self.get_logger().info(f'Traffic sign classifier listening on {image_topic}')

    def normalize_class(self, class_name):
        name = class_name.lower().replace(' ', '_').replace('-', '_')

        if 'left' in name:
            return 'left'
        if 'right' in name:
            return 'right'
        if 'stop' in name:
            return 'stop'
        if 'give' in name:
            return 'give'
        if 'forbidden' in name or 'no_entry' in name:
            return 'forbidden'
        return name

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(frame, verbose=False)
            result = results[0]

            if result.probs is None:
                self.reset_detection()
                return

            class_index = int(result.probs.top1)
            confidence = float(result.probs.top1conf)
            class_name = result.names[class_index]
            detected_class = self.normalize_class(class_name)

            if confidence < self.confidence:
                self.reset_detection()
                return

            if detected_class == self.last_class:
                self.class_count += 1
            else:
                self.last_class = detected_class
                self.class_count = 1

            if self.class_count >= self.stable_frames:
                if detected_class != self.last_published_class:
                    output = String()
                    output.data = detected_class
                    self.publisher.publish(output)
                    self.last_published_class = detected_class
                    self.get_logger().info(
                        f'Published {detected_class} ({confidence:.2f})'
                    )

        except Exception as error:
            self.get_logger().error(f'Image processing failed: {error}')
            self.reset_detection()

    def reset_detection(self):
        self.last_class = None
        self.class_count = 0

        if self.last_published_class is not None:
            output = String()
            output.data = 'none'
            self.publisher.publish(output)
            self.get_logger().info('No stable sign detected; published none.')
            self.last_published_class = None


def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignClassifier()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
