import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


class AprilTagTFPublisher(Node):
    def __init__(self):
        super().__init__('custom_apriltag_tf_publisher')
        self.bridge = CvBridge()
        self.info = None
        self.tf = TransformBroadcaster(self)
        self.image_topic = str(self.declare_parameter('image_topic', '/lucid/triton/image_color').value)
        self.info_topic = str(self.declare_parameter('camera_info_topic', '/lucid/triton/camera_info').value)
        self.camera_frame = str(self.declare_parameter('camera_frame', '').value)
        self.tag_id = int(self.declare_parameter('tag_id', 24).value)
        self.tag_frame = str(self.declare_parameter('tag_frame', 'lucid_triton_tag24').value)
        self.marker_size = float(self.declare_parameter('marker_size', 0.07).value)
        self.max_rate = float(self.declare_parameter('max_rate', 10.0).value)
        self.min_interval = 1.0 / self.max_rate if self.max_rate > 0.0 else 0.0
        self.last_process_time = 0.0
        self.detector_params = cv2.aruco.DetectorParameters_create()
        self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.sub_info = self.create_subscription(CameraInfo, self.info_topic, self.info_cb, qos_profile_sensor_data)
        self.sub_image = self.create_subscription(Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_25h9)
        self.get_logger().info(
            f'Publishing {self.tag_frame} from {self.image_topic} '
            f'(camera_info={self.info_topic}, tag_id={self.tag_id}, size={self.marker_size:.3f} m, max_rate={self.max_rate} Hz)')

    def info_cb(self, msg):
        self.info = msg

    def image_cb(self, msg):
        if self.min_interval > 0.0:
            now_sec = self.get_clock().now().nanoseconds * 1e-9
            if (now_sec - self.last_process_time) < self.min_interval:
                return
            self.last_process_time = now_sec

        if self.info is None or self.info.width != msg.width or self.info.height != msg.height:
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception:
            return
        found = None
        for source in (255 - image, image):
            corners, ids, _ = cv2.aruco.detectMarkers(source, self.dictionary, parameters=self.detector_params)
            if ids is not None and self.tag_id in ids.flatten():
                found = corners[list(ids.flatten()).index(self.tag_id)]
                break
        if found is None:
            return
        k = np.asarray(self.info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(self.info.d, dtype=np.float64)
        half = self.marker_size / 2.0
        obj = np.array([[-half, half, 0], [half, half, 0],
                        [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, found.reshape(4,2), k, d, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok: return
        R, _ = cv2.Rodrigues(rvec); q = self.rot_to_quat(R)
        out = TransformStamped()
        # The camera image carries a device/stream timestamp that is not in
        # the ROS clock domain. TF consumers such as easy_handeye2 query at
        # current ROS time, so stamp this live measurement with ROS now.
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.camera_frame or self.info.header.frame_id or msg.header.frame_id
        out.child_frame_id = self.tag_frame
        out.transform.translation.x, out.transform.translation.y, out.transform.translation.z = [float(x) for x in tvec.flat]
        out.transform.rotation.x, out.transform.rotation.y, out.transform.rotation.z, out.transform.rotation.w = q
        self.tf.sendTransform(out)

    @staticmethod
    def rot_to_quat(R):
        tr = np.trace(R)
        if tr > 0:
            s = np.sqrt(tr + 1.0) * 2; return (float((R[2,1]-R[1,2])/s), float((R[0,2]-R[2,0])/s), float((R[1,0]-R[0,1])/s), float(.25*s))
        i = int(np.argmax(np.diag(R))); j,k = (1,2) if i == 0 else ((2,0) if i == 1 else (0,1)); s = np.sqrt(1+R[i,i]-R[j,j]-R[k,k])*2
        q=[0.,0.,0.,0.]; q[i]=s/4; q[3]=(R[k,j]-R[j,k])/s; q[j]=(R[j,i]+R[i,j])/s; q[k]=(R[k,i]+R[i,k])/s; return tuple(q)


def main():
    rclpy.init(); rclpy.spin(AprilTagTFPublisher()); rclpy.shutdown()
