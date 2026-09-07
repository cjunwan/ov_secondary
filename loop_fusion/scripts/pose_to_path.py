#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import tf.transformations as tfs

class GTTransformer:
    def __init__(self):
        rospy.init_node('gt_transformer', anonymous=True)

        # Publishers
        self.path_pub = rospy.Publisher('/gt_path_transformed', Path, queue_size=10)
        self.marker_pub = rospy.Publisher('/gt_path_transformed_marker', Marker, queue_size=10)
        
        # Message templates
        self.path = Path()
        self.path.header.frame_id = "global"
        self.marker = self.create_marker_template()

        # State variable for the final transformation
        self.transform_matrix = None
        self.point_counter = 0
        self.subsample_rate = 5 # Change this value to adjust dot spacing

        # Subscriber
        self.gt_pose_sub = rospy.Subscriber('/vrpn_mocap/jumping/pose', PoseStamped, self.gt_pose_callback, queue_size=100)
        rospy.loginfo("GT Transformer node started.")

    def create_marker_template(self):
        marker = Marker()
        marker.header.frame_id = "global"
        marker.ns = "gt_path_transformed"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 1.0, 0.0, 1.0 # Yellow
        marker.pose.orientation.w = 1.0
        return marker

    def pose_to_matrix(self, pose):
        trans = [pose.position.x, pose.position.y, pose.position.z]
        orient = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        return tfs.concatenate_matrices(tfs.translation_matrix(trans), tfs.quaternion_matrix(orient))

    def matrix_to_pose_stamped(self, matrix, stamp, frame_id):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = stamp
        pose_stamped.header.frame_id = frame_id
        trans = tfs.translation_from_matrix(matrix)
        orient = tfs.quaternion_from_matrix(matrix)
        pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z = trans
        pose_stamped.pose.orientation.x, pose_stamped.pose.orientation.y, pose_stamped.pose.orientation.z, pose_stamped.pose.orientation.w = orient
        return pose_stamped

    def gt_pose_callback(self, msg):
        # On the first message, compute the combined transformation
        if self.transform_matrix is None:
            rospy.loginfo("First GT pose received. Computing transformation.")
            
            # 1. The desired starting pose: at origin with +45 deg yaw
            target_start_matrix = tfs.quaternion_matrix(tfs.quaternion_from_euler(0, 0, np.pi / 4))

            # 2. The actual first pose from the data
            first_gt_matrix = self.pose_to_matrix(msg.pose)

            # 3. Compute the transformation matrix to get from actual to desired:
            #    T_target = T_transform * T_actual
            #    T_transform = T_target * inverse(T_actual)
            self.transform_matrix = np.dot(target_start_matrix, tfs.inverse_matrix(first_gt_matrix))

        # Apply the transformation to the current GT pose
        current_gt_mat = self.pose_to_matrix(msg.pose)
        transformed_gt_mat = np.dot(self.transform_matrix, current_gt_mat)
        
        # Convert back to PoseStamped
        transformed_pose_stamped = self.matrix_to_pose_stamped(transformed_gt_mat, msg.header.stamp, "global")
        
        # Append to the continuous path every time
        self.path.poses.append(transformed_pose_stamped)

        # Append to the marker only every N points to create a dotted effect
        if self.point_counter % self.subsample_rate == 0:
            self.marker.points.append(transformed_pose_stamped.pose.position)
        self.point_counter += 1

        # Publish
        current_time = rospy.Time.now()
        self.path.header.stamp = current_time
        self.marker.header.stamp = current_time
        if not rospy.is_shutdown():
            self.path_pub.publish(self.path)
            self.marker_pub.publish(self.marker)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        transformer = GTTransformer()
        transformer.run()
    except rospy.ROSInterruptException:
        pass