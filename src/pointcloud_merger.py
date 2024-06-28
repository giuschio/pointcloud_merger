#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs

from sensor_msgs.msg import PointCloud2

class PointCloudMerger:
    def __init__(self):
        rospy.init_node('pointcloud_merger')
        
        # Get parameters from launch file
        self.camera_topics = rospy.get_param('~camera_topics')
        self.camera_frames = rospy.get_param('~camera_frames')
        self.target_frame = rospy.get_param('~target_frame')
        self.publish_frequency = rospy.get_param('~publish_frequency', 1.0)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.buffer = {camera: {'pointcloud': None, 'timestamp': None} for camera in self.camera_frames}

        self.pointcloud_subscribers = []
        for topic, frame in zip(self.camera_topics, self.camera_frames):
            print(f'Subscribing to topic {topic} and frame {frame}')
            sub = rospy.Subscriber(topic, PointCloud2, self.pointcloud_callback, callback_args=frame)
            self.pointcloud_subscribers.append(sub)

        self.merged_pub = rospy.Publisher('~merged_pointcloud', PointCloud2, queue_size=10)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_frequency), self.publish_merged_pointcloud)

    def pointcloud_callback(self, msg, camera_frame):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, msg.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            transformed_cloud = tf2_sensor_msgs.do_transform_cloud(msg, transform)
            self.buffer[camera_frame] = {'pointcloud': transformed_cloud, 'timestamp': msg.header.stamp}

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)

    def get_merged_pointcloud(self):
        pointclouds = [data['pointcloud'] for data in self.buffer.values() if data['pointcloud'] is not None]
        if not pointclouds:
            return None
        
        merged_cloud = pointclouds[0]  # This is where you would implement actual merging logic
        for cloud in pointclouds[1:]:
            merged_cloud += cloud
        return merged_cloud

    def publish_merged_pointcloud(self, event):
        merged_cloud = self.get_merged_pointcloud()
        if merged_cloud:
            self.merged_pub.publish(merged_cloud)

if __name__ == '__main__':
    try:
        merger = PointCloudMerger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
