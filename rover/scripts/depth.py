#!/usr/bin/env python

import rospy
import pcl
import pcl.pcl_visualization
import pcl.pcl_registration
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
from pcl import PointCloud
import pcl.pcl_io as io
import pcl.pcl_filters as filters
import pcl.pcl_segmentation as segmentation

def point_cloud_callback(point_cloud_msg):
    # Convert PointCloud2 message to a PCL PointCloud
    cloud = pcl.PointCloud()
    msg = io.msg_to_pointcloud2(point_cloud_msg)
    pcl.fromROSMsg(msg, cloud)

    # Apply a Voxel Grid filter to downsample the point cloud
    vox = cloud.make_voxel_grid_filter()
    voxel_size = 0.01  # Adjust this value according to your point cloud density
    vox.set_leaf_size(voxel_size, voxel_size, voxel_size)
    cloud_filtered = vox.filter()

    # Create a Passthrough filter to isolate a specific depth range (where the curb is)
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_field = "z"
    passthrough.set_filter_field_name(filter_field)
    z_min = 0.1  # Adjust this value to isolate the depth range of the curb
    z_max = 0.2  # Adjust this value to isolate the depth range of the curb
    passthrough.set_filter_limits(z_min, z_max)
    cloud_filtered = passthrough.filter()

    # Create a RANSAC segmentation object to find the curb plane
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.pcl.SAC_RANSAC)
    seg.set_distance_threshold(0.01)  # Adjust this threshold according to your point cloud noise

    # Call segment() to obtain set of inlier indices and model coefficients
    inliers, coefficients = seg.segment()

    if len(inliers) == 0:
        rospy.loginfo("No curb found in this point cloud.")
        return

    # Extract curb coordinates from inliers
    curb_coordinates = []
    for idx in inliers:
        x, y, z = cloud_filtered[idx][:3]  # Extract X, Y, Z coordinates
        curb_coordinates.append((x, y, z))

    # Process curb_coordinates as needed
    rospy.loginfo("Curb coordinates: %s", curb_coordinates)

def main():
    rospy.init_node("curb_extraction_node")
    rospy.Subscriber("/camera/depth/image_rect_raw", PointCloud2, point_cloud_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

