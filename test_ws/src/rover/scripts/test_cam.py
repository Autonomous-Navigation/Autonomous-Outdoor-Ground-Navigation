import pyrealsense2 as rs
import numpy as np
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pc = rs.pointcloud()
pipeline.start(config)

i = 0
while i<5:

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    print(depth_frame)
    color_frame = frames.get_color_frame()
    print(color_frame)
    i = i+1
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)
    vtx = np.asanyarray(points.get_vertices())
    tex = np.asanyarray(points.get_texture_coordinates())
    print(type(points), points)
    print(type(vtx), vtx.shape, vtx)
    print(type(tex), tex.shape, tex)
pipeline.stop()
