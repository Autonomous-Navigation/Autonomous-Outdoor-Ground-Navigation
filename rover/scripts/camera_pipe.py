import pyrealsense2 as rs

# Create a RealSense pipeline
pipeline = rs.pipeline()

# Configure the pipeline as needed (e.g., enable streams, set options)

# Start streaming
pipeline.start()

try:
    while True:
        # Capture and process frames here
        frames = pipeline.wait_for_frames()
        # Process frames...

except KeyboardInterrupt:
    # Clean up when the program is interrupted
    pipeline.stop()

