 
import numpy as np
import cv2 as cv
import pyrealsense2 as rs
from datetime import datetime

print("Import of required modules OK")

#cap = cv.VideoCapture(-1)

#if not cap.isOpened():
#    print("Cannot open camera")
#    exit()

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

#640x480
#1280x720
#...
#config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

print("Setup OK")

try:
    while True:
        
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        #cv.namedWindow('RealSense', cv.WINDOW_AUTOSIZE)
        #cv.imshow('RealSense', images)

        #Get gray version
        gray = cv.cvtColor(color_image, cv.COLOR_BGR2GRAY)

        #Display the resulting frame
        cv.imshow('CalibrationFrame', gray)

        key = cv.waitKey(1)

        if key == ord('q'):
            break

        if key == ord('c'):
            cv.imwrite(f"calibrate_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png", gray)
            print("Frame acquired")

finally:
    # Stop streaming
    pipeline.stop()
    cv.destroyAllWindows()