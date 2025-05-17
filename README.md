# camera-ros-integration
Implementation of Aruco marker detection, colour detection, and general camera data transmision via topics in ROS2 packages.

camera_node: takes a image from a specified camera device at 15 FPS, converts it to usable data with cv2.
- publishers:
  - image_raw: the raw converted image
-subscribers:
  - N/A

processing_node: takes the image and applied colour detection and ArUco marker detection/classification.
- publishers:
  - image_processed: the processed image with colour highlighting and marker outlines
  - aruco_detections: list of ArUco markers detected (4x4 100 by CIRC 2025 rules)
  - color_detections: list of colors detected (only red & blue implemented currently)
- subscribers:
  - image_raw: image converted from camera_node

Camera nodes and processing nodes must come in pairs, i.e. 2 cameras need 2 camera nodes and 2 processing nodes.

TODO: implement with sockets for server
