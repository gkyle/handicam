# handicam

Handicam is a prototype optical registration solution for Handibot. For more background, [see this blog post](http://www.kylescholz.com/wp/handicam-optical-registration-for-handibot/).

## Required Hardware

Handibot

Host CPU: Handicam can run on any machine that can run [OpenCV](http://www.opencv.org/), including Linux and Windows PCs. The current implementation has Linux-specific support for interactively controlling camera focus and exposure that are critical to capturing high-quality images and achieving precise registration (particularly under different lighting conditions). For Windows or platforms that don’t support [V4L](https://en.wikipedia.org/wiki/Video4Linux), you can manually control the camera settings with other tools.

The image matching operations performed are CPU-intense. While they can run on a Raspberry Pi, they’ll be quite slow.

Camera: I experimented with a few cameras and got the best results from Logitech C930e. These features made it a good fit:
* Wide angle (90 degrees) (enables camera to be mounted close with a full view of cutting area).
* Good resolution and very good image quality.
* Software controllable focus and exposure.

Camera Mount: Homemade. *TODO: Share STL files.*

Markerboard: Homemade. An acrylic plate with [Aruco markers](https://docs.opencv.org/3.1.0/d5/dae/tutorial_aruco_detection.html) that are used to determine the position and orientation of the camera relative to the surface of the workpiece. I cut mine on a laser cutter from clear acrylic. It could be made from 1/8" plywood or MDF or Polyethylene instead. Aruco markers were printed onto adhesive paper and stuck to the surface. *TODO: Share DXF file.*

## Required Software
* CMake
* OpenCV
* Opencv_contrib (specificly aruco, xfeatures2d)

*TODO: For Windows, pointer to tutorials on OpenCV and Cmake for Visual Studio*

## Use and workflow

### Calibration

Before first use, you must generate a [calibration profile](https://docs.opencv.org/3.1.0/d4/d94/tutorial_camera_calibration.html) for your camera. /Suggestion:/
* See OpenCV guides for calibration.
* Use the “capture” tool described below to capture 30+ still images from different positions as indicated in OpenCV docs. Capture these using the same resolution and manual focus settings that you will use later.
* Make an XML file listing your captured image files, as indicated in the OpenCV docs.
* Use the calibration tool included in the OpenCV tutorial files (opencv/samples/cpp/cpp-tutorial-camera_calibration).
* Calibration should be good if result reprojection error is <1.0 pixels, but also check that projected images at end of calibration process don’t appear distorted.
* Edit config.xml to point to the location of your profile.

### Capture

“capture” is a tool for quickly checking the view from the Camera, ensuring the markerboard is in view and in focus, viewing the top-down projection, and calibrating the focus and exposure to minimize frame-over-frame jank.
* (F/f)ocus, (E/e)xposure, (Z/z)oom: Captial letter increases, lowercase letter decreases.
* (C)apture: Captures an image to file. File names are sequenced sinc ethe start of the program. This is particuarly useful for generating calibration input.
* (P)rojection: Display / hide the top-down projection view.
* (M)arkers: Display / hide outline of Aruco markers.
* (S)tability: Enable / disable stability mode, which performs frame-over-frame matching of the video stream and displays X, Y, and rotation offsets, in pixels. Under ideal conditions, if the camera is stationary, the X, Y, and rotation deltas should be ~0. Adjust the focus and exposure until these values are <10px.
* Esc to exit.

### Stitch

“stitch” is an interactive tool for composing an aggregate scanned image of the work surface from several images. When finished scanning, press Esc to save the file and Esc again to exit.

I recommend scanning the work surface with just the camera taped to the markerboard (not inside the Handibot) for more manueverability.

Note: Stitching a tricky. It may take a few tries to get a good complete scan of a large work piece.

### Match

"match" is an interactive tool for viewing the position of the Handibot on the work surface, including relative X, Y, and rotation offsets from a target tile.
* (G)rid: Select the next tile in the Grid.
* (N)udge mode: Once the position of the Handibot is close to alignment with the grid tile, enable nudge mode to fine tune. In this mode, the displayed offsets are the average of the last 10 frames and estimated error is measurements is displayed.

