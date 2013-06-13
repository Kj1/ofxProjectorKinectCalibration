ofxProjectorKinectCalibration
=============================

Openframeworks addon for calibrating a kinect and a projector, currently work in progress
info: http://forum.openframeworks.cc/index.php?topic=12712.0

used addons: 
- ofxCv
- ofxOpenCv

Kinect Backends:
- ofxKinectNui (supported)
- ofxKinect    (not yet)
- ofxOpenNi    (not yet)

For the example
- the above
- ofxFenster
- ofxKinectNui
- ofxUi
- ofxXmlSettings

Chessboard finding parameters:
- CV_CALIB_CB_ADAPTIVE_THRESH Use adaptive thresholding to convert the image to black and white, rather than a fixed threshold level (computed from the average image brightness).
- CV_CALIB_CB_NORMALIZE_IMAGE Normalize the image gamma with equalizeHist() before applying fixed or adaptive thresholding.
- CALIB_CB_FAST_CHECK Run a fast check on the image that looks for chessboard corners, and shortcut the call if none is found. This can drastically speed up the call in the degenerate condition when no chessboard is observed.


Calibration parameters
- CV_CALIB_FIX_PRINCIPAL_POINT The principal point is not changed during the global optimization.
- CV_CALIB_FIX_ASPECT_RATIO The functions considers only fy as a free parameter. The ratio fx/fy stays the same as in the input cameraMatrix . When CV_CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are ignored, only their ratio is computed and used further.
- CV_CALIB_ZERO_TANGENT_DIST Tangential distortion coefficients are set to zeros and stay zero.
- CV_CALIB_FIX_Kx http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#calibratecamera
- CV_CALIB_RATIONAL_MODEL No idea
