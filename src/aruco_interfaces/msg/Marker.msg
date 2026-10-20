# A single detected ArUco marker, expressed in the camera frame
int32 id
float64[3] translation   # x, y, z in metres
float64[3] rotation      # OpenCV rotation vector (axis * angle, radians)
