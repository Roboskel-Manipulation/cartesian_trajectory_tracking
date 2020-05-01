# A ROS package for reactive HRI with Manos

* Piecewise Bezier: Construct a bezier of 20 points and keep only these whose distance is greater than 5mm. Publish each point
with 0.0005s delay on top of the Openpose delay (0.047s)

* Downsampling-Interpolation: Publish interpolated points with 0.0005s delay on top of the Openpose delay (0.047s). Approximatelly
at least in 8% of the points in the natural movements and in 32% of the points in the fast movements.
