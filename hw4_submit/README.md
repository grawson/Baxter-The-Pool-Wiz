This is a Lab 4 submission for group Team 9
Team Members: Jacob Best (jeb2216), Louis boguslav (lhb2117), and Jon Herman (jsh2201)

youtube link: https://www.youtube.com/watch?v=pALvUEDzWGY

1) Describe the overall architecture of your implementation.

Capture image —> Apply tomography —> Warp image —> [start main loop] Capture image —> warp —> filter colors —> u-turn if stop sign —> fwd() —> get Hough lines, angles, distance —> update trajectory


2) What method did your group use to calculate the distance from the center line?

Using rho returned from HoughLines(). 

3) What method did your group use to calculate the angle offset from the center line?

Using theta returned from HoughLines().

4) Describe your control flow algorithm based on distance, angle, and whatever other metrics you used.

We didn’t make any movements less than 15 degrees since the board couldn’t handle those.  We set a threshold of 10 degrees, below which the robot wouldn’t recenter.  We turned the same angle as theta returned from HoughLines().  For far enough rho, we sharpen the angle (move even more towards the center line).


5) What is the purpose of the distance offset from the camera to the homography transform?

The homography allows us to switch from a pixel reference frame to inches on the roadway.  In the original image, the pixel:distance ratio varies with distance, but in the warped image the ratio is constant.  This helps in determining the robots motion.  Additionally, it helps see the actual direction of the road by giving a birds-eye-view that is not affected by the distance of each point to the camera.  

