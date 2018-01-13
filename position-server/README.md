# How the position server works #


## Alter the camera image for more contrast
## Detect the white box of the playing field
## Find contours with EXACTLY two children
## Check if they are triangles and how they're oriented
## Calculate relevant related points
If alpha is the heading, measured counter clockwise from the horizontal axis and the positive Y direction of the
axis is pointing down, that gives us the Rotation matrix R as follows:
![R = \begin{bmatrix}-sin( \alpha ) & -cos( \alpha) \\ -cos( \alpha) & sin (\alpha) \end{bmatrix}  ](http://www.sciweavers.org/tex2img.php?eq=R%20%3D%20%5Cbegin%7Bbmatrix%7D-sin%28%20%5Calpha%20%29%20%26%20-cos%28%20%5Calpha%29%20%5C%5C%20-cos%28%20%5Calpha%29%20%26%20sin%20%28%5Calpha%29%20%5Cend%7Bbmatrix%7D%20%20&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)

Now it's just a matter of 
![ \begin{bmatrix}-sin( \alpha ) & -cos( \alpha) \\ -cos( \alpha) & sin (\alpha) \end{bmatrix}
  .  \begin{bmatrix} x_{1}  & y_{1} \\ x_{2}  & y_{2} \\ x_{3}  & y_{3} \\ x_{4}  & y_{4} \end{bmatrix}](http://www.sciweavers.org/tex2img.php?eq=%20%5Cbegin%7Bbmatrix%7D-sin%28%20%5Calpha%20%29%20%26%20-cos%28%20%5Calpha%29%20%5C%5C%20-cos%28%20%5Calpha%29%20%26%20sin%20%28%5Calpha%29%20%5Cend%7Bbmatrix%7D%20%20.%20%20%5Cbegin%7Bbmatrix%7D%20x_%7B1%7D%20%20%26%20y_%7B1%7D%20%5C%5C%20x_%7B2%7D%20%20%26%20y_%7B2%7D%20%5C%5C%20x_%7B3%7D%20%20%26%20y_%7B3%7D%20%5C%5C%20x_%7B4%7D%20%20%26%20y_%7B4%7D%20%5Cend%7Bbmatrix%7D%20%20&bc=White&fc=Black&im=jpg&fs=12&ff=arev&edit=0)

$$  \overrightarrow{ab} $$
## Black out the area where there was a robot
## Rediscover contours in the rest of the image to find balls
## Pack everything up in a dictionary and push over UDP