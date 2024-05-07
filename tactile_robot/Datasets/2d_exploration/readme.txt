## readme

This dataset comprise 1452 samples of proprio/tactile data from the self-touching robot.

The robot is a serial two arms 2DoF revolute joint robot. 
First angle is the angle of the ""soulder"
Second angle is the angle of the "elbow"

Arm lenghts are specified in the params.txt file
 
Headers:

Time 
	is the computer time in ms.
actionAngle1
	command on joint1 -> angle difference leading to the current angle 
actionAngle2
	command on joint2 -> angle differenc leading to the current angle
jointAngle1
	current join angle1
jointAngle2
	current angle2
pos_x
	finger position x
pos_y 
	finger position y
touch_id_row
	row of most active neuron if above threshold (threshold value in params.txt)
touch_id_col
	column of most active neuron if above threshold
touch
	sensory output of all tactile sensors in a flatten array
	

remarks:

design flaw:
	if touch_id_rwo and touch_id_col are put at 0 0 if there is no activation above the threshold -> doesn't mean there is a touch at 0,0. It means the touch at the current position has not be properly detected above the threshold.

	
	
	

