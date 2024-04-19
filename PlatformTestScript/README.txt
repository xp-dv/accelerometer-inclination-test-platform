Test platform servo angles are set by changing variables in the main.c code
	The variables to set the servo angles are located in the "USER CODE BEGIN PD" section starting at line 33
	XPos is the desired angle of the X Axis servo motor in degrees
	YPos is the desired angle of the Y Axis servo motor in degrees
	The range of both variables is +- 135
	0 corresponds to servo neutral position / platform level with ground
	
Usage Instructions
	1. open main.c 
	2. edit values of XPos and YPos variables to desired angles
	3. upload to nucleo board
	4. press blue push button on nucleo board to run script 
	5. X axis servo will move neutral, then desired position, then return to neutral
	6. After 2 second delay, Y axis servo will move to neutral, then desired position, then return to neutral
	7. press blue push button to run script again
	

