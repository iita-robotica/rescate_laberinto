from controller import Robot, Camera # Step 1: Import Camera

robot = Robot()

colorSensor = robot.getDevice("colour_sensor") # Step 2: Retrieve the sensor, named "colour_sensor", from the robot. Note that the sensor name may differ between robots)

timestep = int(robot.getBasicTimeStep())

colorSensor.enable(timestep) # Step 3: Enable the sensor, using the timestep as the update rate

while robot.step(timestep) != -1:
    
    image = colorSensor.getImage() # Step 4: Retrieve the image frame.
    
    # Get the individual RGB color channels from the pixel (0,0)
    # Note that these functions require you to pass the width of the overall image in pixels.
    # Since this is a 1px by 1px color sensor, the width of the image is just 1. 
    r = colorSensor.imageGetRed(image, 1, 0, 0)
    g = colorSensor.imageGetGreen(image, 1, 0, 0)
    b = colorSensor.imageGetBlue(image, 1, 0, 0)
    
    print("r: " + str(r) + " g: " + str(g) + " b: " + str(b))