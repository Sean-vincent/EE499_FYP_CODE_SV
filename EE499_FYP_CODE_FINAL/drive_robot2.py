
import time


def drive_robot2(fa2):
    """
    Controls the movement of a robot based on line sensor readings.
    Implements basic line following logic and maneuvers the robot accordingly.
    
    Args:
        fa2: Object representing the robot's hardware interface.
    """
    # Define sensor thresholds and correction values
    white = 50
    correction = 15
    turnCorrection = 10
    
    # Continuously monitor line sensor readings and adjust robot movement accordingly
    while True:
        # Read line sensor values
        left_sensor = fa2.ReadLine(0)
        right_sensor = fa2.ReadLine(1)
        
        # Implement line following logic based on sensor readings
        if left_sensor >= white and right_sensor <= white:
            # Adjust motors for right turn
            fa2.SetMotors(turnCorrection, -turnCorrection)
        elif right_sensor >= white and left_sensor <= white:
            # Adjust motors for left turn
            fa2.SetMotors(-turnCorrection, turnCorrection)
        elif right_sensor > white and left_sensor > white: 
            # Move backward and then adjust for left turn
            fa2.Backwards(correction*2)
            time.sleep(0.1)
            fa2.SetMotors(turnCorrection, -turnCorrection)
        else:
            # Move forward with correction
            fa2.Forwards(correction)
