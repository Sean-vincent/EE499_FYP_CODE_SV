# Used for parsing at runtime. Input width value for reference object in millimetres
# python SafeOvertakingOnBendSmallScaleAV.py --width 50




# Define lower and upper bounds for light blue if desired (interchanged with green)
#lower_light_blue = np.array([90, 100, 100])
#upper_light_blue = np.array([150, 255, 255])

# Create a mask for light blue
#mask_light_blue = cv2.inRange(hsv, lower_light_blue, upper_light_blue)



import threading
from scipy.spatial import distance as dist
import imutils 
import numpy as np
import argparse
import imutils
import cv2
import time
import FA
from drive_robot2 import drive_robot2  # Import the drive_robot2 function to control a second robot
import threading

# Initialize the robot controller for one robot
fa = FA.Create()
comport1 = 6
fa.ComOpen(comport1)  # Open a communication port to control the robot

# Define locks for synchronizing access to shared resources across multiple threads
motor_lock = threading.Lock()
camera_lock = threading.Lock()
state_lock = threading.Lock()

# Define global flags and conditions for managing robot states and behaviors
overtake_needed = False
overtake_performed = False
waiting_for_clearance = False
drive_enabled = True
overtake_condition = threading.Condition()
drive_condition = threading.Condition()

# Define the event and set it initially.
overtake_completed = threading.Event()
overtake_completed.set()  # Set the event to true initially to allow overtaking to proceed when conditions are met.


def safe_set_motors(speed_left, speed_right):
    """
    Safely set the motor speeds with thread safety.
    
    Args:
        speed_left: Speed of the left motor.
        speed_right: Speed of the right motor.
        
    """
    # Acquire a lock before setting the motor speeds to ensure thread safety
    with motor_lock:
        # Set the motor speeds
        fa.SetMotors(speed_left, speed_right)

def stop_motors():
    """
    Stop all motors and disable driving.
    
    Ensures the operation is thread-safe and updates the drive_enabled flag.
    """
    # Access the global drive_enabled flag
    global drive_enabled
    
    # Acquire a lock before performing motor operations to ensure thread safety
    with state_lock:
        # Stop all motors by setting their speeds to 0
        safe_set_motors(0, 0)
        
        # Update the drive_enabled flag to False
        drive_enabled = False

def drive_robot1():
    """
    Controls the robot based on line sensor readings.
    Implements a basic line following logic and maneuvers based on line sensor input.
    """
    # Define sensor thresholds and correction values
    white = 50  # Sensor threshold for detecting white surface
    correction = 15  # Base speed for correction movements
    turnCorrection = 10  # Differential for turning corrections

    # Continuously monitor line sensor readings and adjust robot movement accordingly
    while True:
        # Wait for the condition to enable driving
        with drive_condition:
            while not drive_enabled:
                drive_condition.wait(timeout=1)  # Check every second if driving should be enabled

        # Read line sensor values
        with motor_lock:
            left_sensor = fa.ReadLine(0)  # Read left line sensor
        right_sensor = fa.ReadLine(1)  # Read right line sensor
        
        # Implement line following logic based on sensor readings
        if left_sensor >= white and right_sensor <= white:
            # Adjust motors for left turn
            fa.SetMotors(turnCorrection, -turnCorrection)
        elif right_sensor >= white and left_sensor <= white:
            # Adjust motors for right turn
            fa.SetMotors(-turnCorrection, turnCorrection)
        elif right_sensor > white and left_sensor > white:
            # Move backward and then adjust for left turn
            fa.Backwards(correction*2)
            time.sleep(0.1)
            fa.SetMotors(turnCorrection, -turnCorrection)
        else:
            # Move forward with correction
            fa.Forwards(correction)




def detect_reference_object(frame, args):
    """
    Function to detect a reference object of a known size in the frame for distance calculation.
    
    Args:
        frame: Input frame to analyze for the presence of a reference object.
        args: Additional arguments including the known width of the reference object.
        
    Returns:
        Tuple containing the center point of the reference object and the pixel-to-millimeter ratio.
    """
    # Convert the frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Lock the camera operations to ensure exclusive access
    with camera_lock:
        # Define the lower and upper bounds for the yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        # Create a mask for yellow objects
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        # Find contours in the yellow mask and grab them
        cnts = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        reference_contour = None

        # Evaluate each contour found
        for c in cnts:
            perimeter = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * perimeter, True)
            
            # Select the contour with exactly four vertices as the reference object
            if len(approx) == 4:
                reference_contour = c
                break

        # Check if a reference contour is found
        if reference_contour is not None:
            # Calculate the bounding box of the reference object
            (x, y, w, h) = cv2.boundingRect(reference_contour)
            
            # Highlight the reference object on the frame
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
            
            # Calculate the center point of the reference object
            reference_point = (int(x + w / 2), int(y + h / 2))
            
            # Calculate the width of the reference object in pixels
            reference_width_pixels = w
            
            # Calculate the pixel-to-millimeter ratio based on the known width of the reference object
            pixels_per_mm = reference_width_pixels / args["width"]
            
            return reference_point, pixels_per_mm

        # Return None values if no reference object is found
        return None, None

def detect_distance(frame, reference_point, pixels_per_mm):
    """
    Computes the distances from the reference object to other detected objects based on their contours in the frame.
    Uses HSV thresholding for different colors to find potential objects and calculates their distances from the reference object.
    Converts these distances in millimetres using the pixels_per_mm ratio from detect_reference_object().
    
    Args:
        frame: Input frame containing objects to analyze for distance calculation.
        reference_point: Center point of the reference object.
        pixels_per_mm: Pixel-to-millimeter ratio calculated from the reference object.
        
    Returns:
        Dictionary containing distances from the reference object to other objects (yellow, green, and red).

    """


    with camera_lock:
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Define lower and upper range of yellow color in HSV
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])

        # Threshold the HSV image to get only yellow colors
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours in the mask
        cnts = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)

        # Initialize the distances dictionary
        distances = {'yellow': None}

        # Loop over the contours
        for c in cnts:
            # Compute the perimeter of the contour
            perimeter = cv2.arcLength(c, True)

            # Approximate the contour shape
            approx = cv2.approxPolyDP(c, 0.04 * perimeter, True) #c is the contour being approximated.
                                                                 # 0.04 * perimeter is the value of epsilon. 
                                                                 # This is often chosen as a fraction of the contour's perimeter. Smaller values will result in more detailed approximation.
                                                                 # True indicates that the approximated curve should be a closed polygon.

            # If the contour has four vertices, it's likely our reference object
            if len(approx) == 4:
                # Compute the Euclidean distance between the reference point and yellow object (in mms)
                D_pixels = dist.euclidean(reference_point, tuple(approx[0][0]))
                D_millims = D_pixels / pixels_per_mm  # Convert pixel distance to mms

                # Store the distance in the distances dictionary
                distances['yellow'] = D_millims
                break

        # define lower and upper range of green color in HSV
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        #green_mask = cv2.inRange(hsv, lower_green, upper_green)

        # define lower and upper range of red color in HSV
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([179, 255, 255])

        # Kernel
        kernel = np.ones((3,3), np.uint8)

        # Threshold the HSV image to get only green colors
        Greensign = cv2.inRange(hsv, lower_green, upper_green)
        Greensign = cv2.erode(Greensign, kernel, iterations=5)
        Greensign= cv2.dilate(Greensign, kernel, iterations=9)

        # Threshold the HSV image to get only red colors
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        Redsign = cv2.bitwise_or(mask_red1, mask_red2)
        Redsign = cv2.erode(Redsign, kernel, iterations=5)
        Redsign = cv2.dilate(Redsign, kernel, iterations=9)

        # find contours in the masks
        cnts_green = cv2.findContours(Greensign.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_green = imutils.grab_contours(cnts_green)

        cnts_red = cv2.findContours(Redsign.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_red = imutils.grab_contours(cnts_red)

        # Initialize array for centers
        green_centers = []
        red_centers = []

        # loop over the contours of green objects
        for c_green in cnts_green:
            # compute the bounding box for the contour
            (x, y, w, h) = cv2.boundingRect(c_green)
            # draw the bounding box around the green object
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # compute the center of the bounding box
            center = (int(x + w/2), int(y + h/2))
            green_centers.append(center)

        # loop over the contours of red objects
        for c_red in cnts_red:
            # compute the bounding box for the contour
            (x, y, w, h) = cv2.boundingRect(c_red)
            # draw the bounding box around the red object
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # compute the center of the bounding box
            center = (int(x + w/2), int(y + h/2))
            red_centers.append(center)

        # Compute the distances between the reference point (largest yellow object) and green objects
        for green_center in green_centers:
            # Draw a line between the reference point (largest yellow object) and green objects
            cv2.line(frame, reference_point, green_center, (0, 255, 0), 2)

            # Compute the Euclidean distance between the reference point (largest yellow object) and green objects (in mms)
            D_pixels = dist.euclidean(reference_point, green_center)
            D_millims = D_pixels / pixels_per_mm  # Convert pixel distance to millimetres

            # Draw the distance on the image
            cv2.putText(frame, "Obstacle: {:.2f}mm".format(D_millims), (int((reference_point[0] + green_center[0]) / 2), int((reference_point[1] + green_center[1]) / 2) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            # Store the distance in the dictionary
            distances['green'] = D_millims

        # Compute the distances between the reference point (largest yellow object) and red objects
        for red_center in red_centers:
            # Draw a line between the reference point (largest yellow object) and red objects
            cv2.line(frame, reference_point, red_center, (0, 0, 255), 2)

            # Compute the Euclidean distance between the reference point (largest yellow object) and red objects (in mms)
            D_pixels = dist.euclidean(reference_point, red_center)
            D_millims = D_pixels / pixels_per_mm  # Convert pixel distance to mms

            # Draw the distance on the image
            cv2.putText(frame, "Other Car: {:.2f}mm".format(D_millims), (int((reference_point[0] + red_center[0]) / 2), int((reference_point[1] + red_center[1]) / 2) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

            # Store the distance in the dictionary
            distances['red'] = D_millims

        return distances




def calculate_obstacle_dimensions(frame, pixels_per_mm):
    """
    Function to calculate the dimensions of green obstacles in a given frame.
    
    Args:
        frame: Input frame containing the scene to be analyzed.
        pixels_per_mm: Conversion ratio from pixels to millimeters.
        
    Returns:
        Dictionary containing the width and height dimensions of the largest detected green obstacle in millimeters.
    """

    with camera_lock:  # Locking the camera data processing
        # Convert the frame to the HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define the lower and upper bounds of the green color range in HSV
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        
        # Create a mask to isolate green regions in the frame
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        # Find contours of green regions in the mask
        cnts_green = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts_green = imutils.grab_contours(cnts_green)

    # Initialize variables
    obstacle_dimensions = {}
    max_area = 0

    # Iterate through detected contours
    for c in cnts_green:
        # Calculate area of contour
        area = cv2.contourArea(c)
        
        # Update max_area if current contour is larger
        if area > max_area:
            max_area = area
            # Get bounding box coordinates of the largest contour
            (x, y, w, h) = cv2.boundingRect(c)
            # Calculate width and height of the obstacle in millimeters
            obstacle_dimensions['width'] = w / pixels_per_mm
            obstacle_dimensions['height'] = h / pixels_per_mm

            # Draw rectangle and text if a green object is found
            #if max_area > 0:
                #cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Draw green rectangle
                #cv2.putText(frame, f"Width: {obstacle_dimensions['width']:.2f}mm, Height: {obstacle_dimensions['height']:.2f}mm",
                            #(x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)  # Display dimensions

    return obstacle_dimensions

def is_obstacle_in_sector(frame):
    """
    Function to detect obstacles in predefined sectors of a given frame and mark them.
    
    Args:
        frame: Input frame containing the scene to be analyzed.
        
    Returns:
        Boolean value indicating whether any obstacle is detected in the sectors (True) or not (False).
    """
    # Create a copy of the frame to work with without altering the original frame
    #detection_frame = frame.copy()
    
    # Lock the camera operations to ensure exclusive access
    with camera_lock:
        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Define the lower and upper bounds of the green color range in HSV
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        
        # Create a mask to isolate green objects in the frame
        green_mask = cv2.inRange(hsv, lower_green, upper_green)

    # Define the size of each sector square and get frame dimensions
    sector_size = 200
    height, width = frame.shape[:2]
    
    # Define sectors as dictionary containing sector boundaries
    sectors = {
        'top_left': (0, 0, sector_size, sector_size),
        'top_right': (width - sector_size, 0, width, sector_size),
        'bottom_left': (0, height - sector_size, sector_size, height),
        'bottom_right': (width - sector_size, height - sector_size, width, height)
    }

    # Iterate through each sector
    for key, (x1, y1, x2, y2) in sectors.items():
        # Extract the sector from the green mask
        sector_mask = green_mask[y1:y2, x1:x2]
        
        # Check if there are any green pixels (obstacles) in the sector
        if cv2.countNonZero(sector_mask) > 0:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (180, 105, 255), 2)  # Pink for occupied
            # Obstacle detected in the sector, return True immediately
            return True
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)  # Blue for unoccupied

    # No obstacles detected in any sector, return False
    return False




def overtake_action(green_width, total_overtake_distance, obstacle_on_bend):
    """
    Function to perform an overtaking maneuver based on the latest assessment of the obstacle's position and size.
    
    Args:
        green_width: Width of the green obstacle detected.
        total_overtake_distance: Total distance required for overtaking.
        obstacle_on_bend: Boolean indicating whether the obstacle is on a bend or not.
    """
    # Global variables initialization
    global overtake_needed, overtake_performed, drive_enabled
    
    # Reset the event indicating overtaking completion
    overtake_completed.clear()

    # Ensure exclusive access to state-related operations
    with state_lock:
        # Check if overtaking is not needed or already performed
        if not overtake_needed or overtake_performed:
            # Set the overtaking completion event and return if conditions are not met
            overtake_completed.set()
            return

    # Continuous loop for overtaking actions
    while True:
        # Wait until it's necessary to overtake
        with overtake_condition:
            overtake_condition.wait()

            # Check if overtaking is needed and not already performed
            if overtake_needed and not overtake_performed:
                # Stop the robot to prepare for overtaking
                with motor_lock:
                    fa.SetMotors(0, 0)
                time.sleep(0.2)

                # Execute overtaking on a bend if the obstacle is on a bend
                if obstacle_on_bend:
                    perform_bend_overtake(green_width, total_overtake_distance)
                    print("Obstacle is on a bend")
                    
                # Execute overtaking on a straight path if the obstacle is not on a bend
                elif not obstacle_on_bend:
                    perform_straight_overtake(green_width, total_overtake_distance)
                    print("Obstacle is on a straight")

                # Finalize overtaking and set the overtaking completion event
                finalize_overtake()
                overtake_completed.set()
                break

def perform_bend_overtake(green_width, distance):
    """
    Function to perform overtaking maneuver on a bend.
    
    Args:
        green_width: Width of the green obstacle detected.
        distance: Distance to be covered during the overtaking maneuver.
    """
    # Ensure exclusive access to motor operations
    with motor_lock:
        # Turn left by 90 degrees
        fa.Left(90)
        
        # Move forwards by green_width + 110 (110 for 1 car width safety buffer)
        fa.Forwards(green_width + 110)
        
        # Turn right by 90 degrees
        fa.Right(90)
        
        # Move forwards by distance
        fa.Forwards(distance)
        
        # Turn left by 90 degrees
        fa.Left(90)
    
    # Pause for a short duration
    time.sleep(0.2)

def perform_straight_overtake(green_width, distance):
    """
    Function to perform overtaking maneuver on a straight path.
    
    Args:
        green_width: Width of the green obstacle detected.
        distance: Distance to be covered during the overtaking maneuver.
    """
    # Ensure exclusive access to motor operations
    with motor_lock:
        # Turn left by 90 degrees
        fa.Left(90)
        
        # Move forwards by green_width + 30
        fa.Forwards(green_width + 30)
        
        # Turn right by 90 degrees
        fa.Right(90)
        
        # Move forwards by distance + 110 (110 for safety buffer)
        fa.Forwards(distance + 110)
        
        # Turn right by 90 degrees
        fa.Right(90)
        
        # Move forwards by green_width + 30
        fa.Forwards(green_width + 30)
        
        # Turn left by 90 degrees
        fa.Left(90)
    
    # Pause for a short duration to allow camera to begin processing
    time.sleep(0.2)

def finalize_overtake():
    """
    Function to finalize the overtaking maneuver and update related state variables.
    """
    # Declare global variables
    global overtake_needed, overtake_performed, drive_enabled
    
    # Pause for 0.5 seconds to ensure stability
    time.sleep(0.5)
    
    # Ensure exclusive access to motor operations, state variables, and drive condition
    with motor_lock, state_lock, drive_condition:
        # Update state variables
        overtake_needed = False
        overtake_performed = True
        drive_enabled = True
        
        # Notify all threads that the overtake maneuver is complete
        drive_condition.notify_all()



def main():
    """
    Main function responsible for controlling the robot's behavior and interactions with its environment.
    """

    global overtake_needed, overtake_performed, waiting_for_clearance, drive_enabled, obstacle_on_bend

    # Parse command-line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-w", "--width", type=float, required=True,
                    help="width of the reference object in the image (in millimetres)")
    args = vars(ap.parse_args())

    # Initialize camera
    camera = cv2.VideoCapture(1)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    camera.set(cv2.CAP_PROP_FPS, 60)
    time.sleep(0.1)


    # Initialize hardware interface for robot 2
    fa2 = FA.Create()
    comport2 = 7
    fa2.ComOpen(comport2)

    # Start threads for controlling robot movement
    threads = []
    threads.append(threading.Thread(target=drive_robot2, args=(fa2,), daemon=True))
    threads.append(threading.Thread(target=drive_robot1, daemon=True))
    for thread in threads:
        thread.start()


    # Main loop for robot operation
    while True:
        ret, frame = camera.read()
        if not ret:
            print("Error: Camera feed not available")
            continue
        
        # First use of the frame: Detection
        # obstacle_on_bend = is_obstacle_in_sector(frame.copy())  # Copy frame for pure detection
        obstacle_on_bend = is_obstacle_in_sector(frame)
        reference_point, pixels_per_mm = detect_reference_object(frame, args)

        if reference_point and pixels_per_mm:
            obstacle_dimensions = calculate_obstacle_dimensions(frame, pixels_per_mm)
            distances = detect_distance(frame, reference_point, pixels_per_mm)
            #cv2.imshow("Image", frame)

            if 'green' in distances and distances['green'] < 186 and not waiting_for_clearance: # Threshold 186 to stop too close to obstacle
                stop_motors()
                waiting_for_clearance = True
                print("Stopped due to close green object.")
                

            elif waiting_for_clearance and 'red' in distances and distances['red'] >= 350 and not overtake_performed:
                    green_width = obstacle_dimensions['width']
                    total_overtake_distance = distances['green'] + obstacle_dimensions['height'] if 'green' in distances else 0
                    if 'green' in distances:  # Ensure 'green' exists before using it
                        if overtake_completed.is_set():  # Check if the previous overtake has completed
                            overtake_needed = True
                            # Start overtaking action only if not already in progress
                            if not overtake_performed:
                                thread = threading.Thread(target=overtake_action, args=(green_width, total_overtake_distance, obstacle_on_bend))
                                thread.start() # Begin Thread for overtake
                                with overtake_condition:
                                    overtake_condition.notify() #notify overtake condition
                                print(f"Initiating overtaking maneuver. Obstacle on bend: {obstacle_on_bend}")
                    else:
                        print("No green object detected; cannot calculate overtaking distance.")


            elif overtake_performed:
                with state_lock, drive_condition:
                    if 'green' not in distances or distances['green'] > 196: # Threshold 196 to allow 10mm before reevaluating overtake
                        drive_enabled = True
                        waiting_for_clearance = False
                        overtake_performed = False      # Resets to False to enable overtake check again
                        drive_condition.notify_all()  # Notify all waiting threads to re-evaluate conditions
                        print("Resuming normal driving.")
                    else:
                        print("Maintaining safe distance from obstacle.")

        else:
            print("Reference object not detected.")


        cv2.imshow("Image", frame)


        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break


    # Cleanup: Stop motors, release camera, close hardware interface
    stop_motors()
    fa2.SetMotors(0,0)
    camera.release()
    cv2.destroyAllWindows()
    fa.ComClose()
    fa2.ComClose()

    for thread in threads:
        thread.join()

if __name__ == "__main__":
    main()