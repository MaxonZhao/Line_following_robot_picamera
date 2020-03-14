import time
import RPi.GPIO as GPIO
import io
#module for dc motors
from adafruit_motorkit import MotorKit

#modules for pi camera
import sys
import cv2
import numpy as np
import os
from picamera.array import PiRGBArray
from picamera import PiCamera

#modules for LCD
import digitalio
import board
from PIL import Image, ImageDraw
import adafruit_rgb_display.st7735 as st7735

#initialize the pi camera
camera = PiCamera()
camera.resolution = (640, 368)
camera.rotation = 180
#use PIGRBArray to store captured iamges data in a 3D array 
rawCapture = PiRGBArray(camera, size=(640, 368))

# create a stream to store captured images
stream = io.BytesIO()

#Setting LEDs' corresponding pins as GPIO outputs
GPIO.setmode(GPIO.BCM)  
GPIO.setwarnings(False)
GPIO.setup(26, GPIO.OUT)
GPIO.setup(19, GPIO.OUT)
GPIO.setup(6, GPIO.OUT)


def get_display_driver():
    """
    Get the dispay driver
    @return disp: display driver on which we display image
    """
    # Config for display baudrate (default max is 24mhz):
    BAUDRATE = 24000000

    # Configuration for CS and DC pins
    cs_pin = digitalio.DigitalInOut(board.CE0)
    dc_pin = digitalio.DigitalInOut(board.D17)
    reset_pin = digitalio.DigitalInOut(board.D27)

    # Setup SPI bus using hardware SPI:
    spi = board.SPI()

    #initialize st7735 display driver for 1.44" TFT
    disp = st7735.ST7735R(spi, rotation=270, height=128, x_offset=2, y_offset=3, cs=cs_pin, dc=dc_pin, rst=reset_pin, baudrate=BAUDRATE)  
    return disp

def displayRotate(disp):
    """
    Get the current rotation setting of the display
    if it is 90 or 270 degrees, swap the height and the width
    @param disp: display driver on which we display image
    @return width,height: disp's displaying height and width after certain rotation
    """
    if disp.rotation % 180 == 90:
        height = disp.width  
        width = disp.height
    else:
        width = disp.width  
        height = disp.height
    return width,height

def display_image(image,disp,width,height,imageFile):
    """
    Display image to LCD
    @param disp: display driver on which we display image
    @param width: disp's displaying width
    @param height: disp's displaying height
    @param imageFile: image file's path
    """
    # Get drawing object to draw on image.
    draw = ImageDraw.Draw(image)
    # Draw a black filled box to clear the image.
    draw.rectangle((0, 0, width, height), outline=0, fill=(0, 0, 0))
    disp.image(image)
    # Create new blank image for drawing
    image = Image.open(imageFile)
    image = image.resize((width, height), Image.BICUBIC)
    disp.image(image)

images = []
def create_canvas(width, height):
    """
    create a canvas where we can put image on 
    @param width:disp's displaying width
    @param height:disp's displaying height
    @return image: canvas where we can put image on 
    """
    image = Image.new('RGB', (width, height))
    return image

def compute_error():
    """
    detect edges of black line and compute how far away the robot to the line in terms of both horizontal distance
    and angles
    @return: error, how far the robot is away from the black line
    """
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # camera.capture(stream,format='jpeg',resize=(128,128), use_video_port = True, splitter_port = 2)
        # convert frame to ndArray
        image = frame.array

        # stores every image in images list
        images.append(image)
        
        # default error is 1000 which defaults no lines are detected
        err = 1000
        x_last = 320
        y_last = 180
        
        # extract all black lines in pre-determined range
        Blackline = cv2.inRange(image, (0,0,0), (75,75,75)) 
        kernel = np.ones((3,3), np.uint8)
        Blackline = cv2.erode(Blackline, kernel, iterations=5)
        Blackline = cv2.dilate(Blackline, kernel, iterations=9) 
        # find contour lines along the black lines
        contours_blk, hierarchy_blk = cv2.findContours(Blackline.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        contours_blk_len = len(contours_blk)
        
        # display the center point of the image represented by three circles of red, green, blue
        cv2.circle(image,(320,184),20,(0,0,255),5)
        cv2.circle(image,(320,184),10,(0,255,0),5)
        cv2.circle(image,(320,184),2,(255,0,0),5)
        
        # filter out unqualified black lines by testing the length of the edge
        if contours_blk_len > 0 :
        
         if contours_blk_len == 1 :
          blackbox = cv2.minAreaRect(contours_blk[0])
         else:
           canditates=[]
           off_bottom = 0      
           for con_num in range(contours_blk_len):      
            blackbox = cv2.minAreaRect(contours_blk[con_num])
            (x_min, y_min), (w_min, h_min), ang = blackbox      
            box = cv2.boxPoints(blackbox)
            (x_box,y_box) = box[0]
            if y_box > 358 :         
             off_bottom += 1
            canditates.append((y_box,con_num,x_min,y_min))      
           canditates = sorted(canditates)
           if off_bottom > 1:       
            canditates_off_bottom=[]
            for con_num in range ((contours_blk_len - off_bottom), contours_blk_len):
               (y_highest,con_highest,x_min, y_min) = canditates[con_num]       
               total_distance = (abs(x_min - x_last)**2 + abs(y_min - y_last)**2)**0.5
               canditates_off_bottom.append((total_distance,con_highest))
            canditates_off_bottom = sorted(canditates_off_bottom)         
            (total_distance,con_highest) = canditates_off_bottom[0]         
            blackbox = cv2.minAreaRect(contours_blk[con_highest])      
           else:        
            (y_highest,con_highest,x_min, y_min) = canditates[contours_blk_len-1]       
            blackbox = cv2.minAreaRect(contours_blk[con_highest])    
         (x_min, y_min), (w_min, h_min), ang = blackbox
         x_last = x_min
         y_last = y_min
        
         # compute the angle between the robot and the line 
         if ang < -45 :
          ang = 90 + ang
         if w_min < h_min and ang > 0:    
          ang = (90-ang)*-1
         if w_min > h_min and ang < 0:
          ang = 90 + ang      
         setpoint = 320
         
         # compute the error term respect to the center line
         error = int(x_min - setpoint) 
         ang = int(ang)  
         box = cv2.boxPoints(blackbox)
         box = np.int0(box)
         cv2.drawContours(image,[box],0,(0,0,255),3)    
         #put computed error and angle values to the displayed window  
         cv2.putText(image,str(ang),(10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
         cv2.putText(image,str(error),(10, 320), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
         cv2.line(image, (int(x_min),200 ), (int(x_min),250 ), (255,0,0),3)
         if error <= 300 or error >= -300:
            err = error        
         
        #display captured image with red contour box and blue center point to opencv's window 
        cv2.imshow("orginal with line", image)  
        rawCapture.truncate(0)
    
        return err



# frequency of PID output updates
sample_time=0.0002

# maximum and minimum motor powers allowed 
min_limit = -75
max_limit = 75

def SetMotor(motor_number, speed):
    """
    run a motor to a given speed 
    @param motor_number: the motor we want to run. 1 for left motor and 2 for right motor
    @param speed: speed to run the given motor
    """
    # object of motors
    run = MotorKit()

    # map power value to 0-100 scale
    value = speed/100.0
    
    # control the left motor
    if(motor_number ==  1): 
        run.motor2.throttle = value * 0.99
    # control the right motor
    elif(motor_number == 2):
        run.motor1.throttle = -value
        
def move(speedL, speedR):
    """
    move 2 dc motors according to given speed
    @param speedL:speed to the left motor
    @param speedR:speed to the right motor
    """
    
    # check parameter to limit the speed not to exceed min and max power allowed
    if speedL > max_limit:
        speedL = max_limit
    elif speedL < min_limit:
        speedL = min_limit
        
    if speedR > max_limit:
        speedR = max_limit
    elif speedR < min_limit:
        speedR = min_limit
    
        
    SetMotor(2, speedL)
    SetMotor(1, speedR)
    #if going straight, delay to avoid OSError
    if (speedL == speedR):
        time.sleep(sample_time)





class PID(object):
    """
    A simple PID controller. No fuss.
    """

    def __init__(self,
                 Kp = 1.0, Ki = 0.0, Kd = 0.0,
                 exp = 0,
                 sample_time=0.01,
                 min_limit = None, 
                 max_limit = None):
        """
        @param Kp: The value for the proportional gain Kp
        @param Ki: The value for the integral gain Ki
        @param Kd: The value for the derivative gain Kd
        @param exp: The expected value the PID will try to achieve 
        @param sample_time: The time in seconds which the controller should wait before generating a new output value. --> dt
                            The PID works best when it is constantly called (eg. during a loop), but with a sample
                            time set so that the time difference between each update is (close to) constant. If set to
                            None, the PID will compute a new output value every time it is called.
        @param min_limit: The lower bound of output. The output will never go below this limit. 
                            min_limit can also be set to None to have no limit in that direction. 
        @param max_limit: The upper bound of output. The output will never go above tis limit.
                            max_limit can also be set to None to have no limit in that direction. 
        """
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.exp = exp
        self.sample_time = sample_time

        self._min_output = min_limit
        self._max_output = max_limit

        self.reset()

    def compute_output(self, act, dt=None):
        """
        Call the PID controller with current actual value, calculate and return a control output if sample_time seconds has
        passed since the last update. If no new output is calculated, return the previous output (or None if
        no value has been calculated yet).
        @param act: the actual value
        @param dt: If set, uses this value for timestep instead of real time. 
        @return: the ouput torque for the motor
        """
        now = time.process_time()
        if dt is None:
            dt = now - self._last_time if now - self._last_time else 1e-16
        elif dt <= 0:
            print ("dt has nonpositive value {}. Must be positive.".format(dt))

        if self.sample_time is not None and dt < self.sample_time and self._last_output is not None:
            # only update every sample_time seconds
            return self._last_output

        # compute error terms
        error = self.exp - act
        d_input = act - (self._last_input if self._last_input is not None else act)
        

        # compute the proportional term
        # regular proportional-on-error, simply set the proportional term
        self._proportional = self.Kp * error
        
        # compute integral and derivative terms
        if error is not 0:
            self._integral += self.Ki * error * dt
        else :
            # clear the integral term if the robot is on the line
            self._integral = 0
        
        if self._integral > max_limit:
            self._integral = max_limit
        elif self._integral < min_limit:
            self._integral = min_limit
        
        self._derivative = -self.Kd * d_input / dt

        # compute final output
        output = self._proportional + self._integral + self._derivative
        
        if output > max_limit:
            output = max_limit
        elif output < min_limit:
            output = min_limit

        # keep track of state
        self._last_output = output
        self._last_input = act
        self._last_time = now

        return output


    def get_PID(self):
        """
        The P-, I- and D-terms from the last computation as separate components as a tuple. Useful for visualizing
        what the controller is doing or when tuning hard-to-tune systems.
        """
        return self._proportional, self._integral, self._derivative


    def get_PID_constants(self):
        """Return the constants used by the controller """
        return self.Kp, self.Ki, self.Kd

    def set_PID_constants(self, Kp, Ki, Kd):
        """
        Setter for the PID constants
        @param Kp: The value for the proportional gain Kp
        @param Ki: The value for the integral gain Ki
        @param Kd: The value for the derivative gain Kd
        """
        self.Kp = Kp
        self.Ki = Ki 
        self.Kd = Kd

    def get_output_limits(self):
        """Return the current output limits"""
        return self._min_output, self._max_output

    def reset(self):
        """
        Reset the PID controller internals, setting each term to 0 as well as cleaning the integral,
        the last output and the last input (derivative calculation).
        """
        self._proportional = 0
        self._integral = 0
        self._derivative = 0

        self._last_time = time.process_time()

        self._last_output = None
        self._last_input = None



# PID constants
Kp_L = 0.14
Ki_L = 0.0
Kd_L = 0.002
exp_L = 0

#create a PID object
PID_line = PID(Kp_L,Ki_L,Kd_L,exp_L,sample_time,0,0)

#the motor's default forward speed
forward_speed = 45

#setting up LCD display driver and create a canvas ready to put images on
disp = get_display_driver()
width,height = displayRotate(disp)
canvas = create_canvas(width, height)
#display an image of one of our group member during line tracking with pi camera process
display_image(canvas,disp,width,height,"/home/pi/Desktop/Kush.bmp")

# range of x coordinates that defined as on the line
tol = 35
while True:
    #turn on LED for better recognition
    GPIO.output(26,True)
    GPIO.output(19,True)
    GPIO.output(6, True)
  
    # compute err term for line detection
    err = compute_error()
    
    # compute torque based on current error term
    torque = PID_line.compute_output(err)
    
    # make error term to be 0 within the width of the black line
    if err > 0 and err > tol:
        err = err - tol
    if err < 0 and err < tol:
        err = err + tol
    print ("err = ", err," torque = ",torque)
    if err == 0:
        move(forward_speed, forward_speed)
    elif err > 500:  # no lines are detected, play video recorded along the line following process
        # simply stop in this case   
        move(0,0)
        # iterate trough each ndarray, converting them into an image and display 
        for image in images:
            im = Image.fromarray(image)
            im = im.resize((128, 128), Image.BICUBIC)
            disp.image(im)
            time.sleep(0.02)
        display_image(canvas,disp,width,height,"/home/pi/Desktop/stop_sign.png")
        #when finished displaying recorded images, flush the images array to prevent overflow
        images.clear()
    elif err > 0: 
        # line is on the left, torque is -, turn left
        move(40,40-torque)
    else:
        
        # line is on the right, torque is +, turn right
        move(40+torque, 40)

    #quit the program with tapping key 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything is done, release the capture 
cv2.destroyAllWindows()

