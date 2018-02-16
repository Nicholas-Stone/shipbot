# I2C with the Arduino as the master device and the OpenMV Cam as the slave.
#
# Please wire up your OpenMV Cam to your Arduino like this:
#
# OpenMV Cam Master I2C Data  (P5) - Arduino Uno Data  (A4)
# OpenMV Cam Master I2C Clock (P4) - Arduino Uno Clock (A5)
# OpenMV Cam Ground                - Arduino Ground

import pyb, ustruct, sensor, image, time, math

from pyb import Pin

text = "Hello World!\n"
data = ustruct.pack("<%ds" % len(text), text)
# Use "ustruct" to build data packets to send.
# "<" puts the data in the struct in little endian order.
# "%ds" puts a string in the data stream. E.g. "13s" for "Hello World!\n" (13 chars).
# See https://docs.python.org/3/library/struct.html

# READ ME!!!
#
# Please understand that when your OpenMV Cam is not the I2C master it may miss responding to
# sending data as a I2C slave no matter if you call "i2c.send()" in an interupt callback or in the
# main loop below. When this happens the Arduino will get a NAK and have to try reading from the
# OpenMV Cam again. Note that both the Arduino and OpenMV Cam I2C drivers are not good at getting
# unstuck after encountering any I2C errors. On the OpenMV Cam and Arduino you can recover by
# de-initing and then re-initing the I2C peripherals.

# The hardware I2C bus for your OpenMV Cam is always I2C bus 2.
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
bus.deinit() # Fully reset I2C device...
bus = pyb.I2C(2, pyb.I2C.SLAVE, addr=0x12)
print("Waiting for Arduino...")

# Note that for sync up to work correctly the OpenMV Cam must be running this script before the
# Arduino starts to poll the OpenMV Cam for data. Otherwise the I2C byte framing gets messed up,
# and etc. So, keep the Arduino in reset until the OpenMV Cam is "Waiting for Arduino...".

##################
#### NEW CODE ####
##################

#COLOR DETECTION SETUP
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(60)
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock()

#GLOBALS
color = ""
avgAspectRatio = 0;
avgArea = 0;
avgXPos = 0;
avgYPos = 0;
returnData = "";
v3Pos = "";
largestArea = 0
largestAspectRatio = 0
largestX = 0
largestY = 0
avgGreenX = 0
avgGreenY = 0
center = 98
error = 2
arduinoCommStateCurr = 0;
aarduinoCommStatePast = 0;
angle = 0
p_in = Pin('P6', Pin.IN)


#TRANSMIT FUNCTION: sends data to Arduino
def send( str ):
    data = ustruct.pack("<%ds" % len(str), str)
    try:
        bus.send(ustruct.pack("<h", len(data)), timeout=10000) # Send the len first (16-bits).
        try:
            bus.send(data, timeout=10000) # Send the data second.
            #print("DATA SENT SENT SENT SENT!") # Only reached on no error.
        except OSError as err:
            pass # Don't care about errors - so pass.
            # Note that there are 3 possible errors. A timeout error, a general purpose error, or
            # a busy error. The error codes are 116, 5, 16 respectively for "err.arg[0]".
    except OSError as err:
        pass # Don't care about errors - so pass.
        # Note that there are 3 possible errors. A timeout error, a general purpose error, or
        # a busy error. The error codes are 116, 5, 16 respectively for "err.arg[0]".
    return


#IS CENTERED FUNCTION: checks if the object is centered in the view
#                      if the object isnt centered updates returnData w/ direction
#                      needed to move to center object

def isCentered():
    global avgXPos
    global returnData
    #print("AVG X POS: ")
    #print(avgXPos)
    #object in center of view
    if((avgXPos > (center - error)) and (avgXPos < (center + error))):
        #print("CENTERED")
        returnData = "C";
    #object on left edge of view
    elif(avgXPos < center):
        if(abs(avgXPos - center) > 20):
            #fast
            #print("MOVE RIGHT FAST")
            returnData = "R";
        elif(abs(avgXPos - center) > 7):
            #medium
            #print("MOVE RIGHT MEDIUM")
            returnData = "E";
        else:
            #print("MOVE RIGHT SLOW")
            #slow
            returnData = "H";
    #object on right edge of view
    else:
        if(abs(avgXPos - center) > 20):
            #fast
            #print("MOVE LEFT FAST")
            returnData = "L";
        elif(abs(avgXPos - center) > 10):
            #medium
            #print("MOVE LEFT MEDIUM")
            returnData = "T";
        else:
            #slow
            #print("MOVE LEFT SLOW")
            returnData = "K";
    return;


#V1 ORIENTATION: checks if we are centered on V1 and and updates
#                   data with orientation /***

def v1Orientation():
    #print(avgAspectRatio)
    color
    avgAspectRatio
    global returnData

    #print(avgAspectRatio)
    #print(color)
    #check if the valve is V1 in Horiz. orientation
    if((avgAspectRatio > .85) and (avgAspectRatio < 1.25)):
        returnData += "H"
        #print("V1 HORIZONTAL")
    #check if the valve is V1 in Vert. orientation
    elif((avgAspectRatio > .25) and (avgAspectRatio < 0.4)):
        returnData += "V"
        #print("V1 VERTICAL")
    else:
        returnData += "N"
    return;


#V3 ORIENTATION: checks if we are centered on V1 and and updates
#                data with orientation

def v3Orientation():
    color
    avgAspectRatio
    avgArea
    global returnData
    global v3Pos

    #check if the valve is V3 in Horiz. orientation
    if(((avgAspectRatio > 2.2) and (avgAspectRatio < 4.0)) or ((avgAspectRatio > .27) and (avgArea>700) and (avgAspectRatio < .45))):
        returnData += "H"
        if((avgAspectRatio > 2.2) and (avgAspectRatio < 3.5)):
            v3Pos = "0"
        else:
            v3Pos = "1"
        #print("V3 HORIZONTAL")
    #check if the valve is V3 in Vert. orientation
    elif(((avgAspectRatio > .05) and (avgAspectRatio < .2)) or ((avgAspectRatio > .35) and (avgAspectRatio < .7))):
        returnData += "V"
        if(((avgAspectRatio > .35) and (avgAspectRatio < .7))):
            v3Pos = "0"
        else:
            v3Pos = "1"
        #print("V3 VERTICAL")
    else:
        returnData += "N"
    return;


#VALVE ANGLE: calculates the initial angle of the vale from center

def valveAngle():
    angle = 0
    diffX = avgGreenX - avgXPos
    diffY = avgGreenY - avgYPos #used to ultiply by -1
    global returnData

    denom1 = float(math.sqrt((diffX*diffX) + (diffY*diffY)))

    if(denom1 != 0):
        denom2 = math.acos(diffY/denom1)
    else:
        denom2 = 1

    angle = math.degrees(denom2)

    if(diffX > 0):
        angle = 360 - angle

    angle = angle + 100

    returnData += str(int(angle))
    print("ANGLE: ")
    print(str(int(angle-100)))

    return


# MAIN FUNCTION: waits until recieves command from arduino than finds the largest blob in the center
#                of the screen and averages center x value  and aspect ratio of it 5 times

while(True):
    clock.tick()
    img = sensor.snapshot()
    returnData = "";
    color = ""
    avgAspectRatio = 0;
    avgXPos = 0;
    avgYPos = 0;
    avgGreenX = 0
    avgGreenY = 0
    #gate:
    #ball Horizontal: (0, 72, -128, 57, -128, -28)
    #ball vertical: (0, 50, -3, 21, -63, -38)
    #shuttlecock:
    #rotory: (0, 92, 11, 73, 6, 47)
    #breakers: (0, 100, 18, 127, 9, 127)
    #direction dot: (57, 93, -32, -15, -2, 20)
    #direction dot: (68, 78, -25, -10, 19, 31)


    #read arduino value

    arduinoCommStatePast = arduinoCommStateCurr
    arduinoCommStateCurr = p_in.value() # get value, 0 or 1
    a = 1;


    #if((arduinoCommStateCurr == 1) and (arduinoCommStatePast == 0)):
    if(a == 1):
        for x in range(0,10):
            img = sensor.snapshot()
            done = 0;
            count = 0;
            for blob in img.find_blobs([(0, 72, -128, 57, -128, -28),(57, 93, -32, -15, -2, 20), (0, 100, 18, 127, 9, 127),(5, 65, 5, 42, -128, -30)], merge = False, margin = 5):
                aspectRatio = blob.h()/blob.w()
                area = blob.area()
                centerX = blob.cx()
                centerY = blob.cy()

                if(blob.code() == 1):
                    area = area
                    #img.draw_rectangle(blob.rect(), color = (0,0,255))
                elif(blob.code() == 2):
                    #print("GREEN");
                    avgGreenY = centerY
                    avgGreenX = centerX
                    img.draw_rectangle(blob.rect(), color = (0,255,0))
                elif(blob.code() == 4):
                    area = area
                    #img.draw_rectangle(blob.rect(), color = (255,133,0))

                #check if box is min. area and near center
                if((area > 100) and (centerX >= 50) and (centerX <= 280)):
                    img.draw_rectangle(blob.rect(), color = (255,133,0))
                    if(count == 0):
                        largestAspectRatio = aspectRatio
                        largestArea = area
                        largestX = centerX
                        largestY = centerY
                    elif(area > largestArea):
                        largestAspectRatio = aspectRatio
                        largestArea = area
                        largestX = centerX
                        largestY = centerY

            avgAspectRatio = avgAspectRatio + largestAspectRatio
            avgArea = avgArea + largestArea
            avgXPos = avgXPos + largestX
            avgYPos = avgYPos + largestY

        avgAspectRatio = avgAspectRatio/10
        avgArea= avgArea/10
        avgXPos = avgXPos/10
        avgYPos = avgYPos/10
        #avgGreenX = avgGreenX/5
        #avgGreenY = avgGreenY/5
        print(avgArea)
        print(avgAspectRatio)

        isCentered()
        v1Orientation()
        v3Orientation()
        valveAngle()
        returnData += v3Pos
        send(returnData)
        print("RETURN DATA: ")
        print(returnData)
        print("")


