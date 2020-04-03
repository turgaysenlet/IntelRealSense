import maestro
servo = maestro.Controller()
servo.setAccel(1,1)      #set servo 0 acceleration to 4
servo.setSpeed(1,4)     #set speed of servo 1
servo.setAccel(2,1)      #set servo 0 acceleration to 4
servo.setSpeed(2,4)     #set speed of servo 1
# servo.setTarget(1,6000)  #set servo to move to center position
# servo.setTarget(2,6000)  #set servo to move to center position
# x = servo.getPosition(1) #get the current position of servo 1
# print(x)
# y = servo.getPosition(1) #get the current position of servo 2
# print(y)

servo.setAccel(0,1)      #set servo 0 acceleration to 4
servo.setSpeed(0,1)     #set speed of servo 1
servo.setTarget(0,6000)  #set servo to move to center position
servo.close()