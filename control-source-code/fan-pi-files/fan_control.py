from time import sleep
import RPi.GPIO as GPIO

in1 = 5
in2 = 6
en = 26
temp1 = 1

# Initialize pinout
GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)

# Turn off fan
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)

p=GPIO.PWM(en,1000)
p.start(25)
print("\n")
print("The default speed & direction of motor is LOW & Forward.....")
print("r-run s-stop f-forward b-backward l-low m-medium h-high c-calibrate_sweep e-exit")
print("\n")    

while(1):

    x=input()
    
    if x=='r':
        print("Run")
        if(temp1==1):
         GPIO.output(in1,GPIO.HIGH)
         GPIO.output(in2,GPIO.LOW)
         print("Forward")
         x='z'
        else:
         GPIO.output(in1,GPIO.LOW)
         GPIO.output(in2,GPIO.HIGH)
         print("Backward")
         x='z'


    elif x=='s':
        print("Stop")
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        x='z'

    # elif x=='f':
    #     print("forward")
    #     GPIO.output(in1,GPIO.HIGH)
    #     GPIO.output(in2,GPIO.LOW)
    #     temp1=1
    #     x='z'

    # elif x=='b':
    #     print("backward")
    #     GPIO.output(in1,GPIO.LOW)
    #     GPIO.output(in2,GPIO.HIGH)
    #     temp1=0
    #     x='z'

    elif x=='l':
        print("Low")
        p.ChangeDutyCycle(25)
        x='z'

    elif x=='m':
        print("Medium")
        p.ChangeDutyCycle(50)
        x='z'

    elif x=='h':
        print("High")
        p.ChangeDutyCycle(75)
        x='z'
     
    
    elif x=='e':
        GPIO.cleanup()
        break

    # Symmetry calibration sweep from min to max speeds
    elif x=='c':
        print("Starting sweep...")
        # Reset 
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)
        p.ChangeDutyCycle(95)
        sleep(5)
        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        sleep(25)

        # Begin sweep
        GPIO.output(in1,GPIO.HIGH)
        GPIO.output(in2,GPIO.LOW)

        # p.ChangeDutyCycle(50)
        for i in range(0,81):
            p.ChangeDutyCycle(i)
            sleep(0.5)
        print("Sweep done.")

        GPIO.output(in1,GPIO.LOW)
        GPIO.output(in2,GPIO.LOW)
        x='z'
    
    else:
        print("<<<  wrong data  >>>")
        print("Please enter the defined data to continue.....")