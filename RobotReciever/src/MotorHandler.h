#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H


/*
Direction - allows us to keep track of if the direction of a given object is forward or reversed
*/


enum class Direction{
        Forward,
        Reverse
};


/*
MotorDriver - designed to control the PWM of a motor and drive it in a desired manner

inputs into contructor:  the pwm pins for lpwm and rpwm (can be seen on the physical motor driver which is the BTS7960)

methods: 
    init()              -> initializes the pins and sets them to 0 to start
    setDirection()      -> sets current_direction to reflect the direction the motor is moving 
    setSpeed()          -> takes a percentage (i.e. from -100 to 100) and determines power needed
    setVelocity()       -> sets the pwm pins to the appropriate value using setSpeed() and chooses the approptiate 
                        direction using  setDirection()
    getDirection()      -> returns the direction that the motor is going

variables:
    LPWM                -> the left PWM pin
    RPWM                -> the right PWM pin
    speed               -> the desired analog speed of the motor
    current_direction   -> the current direction of the motor
    MCP                 -> the IO expansion board object
*/


class MotorDriver{

    public:
        MotorDriver(int _lpwm, int _rpwm, Adafruit_MCP23X17* _mcp = NULL);

        void init();
        void setDirection(Direction dir);
        float setSpeed(int percent);
        void setVelocity(int percent);
        Direction getDirection();
    private:

        int LPWM;
        int RPWM; 
        int speed;

        Direction current_direction;
        Adafruit_MCP23X17* MCP;
};


/*
StepperDriver - designed to control the stepper motor in a desired way using the AccelStepper library

inputs into contructor:  the  pins for pulse and direction, as well as the limit switch sensor

methods: 
    init()              -> initializes the pins and objects, as well as calibrates the stepper motor
    movePosition()      -> moves the weight to the corresponding position, where each position input (1, 2, or 3) 
                        stands for the arm it needs to move to

variables:
    pulsePin            -> the pin in charge of pulses of the stepper motor
    dirPin              -> the pin in charge of the direction of the stepper motor
    sensorPin           -> the pin in charge of the limit switch for homing the stepper motor
    myStepper           -> object in charge of the stepper motor, using the AccelStepper Library
    calibrated          -> a boolean that tells us if the stepper motor has been calibrated with the limit switch or not
    halfLength          -> the amount of steps needed for the balancer to the halfway position on the robot
    fullLength          -> the amount of steps needed for the balancer to the full position on the robot

*/


class StepperDriver{

    public:
        StepperDriver(int _pulse_pin, int _dir_pin, int _sensor_pin);

        void init();
        void movePosition(int targetPos);

    private:
        int pulsePin;
        int dirPin;
        int sensorPin;
        AccelStepper myStepper;
        bool calibrated;
        int halfLength = 4500;
        int fullLength = 8500;
};


/*
pulsePin - allows us to keep track of if the interrupt pins, specifying the pin used and the 
*/


enum class pulsePin{
        feather0,
        feather1,
        mcp0
};


/*
** currently under construction **
Encoder - designed to handle the encoder from a drive motor. needs to use "glue routines" to get interrupts
          working in the class, which are necessary because the encoder is a quaditure encoder. to do this,
          three instances are created (and set to NULL), and external wheelSpeed (the ISR) functions are
          created with a switch/case that looks for the different available 'pulseA' pins.

inputs into the constructor: the 'pulseA' pin and 'pulseB' pin, which are responsible for interpreting the 
                             data from the quaditure encoder, where the 'pulseA' must be attached to an interrupt
                             pin. it also takes in 'MCP', which is the IO expander. this is defaulted to NULL in 
                             case we have some encoders that aren't hooked up to the expander.

methods:
    wheelSpeed()        -> the interrupt service routine (ISR) for using the encoders
    init()              -> initializes the pulseA/pulseB pins, which change depending on the encoder because
                        of the glue routines
    getPosition()       -> returns the position value from the encoders
    getVelocity()       -> returns the velocity value from the encoders

variables:
    pulseA              -> the pin for the pulseA value, which needs to be attached to an interrupt
    pulseB              -> the pin for the pulseB value
    direction           -> used in the ISR and makes values either add or subtract based on the direction
    MCP                 -> the IO expansion board object
*/


class Encoder{

    int encoder0PinALast;
    int velocity;
    long position;

    static Encoder * instances [3];

    static void wheelSpeedExt0(){
        if (Encoder::instances[0] != NULL){
            Encoder::instances[0] -> wheelSpeed();
        }
    }
    static void wheelSpeedExt1(){
        if (Encoder::instances[1] != NULL){
            Encoder::instances[1] -> wheelSpeed();
        }
    }
    static void wheelSpeedExt2(){
        if (Encoder::instances[2] != NULL){
            Encoder::instances[2] -> wheelSpeed();
        }
    }

    public:
        Encoder(pulsePin _pin_loc, int _pulse_a, int _pulse_b, Adafruit_MCP23X17* _mcp = NULL);

        void wheelSpeed();

        void init();
        long getPosition();
        int getVelocity();
    private:
        pulsePin pinLoc;
        int pulseA;
        int pulseB;
        Direction direction;
        Adafruit_MCP23X17* MCP;
};


/*
DriveControl - designed to control all of the robots drive motors

inputs into constructor: each of the motors (left, middle, and right motors)
                         can be used for any set of three motors

methods:
    init()              -> initializes all of the motors, and sets their velocity to 0. also sets our speed variable to 0
    setSpeed()          -> sets the private speed variable to whatever value we give it (should be a percent -100 to 100)
    update()            -> sets the motors based on the given buffer

variables:
    LeftMotor           -> object of the left motor
    MiddleMotor         -> object of the middle motor
    RightMotor          -> object of the right motor
    speed               -> the desired analog speed of the motors
*/


class DriveControl{

    public:
        DriveControl(MotorDriver* _LeftMotor, MotorDriver* _MiddleMotor, MotorDriver* _RightMotor);

        void init();
        void setSpeed(int percent);
        void update();
    private:

        MotorDriver* LeftMotor;
        MotorDriver* MiddleMotor;
        MotorDriver* RightMotor;

        int speed;


};


/*
ActuatorControl - designed to control all of the actuator motors

inputs into constructor: each of the motors (left, middle, and right motors)
                         can be used for any set of three actuators, as well as 
                         the stepper motor and the expander board.

methods:
    init()              -> initializes all of the motors, and sets their velocity to 0. also sets our speed variable to 0
    setSpeed()          -> sets the private speed variable to whatever value we give it (should be a percent -100 to 100)
    update()            -> sets the motors based on the given buffer

variables:
    LeftMotor           -> object of the left motor
    MiddleMotor         -> object of the middle motor
    RightMotor          -> object of the right motor
    DynamicBalancer     -> object of the stepper motor for the balancer
    speed               -> the desired analog speed of the motors
    left_was_toggled    -> shows if the left motor button was toggled
    middle_was_toggled  -> shows if the left motor button was toggled
    right_was_toggled   -> shows if the left motor button was toggled

*/


class ActuatorControl{

    public:
        ActuatorControl(MotorDriver* _left_motor, MotorDriver* _middle_motor, MotorDriver* _right_motor, StepperDriver* _dynamic_balancer, Adafruit_MCP23X17* _mcp);

        void init();
        void setSpeed(int percent);
        void update(uint8_t buf);
    private:

        MotorDriver* LeftMotor;
        MotorDriver* MiddleMotor;
        MotorDriver* RightMotor;
        StepperDriver* DynamicBalancer;
        Adafruit_MCP23X17* MCP;

        int speed;

        unsigned long LeftEventTime;
        unsigned long MiddleEventTime;
        unsigned long RightEventTime;

        int waitTime = 2000;
};


/*
** currently under construction **
MotorClosedLoop - designed for closed loop motor control

inputs into constructor: a motor and an associated encoder

methods:
    init()              -> initializes the motor and encoder objects

variables:
    Motor               -> the motor in question
    MotorEncoder        -> the encoder in question
*/

class MotorClosedLoop{
    public:
        MotorClosedLoop(MotorDriver* _motor, Encoder* _encoder);

        void init();

    private:
        MotorDriver* Motor;
        Encoder* MotorEncoder;
};


#endif