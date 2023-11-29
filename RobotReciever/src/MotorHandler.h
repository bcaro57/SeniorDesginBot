#ifndef MOTOR_HANDLER_H
#define MOTOR_HANDLER_H


class MotorClosedLoop{
    public:
        MotorClosedLoop(MotorDriver* _motor, Encoder* _encoder);

        void init();

    private:
        MotorDriver* Motor;
        Encoder* MotorEncoder;
};



enum class Direction{
        Forward,
        Reverse
};

class MotorDriver{

    public:
        MotorDriver(int _lpwm, int _rpwm);

        void init();
        void setDirection(Direction dir);
        float setSpeed(int percent);
        void setVelocity(int percent);
    private:

        int LPWM;
        int RPWM;
        int speed;

        Direction current_direction;
        // Adafruit_PCF8575* pcf;
};

class MotorControl{

    public:
        MotorControl(MotorDriver* _LeftMotor, MotorDriver* _MiddleMotor, MotorDriver* _RightMotor);

        void init();
        void setSpeed(int percent);
        void setMotors(bool LeftOn, bool MiddleOn, bool RightOn);
        void update(uint8_t buf, int MotorState);
    private:

        MotorDriver* LeftMotor;
        MotorDriver* MiddleMotor;
        MotorDriver* RightMotor;

        int speed;

        const int wheelbase_coeff = 1;

};

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
        Encoder(int _pulse_a, int _pulse_b);

        void wheelSpeed();

        void init();
        long getPosition();
        int getVelocity();
    private:
        int pulseA;
        int pulseB;
        Direction direction;
};


#endif