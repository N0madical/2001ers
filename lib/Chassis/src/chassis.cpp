#include "chassis.h"
#include "Romi32U4MotorTemplate.h"

Romi32U4EncodedMotor<LEFT_XOR, LEFT_B, PWM_L, DIR_L, OCR_L> leftMotor("L");
Romi32U4EncodedMotor<RIGHT_XOR, RIGHT_B, PWM_R, DIR_R, OCR_R> rightMotor("R");

float vMultiplier;
float uMultiplier;
float omegaMultiplier;
const int ticksPerRevolution = 1430;

/**
 * Because it's declared static, we initialize Chassis::loopFlag here.
 */
uint8_t Chassis::loopFlag = 0;

/**
 * For taking snapshots and raising the flag.
 */
void Chassis::Timer4OverflowISRHandler(void) 
{
    loopFlag++;

    leftMotor.speed = leftMotor.CalcEncoderDelta();
    rightMotor.speed = rightMotor.CalcEncoderDelta();

    leftMotor.distance += leftMotor.speed;
    rightMotor.distance += rightMotor.speed;
}

Pose Chassis::GetPosition(void)
{
    Pose position;

    position.x = 0;
    position.y = (leftMotor.distance + rightMotor.distance) / 2;
    position.theta = (rightMotor.distance - leftMotor.distance) / 2;

    return position;
}

/**
 * ISR for timing. On Timer4 overflow, we take a 'snapshot' of the encoder counts 
 * and raise a flag to let the program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
   Chassis::Timer4OverflowISRHandler();
}

/**
 * Sets up a hardware timer on Timer4 to manage motor control on a precise schedule.
 * 
 * We set the timer to set an interrupt flag on overflow, which is handled
 * by ISR(TIMER4_OVF_vect) below.
 */
void Chassis::InitializeMotorControlTimer(void)
{
    Serial.println("InitTimer");
    // Disable interupts while we mess with the Timer4 registers
    cli(); 
  
    // Set up Timer4
    TCCR4A = 0x00; // Disable output to pins
    TCCR4B = 0x0A; // Sets the prescaler -- see pp. 167-8 in datasheet
    TCCR4C = 0x00; // Disables output to pins (but see below for buzzer)
    TCCR4D = 0x00; // Normal mode: count up and roll-over

    /**
     * Calculate TOP based on prescaler and loop duration. Note that loop is in integer ms --
     * there may be some rounding. Multiples of 4 ms will be exact.
     */
    uint16_t top = ((CONTROL_LOOP_PERIOD_MS * 16000ul) >> 9) - 1; // divides by 512

    /**
     * Here we do a little trick to allow full 10-bit register access. 
     * We have 2 _bits_ in TC4H that we can use to add capacity to TOP.
     * 
     * Note that the maximum period is limited by TOP = 0x3FF. If you want
     * a longer period, you'll need to adjust the pre-scaler.
     * 
     * There is no minumum period, but precision breaks down with low values, 
     * unless you adjust the pre-scaler, but the encoder resolution is limited,
     * so you only want to go so fast.
    */
    uint8_t highbits = top / 256;
    uint8_t lowbits = top - highbits;
    TC4H = highbits; OCR4C = lowbits;

    // Enable overflow interrupt
    TIMSK4 = 0x04; 

    /**
     * Uncommenting the following lines will pipe the timer signal to pin 6, 
     * which controls the buzzer. The pin will toggle at the loop rate, which
     * allows you to check that the timing is correct. It will also make a lot
     * of noise, so do so sparingly.
     */
    // TCCR4C = 0x04;
    // pinMode(6, OUTPUT);

    // Re-enable interrupts
    sei(); 

    Serial.println("/InitTimer");
}

void Chassis::InititalizeChassis(void)
{
    InitializeMotorControlTimer();
    InitializeMotors();
    pinMode(YELLOW_PIN, OUTPUT);

    vMultiplier = (PI * WHEEL_DIAMETER) / (2 * ticksPerRevolution);
    uMultiplier = 1 / (ticksPerRevolution * 2);
    omegaMultiplier = (PI * WHEEL_DIAMETER) / (ticksPerRevolution * WHEELBASE);
}

void Chassis::LightOn(void)
{
    digitalWrite(YELLOW_PIN, HIGH);
}

void Chassis::LightOff(void)
{
    digitalWrite(YELLOW_PIN, LOW);
}

/**
 * The main Chassis loop.
 */
bool Chassis::ChassisLoop(Twist& velocity)
{
    bool retVal = false;

    if(loopFlag)
    {
        if(loopFlag > 1) Serial.println("Missed an update in Robot::RobotLoop()!");

        #ifdef __LOOP_DEBUG__
            Serial.print(millis());
            Serial.print('\n');
        #endif

        /* Update the wheel velocity so it gets back to Robot. */
        velocity = CalcOdomFromWheelMotion();

        loopFlag = 0;

        retVal = true;
    }

    return retVal;
}

/**
 * Some motor methods.
 */
void Chassis::InitializeMotors(void)
{
    Romi32U4MotorBase::InitializePWMTimerAndInterrupts();

    leftMotor.InitializeMotor();
    rightMotor.InitializeMotor();
}

void Chassis::SetMotorEfforts(int16_t left, int16_t right) 
{
    leftMotor.SetMotorEffortDirect(left); 
    rightMotor.SetMotorEffortDirect(right);
}


Twist Chassis::CalcOdomFromWheelMotion(void)
{
    Twist velocity;

    //Odom stats:
    //1430 ticks per full wheel rotation
    //20ms/loop
    //motor.speed = ticks per 20ms
    //Wheel diameter = 69mm = 0.069m
    //Wheelbase = 13.8cm = 0.138m

    //Measured in m/dt
    //motor.speed = (leftMotor.speed+rightMotor.speed)/2
    //TPS (ticks per second) = motor.speed * (1second/20ms = 50)
    //RPS = TPS / 1430ticks/rotation
    //v = RPS * (PI * 0.066m = ~0.207 m/rotation)
    //v = (((motor.speed/2)*50)/1430)*0.207
    //Multiplier formula = (PI * 0.066) / (2 * 1430ticks/rev)
    velocity.v = (leftMotor.speed + rightMotor.speed)*vMultiplier;

    //Measured in rad/dt
    // w = 13.3cm = 0.133m
    // Multiplier formula = (PI * 0.069m dia) / (1430 ticks/rev * 0.133m wheelbase)
    velocity.omega = (leftMotor.speed - rightMotor.speed)*omegaMultiplier;

    //Measured in RPdt
    // Multiplier formula = 1 / (1430 ticks/rev * 2) = 0.00035
    // For RPM = 1 / (1430 ticks/rev * 2)
    velocity.u = (leftMotor.speed + rightMotor.speed)*uMultiplier;

    /**
     * Note that you might want to calculate the deltas instead of speeds (to save some floating point maths). 
     * In that case, you should return a Pose instead of a Twist.
     */
    
#ifdef __CHASSIS_DEBUG__
    TeleplotPrintTuple("Speed", leftMotor.speed, rightMotor.speed);
    TeleplotPrintTuple("Distance", leftMotor.GetEncoderTotal(), rightMotor.GetEncoderTotal());
    TeleplotPrint("u", velocity.u);
    TeleplotPrint("omega", velocity.omega);
#endif

    return velocity;
}
