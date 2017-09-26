// SunTracker v1.0
// 26 August 2017, Benjamin Shanahan.
//
// Two-axis tracking mechanism. Uses two independently controlled stepper 
// stepper motors and a 2x2 grid of light-dependent resistors (LDR) to
// determine sun position in the sky and rotate to where light is the 
// strongest.

#define SERIAL       true   // print serial values to Arduino serial monitor?
#define DEG_PER_STEP 0.9    // number of degrees per motor step
#define STEP_PER_DEG 1/0.9  // inverse (so we don't need division (slow!))
#define RESOLUTION   8      // stepper board resolution is set to 1/8 of a step
#define DELTA        1      // number of degrees to move per search iteration
#define TOLERANCE    50     // tolerance for LDR equality
#define MIN_ROT      0      // (deg) min rotation for motors
#define MAX_ROT      360    // (deg) max rotation for motors before reset

enum MotorDirection {
    RIGHT = 1, 
    LEFT  = 0, 
    UP    = 1, 
    DOWN  = 0
};

enum MotorPin {
    ROTATE_DIR_PIN  = 3, 
    ROTATE_STEP_PIN = 2,
    TILT_DIR_PIN    = 7,
    TILT_STEP_PIN   = 6
};

enum Motor {
    ROTATE,
    TILT
};

// Define pins for LDRs
enum LDRPins {
    RIGHT_LDR_PIN  = A0,
    LEFT_LDR_PIN   = A1,
    TOP_LDR_PIN    = A2,
    BOTTOM_LDR_PIN = A3
};

// Define error codes
enum Error {
    UNKNOWN_MOTOR,
    DEGREES_CANNOT_BE_NEGATIVE
};

int    rval, lval, tval, bval;  // analogRead samples from LDRs
double currRotMotorDegrees  = (MAX_ROT-MIN_ROT)/2;
double currTiltMotorDegrees = (MAX_ROT-MIN_ROT)/2;

void setup()
{

    // Define LDR pin inputs
    pinMode(RIGHT_LDR_PIN, INPUT);
    pinMode(LEFT_LDR_PIN, INPUT);
    pinMode(TOP_LDR_PIN, INPUT);
    pinMode(BOTTOM_LDR_PIN, INPUT);

    // Define motor pin outputs
    pinMode(ROTATE_DIR_PIN, OUTPUT);
    pinMode(ROTATE_STEP_PIN, OUTPUT);
    pinMode(TILT_DIR_PIN, OUTPUT);
    pinMode(TILT_STEP_PIN, OUTPUT);

    // Start serial (if desired)
    if (SERIAL)
        Serial.begin(9600);

}

// Move motor number of degrees in direction
void move(int motor, int direction, double degrees)
{

    MotorPin stepPin;

    if (degrees <= 0)
        return DEGREES_CANNOT_BE_NEGATIVE;
    
    // Do stuff specific to motor type
    if (motor == ROTATE)
    {
        digitalWrite(ROTATE_DIR_PIN, direction);
        stepPin = ROTATE_STEP_PIN;
    }
    else if (motor == TILT)
    {
        digitalWrite(TILT_DIR_PIN, direction);
        stepPin = TILT_STEP_PIN;
    }
    else
        return UNKNOWN_MOTOR;

    // calculate number of steps to increment based on degrees
    int steps = (int) (degrees * STEP_PER_DEG * RESOLUTION);

    for(int i = 0; i < steps; i++)
    {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(100);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(100);
    }

}

void loop()
{

    // This is a one-axis prototype
    // NOTE: LDR value decreases in bright light
    rval = map(analogRead(RIGHT_LDR_PIN), 50, 950, 0, 1000);
    lval = map(analogRead(LEFT_LDR_PIN), 700, 1000, 0, 1000);

    Serial.println("lval: " + String(lval) + ", rval: " + String(rval));

    // If rval is greater than lval, we want to turn left; if lval is greater
    // than rval, we want to turn right. This is all within a set tolerance
    // level so that once the two sensors are close enough, they stop moving.
    if ((rval - lval) > TOLERANCE)  // rotating LEFT (360 -> 0 deg)
    {
        currRotMotorDegrees -= DELTA;  // we're turning LEFT
        // If our current desired rotation puts us past MIN_ROT, subtract our
        // desired rotation degrees from MAX_ROT and rotate to that position
        // in the opposite direction. This prevents the wires from getting 
        // twisted.
        if (currRotMotorDegrees < MIN_ROT)
        {
            currRotMotorDegrees = MAX_ROT - DELTA;
            move(ROTATE, RIGHT, currRotMotorDegrees);
        }
        else
            move(ROTATE, LEFT, DELTA);
    }
    else if ((lval - rval) > TOLERANCE)  // rotating RIGHT (0 -> 360 deg)
    {
        currRotMotorDegrees += DELTA;  // we're turning RIGHT
        // If our current desired rotation puts us past MAX_ROT, reset to zero
        // (going opposite direction) and continue counting from there. This
        // prevents the wires from getting twisted.
        if (currRotMotorDegrees > MAX_ROT)
        {
            currRotMotorDegrees = (int) currRotMotorDegrees % MAX_ROT;
            move(ROTATE, LEFT, MAX_ROT - currRotMotorDegrees);
        }
        else
            move(ROTATE, RIGHT, DELTA);
    }

}