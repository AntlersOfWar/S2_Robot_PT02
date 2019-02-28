#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHRPS.h>
#include <FEHServo.h>
#define WHEEL_RADIUS 1.375
#define COUNTS_PER_REV 48
#define PI 3.14159265
#define ROBOT_RADIUS 4.0
#define OFFSET 8.0
/* TODO: Add more defines possibly for controlled servo rotation. */

float redThresh = 0.7;

/* TODO: Make sure you have the right number of motors/encoders declared. */
FEHServo main_servo(FEHServo::Servo7);
FEHServo lever_servo(FEHServo::Servo6);

//P3_6, and P3_7 cannot be used for digital encoders.
//fl means front-left, br means back-right.
DigitalEncoder fr_encoder(FEHIO::P0_0);
DigitalEncoder bl_encoder(FEHIO::P3_0);

FEHMotor fr_motor(FEHMotor::Motor3, 5.0);
FEHMotor bl_motor(FEHMotor::Motor0, 5.0);

AnalogInputPin cds(FEHIO::P0_4);

// Global bool for orientation swapping
bool vertical;

int theoreticalCounts(int inches) {
    int counts = (inches * COUNTS_PER_REV) / (2 * PI * WHEEL_RADIUS);
    return counts;
}

int theoreticalDegree(int degrees){
    /*
    int counts = (degrees * COUNTS_PER_REV * ROBOT_RADIUS) / (2 * PI * WHEEL_RADIUS);
    */
    float radians = degrees * PI / 180;
    int arclength = ROBOT_RADIUS * radians;
    return theoreticalCounts(arclength);
}

//Here 'move_forward' means positive movement.
/* IMPORTANT: Our coordinate system for this program is a top-down view of the course, with the starting point as the origin.
 * DDR is in positive X and lever is in positive Y. */
void move_forward(int percent, int inches)
{
    //Reset all encoder counts
    fr_encoder.ResetCounts();
    bl_encoder.ResetCounts();

    int counts = theoreticalCounts(inches);
    int actualCts = 0;

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    //Some motors have to turn backwards depending on the orientation, also.

    //While the average of the left or right encoders is less than theoretical counts,
    //keep running motors

    if(vertical){
        fr_motor.SetPercent(-percent);
        bl_motor.SetPercent(percent);
    }
    else{
        fr_motor.SetPercent(percent);
        bl_motor.SetPercent(-percent);
    }

    while(fr_encoder.Counts() < counts || bl_encoder.Counts() < counts){
        LCD.Clear();
        LCD.Write("Moving forward ");
        LCD.Write(inches);
        LCD.WriteLine(" inches");
        LCD.Write("THEORETICAL COUNTS: ");
        LCD.WriteLine(theoreticalCounts(inches));
        LCD.Write("Actual BLE Counts: ");
        LCD.WriteLine(bl_encoder.Counts());
        LCD.Write("Actual FRE Counts: ");
        LCD.WriteLine(fr_encoder.Counts());
    }

    //Turn off motors
    fr_motor.Stop();
    bl_motor.Stop();
}

//Same as move_forward, but percents are multiplied by -1 (reverse).
void move_backward(int percent, int inches)
{
    //Reset all encoder counts
    fr_encoder.ResetCounts();
    bl_encoder.ResetCounts();

    int counts = theoreticalCounts(inches);

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    //Some motors have to turn backwards depending on the orientation, also.
    if(vertical){
        fr_motor.SetPercent(percent);
        bl_motor.SetPercent(-percent);
    }
    else{
        fr_motor.SetPercent(-percent);
        bl_motor.SetPercent(percent);
    }

    //While the average of the left and right encoders is less than theoretical counts,
    //keep running motors
    while(fr_encoder.Counts() < counts || bl_encoder.Counts() < counts){
        LCD.Clear();
        LCD.Write("Moving backwards ");
        LCD.Write(inches);
        LCD.WriteLine(" inches");
        LCD.Write("THEORETICAL COUNTS: ");
        LCD.WriteLine(theoreticalCounts(24));
        LCD.Write("Actual BLE Counts: ");
        LCD.WriteLine(bl_encoder.Counts());
        LCD.Write("Actual FRE Counts: ");
        LCD.WriteLine(fr_encoder.Counts());
    }

    //Turn off motors
    fr_motor.Stop();
    bl_motor.Stop();
}

void turnLeft(int percent, int degrees){
    //Reset all encoder counts
    fr_encoder.ResetCounts();
    bl_encoder.ResetCounts();

    int counts = theoreticalDegree(degrees);

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    //Some motors have to turn backwards depending on the orientation, also.
    fr_motor.SetPercent(-percent);
    bl_motor.SetPercent(-percent);

    //While the average of the left and right encoders is less than theoretical counts,
    //keep running motors
    while(fr_encoder.Counts() < counts || bl_encoder.Counts() < counts){
        LCD.Clear();
        LCD.Write("Turning left ");
        LCD.Write(degrees);
        LCD.WriteLine(" degrees");
        LCD.Write("THEORETICAL COUNTS: ");
        LCD.WriteLine(theoreticalCounts(degrees));
        LCD.Write("Actual BLE Counts: ");
        LCD.WriteLine(bl_encoder.Counts());
        LCD.Write("Actual FRE Counts: ");
        LCD.WriteLine(fr_encoder.Counts());
    }

    //Turn off motors
    fr_motor.Stop();
    bl_motor.Stop();
}

void turnRight(int percent, int degrees){
    //Reset all encoder counts
    fr_encoder.ResetCounts();
    bl_encoder.ResetCounts();

    int counts = theoreticalDegree(degrees);

    //Set motors to desired percent. Some motors have to turn backwards, so make percent negative.
    //Some motors have to turn backwards depending on the orientation, also.
    fr_motor.SetPercent(percent);
    bl_motor.SetPercent(percent);

    //While the average of the left and right encoders is less than theoretical counts,
    //keep running motors
    while(fr_encoder.Counts() < counts || bl_encoder.Counts() < counts){
        LCD.Clear();
        LCD.Write("Turning right ");
        LCD.Write(degrees);
        LCD.WriteLine(" degrees");
        LCD.Write("THEORETICAL COUNTS: ");
        LCD.WriteLine(counts);
        LCD.Write("Actual BLE Counts: ");
        LCD.WriteLine(bl_encoder.Counts());
        LCD.Write("Actual FRE Counts: ");
        LCD.WriteLine(fr_encoder.Counts());
    }

    //Turn off motors
    fr_motor.Stop();
    bl_motor.Stop();
}

//Swap orientation of wheels
//Allow slower, controlled rotation of the main servo. Increase step for faster rotation
//Input true if the wheels are vertically aligned with field, false if horizontally aligned
void swapOrientation(float step) {
    float posVert = 131.0, posHoriz = 41.0, currentPos;

    if(vertical){
        // Init servo
        currentPos = posVert;

        fr_motor.SetPercent(-15);
        bl_motor.SetPercent(-15);

        while (currentPos > posHoriz) {
            currentPos = currentPos - step;
            LCD.Clear();
            LCD.Write("Current Position: ");
            LCD.WriteLine(currentPos);
            main_servo.SetDegree(currentPos);
        }

        fr_motor.Stop();
        bl_motor.Stop();
    }
    else{
        //Init servo
        currentPos = posHoriz;

        fr_motor.SetPercent(15);
        bl_motor.SetPercent(15);

        while (currentPos < posVert) {
            currentPos = currentPos + step;
            LCD.Clear();
            LCD.Write("Current Position: ");
            LCD.WriteLine(currentPos);
            main_servo.SetDegree(currentPos);
        }

        fr_motor.Stop();
        bl_motor.Stop();
    }
}

int main()
{
    // min for main servo: 725
    // max for main servo: 2468
    // min for lever servo: 514
    // max for lever servo: 2430

    //Initialize main servo
    main_servo.SetMin(725);
    main_servo.SetMax(2468);

    //Starting orientation
    vertical = true;

    move_forward(50, 15);
    Sleep(2000);
    swapOrientation(2.0);
    Sleep(2000);
    vertical = false;
    move_forward(50, 15);
    Sleep(2000);
    swapOrientation(2.0);
    Sleep(2000);
    vertical = true;
    move_backward(50, 15);
    Sleep(2000);
    swapOrientation(2.0);
    Sleep(2000);
    vertical = false;
    move_backward(50, 15);

    /* CdS cell
    //Initialize the screen
    LCD.Clear(BLACK);
    LCD.SetFontColor(WHITE);

    lever_servo.SetDegree(120.0);

    while(cds.Value() > redThresh) {
        LCD.Clear();
        LCD.WriteLine("Looking for Red Light...");
        LCD.WriteLine(cds.Value());
    }
    */


}
