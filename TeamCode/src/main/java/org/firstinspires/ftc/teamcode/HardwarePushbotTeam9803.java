package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbotTeam9803
{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  ballDrive    = null;
    public DcMotor  pushDrive    = null;
    public Servo    rightBeacon = null;
    public Servo    leftBeacon  = null;
    public ColorSensor frontColor = null;


    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double BEACONLEFT_ZEROED  = 0.0 ;
    public static final double BEACONLEFT_PRESS  = 0.5 ;
    public static final double BEACONRIGHT_ZEROED  = 1.0 ;
    public static final double BEACONRIGHT_PRESS  = 0.5 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor  */
    public HardwarePushbotTeam9803(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("leftDrive");
        rightMotor  = hwMap.dcMotor.get("rightDrive");
        ballDrive    = hwMap.dcMotor.get("ballDrive");
        pushDrive    = hwMap.dcMotor.get("pushDrive");
        leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        leftBeacon  = hwMap.servo.get("leftBeacon");
        rightBeacon = hwMap.servo.get("rightBeacon");


        frontColor = hwMap.colorSensor.get("frontColor");

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        pushDrive.setPower(0);
        ballDrive.setPower(0.0);
        leftMotor.setMaxSpeed(1800);
        rightMotor.setMaxSpeed(1800);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ballDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pushDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBeacon.setPosition(BEACONLEFT_ZEROED);
        rightBeacon.setPosition(BEACONRIGHT_ZEROED);

        frontColor.enableLed(true);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

