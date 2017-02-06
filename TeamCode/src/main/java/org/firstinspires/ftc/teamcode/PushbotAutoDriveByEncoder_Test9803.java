/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;

@Autonomous(name="Pushbot: 9803 TestBot", group="9803")
public class PushbotAutoDriveByEncoder_Test9803 extends LinearVisionOpMode {
    /* Declare OpMode members. */
    HardwarePushbotTeam9803 robot   = new HardwarePushbotTeam9803();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.9375 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.85;
    static final double     TURN_SPEED              = 0.3;
    public static final double BEACONLEFT_ZEROED    = 0.0;
    public static final double BEACONLEFT_PRESS     = 0.5;
    public static final double BEACONRIGHT_ZEROED   = 1.0;
    public static final double BEACONRIGHT_PRESS    = 0.5;
    public void idle(){
    }
    public void loop(){
        super.loop();

    }
    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        robot.init(hardwareMap);
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d : %7d",
                          robot.leftMotor.getCurrentPosition(),
                          robot.rightMotor.getCurrentPosition()
        );
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //cases

        int x = 1;
        int colorCount = 0;
        encoderDrive(.7, 36, 36, 10);
        encoderDrive(.9, -20.2, 20.2, 10);
        encoderDrive(.9, 32, 32, 10);
        encoderDrive(.9, 15, -15, 10);
        encoderDrive(.9, 3, 3, 10);
        encoderDrive(.9, 5, -5, 10);
        encoderDrive(.9, 50, 50, 10);
        encoderDrive(.9, 33.5, -33.5, 10);
        encoderDrive(.9, 65, 65, 10);
        visionAct(visionFind());
        /*
        switch(x){
            default:
                x = 1;
            case 1: //move forward 10 inches at DRIVE_SPEED.
                encoderDrive(DRIVE_SPEED, 48, 48, 10);
                x=2;
            case 2:
                encoderDrive(TURN_SPEED, 5, 1, 10);
                x=3;


            case 3:
                encoderDrive(DRIVE_SPEED, 45, 45, 10);
                x=4;
            case 4:
                encoderDrive(TURN_SPEED, 1, 5, 10);
                x=5;
            case 5:
                telemetry.addData("Clear", robot.frontColor.alpha());
                telemetry.update();
                if (robot.frontColor.alpha()>= 1){
                    x=6;
                }
                else if (robot.frontColor.alpha()<1){
                    robot.leftMotor.setPower(0.1);
                    robot.rightMotor.setPower(0.1);
                }
            case 6:
                encoderDrive(TURN_SPEED, 3, -3, 8);
                encoderDrive(DRIVE_SPEED, 10, 10, 10);
                boolean beacon = visionFind();
                visionAct(beacon);
                x=7;

                    }

                */

    }
    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = (robot.leftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newRightTarget = (robot.rightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));
            robot.leftMotor.setTargetPosition(newLeftTarget);
            robot.rightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower((speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.leftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d : %7d",
                                            robot.leftMotor.getCurrentPosition(),
                                            robot.rightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    int frameCount = 0;
    int blueCount = 0;
    public boolean visionFind() throws InterruptedException{
        boolean blueLeft;

        waitForVisionStart();

        this.setCamera(Cameras.PRIMARY); //Secondary for front.
        this.setFrameSize(new Size(900, 900));

        enableExtension(Extensions.BEACON);         //Beacon detection
        enableExtension(Extensions.ROTATION);       //Automatic screen rotation correction
        enableExtension(Extensions.CAMERA_CONTROL); //Manual camera control
        beacon.setAnalysisMethod(Beacon.AnalysisMethod.FAST);

        beacon.setColorToleranceBlue(0.0);
        beacon.setColorToleranceRed(0.0);
        rotation.setIsUsingSecondaryCamera(false);
        rotation.disableAutoRotate();
        rotation.setActivityOrientationFixed(ScreenOrientation.PORTRAIT);

        cameraControl.setColorTemperature(CameraControlExtension.ColorTemperature.AUTO);
        cameraControl.setAutoExposureCompensation();

        for (int i = 0; i < 20; i++) { //telemetry additions for camera details
            //Log a few things

            telemetry.addData("Beacon Color", beacon.getAnalysis().getColorString());
            telemetry.addData("Beacon Center", beacon.getAnalysis().getLocationString());
            telemetry.addData("Beacon Confidence", beacon.getAnalysis().getConfidenceString());
            telemetry.addData("Beacon Buttons", beacon.getAnalysis().getButtonString());
            telemetry.addData("Screen Rotation", rotation.getScreenOrientationActual());
            telemetry.addData("Frame Rate", fps.getFPSString() + " FPS");
            telemetry.addData("Frame Size", "Width: " + width + " Height: " + height);
            telemetry.addData("Frame Counter", frameCount);
            if (beacon.getAnalysis().getColorString().startsWith("blue")) {
                blueCount += 1;
            }

            waitOneFullHardwareCycle();
        }
        if (blueCount >= 5) {
            return true;
        }else{
            return false;
        }
    }
    public void visionAct (boolean blueLeft){
        if (blueLeft == true){
            robot.leftBeacon.setPosition(BEACONLEFT_PRESS);
            encoderDrive(1.0,5,5,100.0);
        }
        else if (blueLeft == false){
            robot.rightBeacon.setPosition(BEACONRIGHT_PRESS);
            encoderDrive(1.0, 5, 5, 100.0);
        }
    }
}
