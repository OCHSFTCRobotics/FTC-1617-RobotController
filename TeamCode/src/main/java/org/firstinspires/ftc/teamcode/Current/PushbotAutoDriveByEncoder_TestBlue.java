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
package org.firstinspires.ftc.teamcode.Current;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Legacy.HardwarePushbotTeam5308;
import org.lasarobotics.vision.android.Cameras;
import org.lasarobotics.vision.ftc.resq.Beacon;
import org.lasarobotics.vision.opmode.LinearVisionOpMode;
import org.lasarobotics.vision.opmode.extensions.CameraControlExtension;
import org.lasarobotics.vision.util.ScreenOrientation;
import org.opencv.core.Size;
import org.opencv.ml.DTrees;

@Autonomous(name="Pushbot: 9803 Blue", group="9803")
public class PushbotAutoDriveByEncoder_TestBlue extends LinearVisionOpMode {
    /* Declare OpMode members. */
    HardwarePushbotTeam9803 robot   = new HardwarePushbotTeam9803(  );   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    static final double     COUNTS_PER_MOTOR_REV    = 1440;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double     DRIVE_SPEED             = 0.85;
    static final double     TURN_SPEED              = 0.3;
    public static final double BEACON_LEFTPRESS     = 0.0;
    public static final double BEACON_ZEROED        = 0.5;
    public static final double BEACON_RIGHTPRESS    = 1.0;
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
        GyroSensor gyro = robot.frontGyro;
        ColorSensor colorLeft = robot.colorLeft;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.colorLeft.enableLed(true);
        robot.colorRight.enableLed(true);
        idle();
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Front",  "Starting front at %7d : %7d",
                robot.frontLeftMotor.getCurrentPosition(),
                robot.frontRightMotor.getCurrentPosition());
        telemetry.addData("Back",  "Starting back at %7d : %7d",
                robot.backLeftMotor.getCurrentPosition(),
                robot.backRightMotor.getCurrentPosition());


        telemetry.update();
        gyro.calibrate(); //initializes gyro, may take time
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        int x = 1;
        int colorCount = 0;

        basicDrive(DRIVE_SPEED, 10, 10, 15); //Simple forward/backwards
        sideDrive(DRIVE_SPEED, 10, 15); //Simple sideways, factored; defaults positives to right, use negative for left
        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10, 15); //Simple example of least abstract encoder drive.
        angleCorrection(gyro.getHeading(), 0);
    }

    //Todo:Remove deprecated encoderDrive from original tank design.
    /*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (robot.frontLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newFrontRightTarget = (robot.frontRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            newBackLeftTarget = (robot.backLeftMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH));
            newBackRightTarget = (robot.backRightMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH));
            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.backRightMotor.setPower(Math.abs(speed));
            robot.backLeftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.backLeftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d : %7d",
                                            robot.frontLeftMotor.getCurrentPosition(),
                                            robot.frontRightMotor.getCurrentPosition());
                telemetry.addData("Path3",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Path4",  "Running at %7d : %7d",
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition());
                telemetry.addData("Clear Left", robot.colorLeft.alpha());
                telemetry.addData("Clear Right", robot.colorRight.alpha());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
        else {
            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }
    */
    public void basicDrive (double speed,
                            double leftInches, double rightInches,
                            double timeoutS){
        encoderDrive(speed, leftInches, rightInches, leftInches, rightInches, timeoutS);
    }
    public void sideDrive (double speed,
                            double factor,
                            double timeoutS){
        encoderDrive(speed, factor, -factor, -factor, factor, timeoutS);
    }
    public void angledDrive (double speed,
                             double leftInches, double rightInches,
                             double timeoutS, int angle){

    }
    public void angleCorrection (int curAngle, int reqAngle){
        int curAngleTemp = curAngle;
        int reqAngleTemp = reqAngle;

        if (curAngleTemp > 180){
            curAngleTemp = curAngleTemp - 360;
        }
        if (reqAngleTemp > 180){
            reqAngleTemp = reqAngleTemp - 360;
        }
        if (curAngleTemp > reqAngleTemp + 3){
            basicDrive(DRIVE_SPEED, -.5, .5, 10);
        }
        else if (curAngleTemp < reqAngleTemp - 3){
            basicDrive(DRIVE_SPEED, .5, -.5, 10);
        }
        else{
            telemetry.addLine("Info: No significant error detected. Either on track or messed up.");
            telemetry.update();
        }



    }
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches,
                             double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = (robot.frontLeftMotor.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH));
            newFrontRightTarget = (robot.frontRightMotor.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH));
            robot.frontLeftMotor.setTargetPosition(newFrontLeftTarget);
            robot.frontRightMotor.setTargetPosition(newFrontRightTarget);

            newBackLeftTarget = (robot.backLeftMotor.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH));
            newBackRightTarget = (robot.backRightMotor.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH));
            robot.backLeftMotor.setTargetPosition(newBackLeftTarget);
            robot.backRightMotor.setTargetPosition(newBackRightTarget);


            // Turn On RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontRightMotor.setPower(Math.abs(speed));
            robot.frontLeftMotor.setPower(Math.abs(speed));
            robot.backRightMotor.setPower(Math.abs(speed));
            robot.backLeftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.backLeftMotor.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d : %7d",
                        robot.frontLeftMotor.getCurrentPosition(),
                        robot.frontRightMotor.getCurrentPosition());
                telemetry.addData("Path3",  "Running to %7d :%7d", newBackLeftTarget,  newBackRightTarget);
                telemetry.addData("Path4",  "Running at %7d : %7d",
                        robot.backLeftMotor.getCurrentPosition(),
                        robot.backRightMotor.getCurrentPosition());
                telemetry.addData("Clear Left", robot.colorLeft.alpha());
                telemetry.addData("Clear Right", robot.colorRight.alpha());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
        else {
            // Stop all motion;
            robot.frontLeftMotor.setPower(0);
            robot.frontRightMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            robot.backRightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
            if (beacon.getAnalysis().getColorString().startsWith("blue")) { //blue:red
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
    //Todo: Come back and test for different movements.
    public void visionAct (boolean blueLeft){
        if (blueLeft == true){
            robot.leftBeacon.setPosition(BEACON_RIGHTPRESS);
            basicDrive(1.0, 5, 5, 10);
            //encoderDrive(1.0,5,5,100.0);
        }
        else if (blueLeft == false){
            robot.rightBeacon.setPosition(BEACON_RIGHTPRESS);
            basicDrive(1.0, -5, -5, 10);
            //encoderDrive(1.0, 5, 5, 100.0);
        }
    }
    public static int correctNum(int num){
        if (num <= 360 && num >= 355){

        }
        return num;
    }
}

