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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

//import org.firstinspires.ftc.teamcode.Legacy.HardwarePushbotTeam5308;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbotTeam5308 class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop5308 for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Pushbot: 9803 Teleop", group="9803")
public class Teleop9803 extends OpMode{

    /* Declare OpMode members. */
    HardwarePushbotTeam9803 robot       = new HardwarePushbotTeam9803(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftDriveStick;
        double rightDriveStick;
        double leftTrig;
        double rightTrig;


        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        leftDriveStick = gamepad1.left_stick_y;
        rightDriveStick = gamepad1.right_stick_y;
        leftTrig = gamepad1.left_trigger;
        rightTrig= gamepad1.right_trigger;

        //DRIVE ORDER: Front Left, Front Right, Back Left, Back Right
        /*
        switch (SWITCHDRIVE){
            case TESTSTATE;
        }
        */
        if (rightTrig<=.1&&leftTrig<=.1){
            drive(robot, rightDriveStick, leftDriveStick, rightDriveStick, leftDriveStick);
        }


        if (gamepad1.right_trigger>0.1){
            drive(robot, rightTrig, -rightTrig, -rightTrig, rightTrig);

        }
        if (gamepad1.left_trigger>0.1){
            drive(robot, -leftTrig, leftTrig, leftTrig, -leftTrig);
        }

        if (gamepad2.right_bumper){
            robot.beacon.setPosition(0.0);
        }
        if (gamepad2.left_bumper){
            robot.beacon.setPosition(1.0);
        }
        if (gamepad2.start){
            robot.beacon.setPosition(.5);
        }

        /*

        if (gamepad1.y) {
            drive(robot, .99, 0, 0, .99);
        }
        if (gamepad1.x) {
            drive(robot, 0, .99, .99, 0);
        }
        if (gamepad1.a) {
            drive(robot, 0, -.99, -.99, 0);
        }
        if (gamepad1.b) {
            drive(robot, -.99, 0, 0, -.99);
        }

        */
        //WINGED_POS = robot.wing.getPosition();



        // Send telemetry message to signify robot running;
        telemetry.addData("left",  "%.2f", leftDriveStick);
        telemetry.addData("right", "%.2f", rightDriveStick);
        telemetry.addData("gyro", "%d", robot.frontGyro.getHeading());
        telemetry.update();



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
    public static void drive(HardwarePushbotTeam9803 robot,
                             double frontLeft, double frontRight,
                             double backLeft, double backRight){
        robot.backLeftMotor.setPower(backLeft);
        robot.backRightMotor.setPower(backRight);
        robot.frontLeftMotor.setPower(frontLeft);
        robot.frontRightMotor.setPower(frontRight);
    }
    public enum SWITCHDRIVE{
        TESTSTATE
    }

}
