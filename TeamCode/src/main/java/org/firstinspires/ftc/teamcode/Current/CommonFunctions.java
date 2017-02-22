package org.firstinspires.ftc.teamcode.Current;

/**
 * Created by jasona99 on 2/22/2017.
 */

public class CommonFunctions {
    public static void drive(HardwarePushbotTeam9803 robot,
                             double frontLeft, double frontRight,
                             double backLeft, double backRight){
        robot.backLeftMotor.setPower(backLeft);
        robot.backRightMotor.setPower(backRight);
        robot.frontLeftMotor.setPower(frontLeft);
        robot.frontRightMotor.setPower(frontRight);
    }
}
