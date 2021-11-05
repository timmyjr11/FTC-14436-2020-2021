package org.firstinspires.ftc.teamcode.testAndCopies;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@TeleOp
@Disabled
public class HbotDec18 extends LinearOpMode {
    //Creates motors and sets to null (0).
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() {
        //Gets the motors from the rev hub configuration.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {
            //Uses the setPower method to determine the speed of the motors using the left and right sticks
            setPower(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            Teleboi();
        }
    }

    private void Teleboi() {
        //Displays data to the driver station controller, to show the stick input and motor speeds
        telemetry.addData("leftStick x", gamepad1.left_stick_x);
        telemetry.addData("leftStick y", gamepad1.left_stick_y);
        telemetry.addData("rightStick x", gamepad1.right_stick_x);
        telemetry.addData("FL", frontLeft.getPower());
        telemetry.addData("FR", frontRight.getPower());
        telemetry.addData("BR", backRight.getPower());
        telemetry.addData("BL", backLeft.getPower());
    }

    private void setPower(float y, float x, float rot) {
        double frontLeftPower = y + x + rot;
        double backLeftPower = y - x + rot;
        double frontRightPower = y - x - rot;
        double backLRightPower = y + x - rot;

        //Puts all the power for the motors in one name, motorPowers
        double[] motorPowers = {Math.abs(frontLeftPower), Math.abs(backLeftPower), Math.abs(frontRightPower), Math.abs(backLRightPower)};

        //Creates an array to sort the motorPowers
        Arrays.sort(motorPowers);

        //Sorts motors
        if (motorPowers[3] != 0) {
            frontLeftPower /= motorPowers[3];
            frontRightPower /= motorPowers[3];
            backLeftPower /= motorPowers[3];
            backLRightPower /= motorPowers[3];
        }

        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(-frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(-backLRightPower);
    }
}