package org.firstinspires.ftc.teamcode.testAndCopies.halcyonCopies;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Arrays;

@Deprecated
@Disabled
@TeleOp
public class PlaceholderBot extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;


    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
    }

    @Override
    public void loop() {
        setPower(gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        teleBoi();
    }

    private void setPower(double y, double x, double rot) {
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

    private void teleBoi(){
        telemetry.addData("leftStick x", gamepad1.left_stick_x);
        telemetry.addData("leftStick y", gamepad1.left_stick_y);
        telemetry.addData("rightStick x", gamepad1.right_stick_x);
        telemetry.addData("FL", frontLeft.getPower());
        telemetry.addData("FR", frontRight.getPower());
        telemetry.addData("BR", backRight.getPower());
        telemetry.addData("BL", backLeft.getPower());

    }
}