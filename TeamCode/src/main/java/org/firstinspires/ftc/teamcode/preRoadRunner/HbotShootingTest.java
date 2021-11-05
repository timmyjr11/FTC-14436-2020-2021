package org.firstinspires.ftc.teamcode.preRoadRunner;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;
@Config
@TeleOp
@Disabled
public class HbotShootingTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //Creates motors and sets to null (0).
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotorEx backShooter;
    DcMotorEx frontShooter;
    DcMotor convey;
    DcMotor intake;
    Servo stopper;
    Servo tapper;
    Servo leftPivot;
    Servo rightPivot;
    Servo Gripper;
    Servo armPivot;

    int shooterPower;

    double Velocity;

    final ElapsedTime timer = new ElapsedTime();
    final double totalTime = 120;
    ElapsedTime stopperTime = new ElapsedTime();
    ElapsedTime shooterTime = new ElapsedTime();


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //Gets the motors from the rev hub configuration.
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        stopper = hardwareMap.get(Servo.class, "stopper");
        tapper = hardwareMap.get(Servo.class, "tapper");
        convey = hardwareMap.get(DcMotor.class, "convey");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        armPivot = hardwareMap.get(Servo.class, "armPivot");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontShooter.setVelocityPIDFCoefficients(15,0,5,14.2);
        backShooter.setVelocityPIDFCoefficients(15,0,5,14);


        shooterPower = 0;

        Velocity = 1385;


        waitForStart();

        timer.reset();

        tapper.setPosition(0);
        stopper.setPosition(1);
        leftPivot.setPosition(0);
        rightPivot.setPosition(1);
        Gripper.setPosition(0.5);
        armPivot.setPosition(1);

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("FrontVelo",frontShooter.getVelocity());
            telemetry.addData("BackVelo", backShooter.getVelocity());
            telemetry.addData("TargetVelo", Velocity);
            telemetry.update();

            setPower05(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

            autoShooting();


            if (gamepad2.right_bumper) {
                convey.setPower(1);
                intake.setPower(1);
            }

            if (gamepad2.left_bumper) {
                convey.setPower(-1);
                intake.setPower(-1);
            }

            if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
                convey.setPower(0);
                intake.setPower(0);
            }

            if (gamepad2.b){
                if (Gripper.getPosition() == 0){
                    Gripper.setPosition(0.5);
                } else if (Gripper.getPosition() == 0.5){
                    Gripper.setPosition(0);
                }
                sleep(200);
            }

            if (gamepad2.a){
                if (armPivot.getPosition() == 0){
                    armPivot.setPosition(1);
                } else if (armPivot.getPosition() == 1) {
                    armPivot.setPosition(0);
                }
                sleep(200);
            }

            stopper.setPosition(1);


        }

    }
    private void setPower05(double y, double x, double rot) {
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

        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction, halves the speed
        frontLeft.setPower(frontLeftPower * 0.5);
        frontRight.setPower(frontRightPower * 0.5);
        backLeft.setPower(backLeftPower * 0.5);
        backRight.setPower(backLRightPower * 0.5);
    }
    private void autoShooting(){

        if (gamepad2.y && shooterPower == 0) {
            backShooter.setVelocity(Velocity);//change later
            frontShooter.setVelocity(Velocity);
            shooterPower = 1;
            sleep(200);
        } else if (gamepad2.y && shooterPower == 1) {
            backShooter.setPower(0);
            frontShooter.setPower(0);
            shooterPower = 0;
            sleep(200);
        }


        if (gamepad2.dpad_down){
            rightPivot.setPosition(0);
            leftPivot.setPosition(1);
        }
        if (gamepad2.right_trigger > 0.5 && shooterPower == 1) {
            stopperTime.reset();
            while (stopperTime.milliseconds() < 1300) {
                stopper.setPosition(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                backShooter.setVelocity(Velocity);
                frontShooter.setVelocity(Velocity);
                telemetry.addData("FrontVelo",frontShooter.getVelocity());
                telemetry.addData("BackVelo", backShooter.getVelocity());
                telemetry.addData("TargetVelo", Velocity);
                telemetry.update();
            }
            for (int tapMove = 0; tapMove < 6; tapMove++) {
                if (tapper.getPosition() == 0) {
                    tapper.setPosition(1);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    backShooter.setVelocity(Velocity);
                    frontShooter.setVelocity(Velocity);
                    telemetry.addData("FrontVelo",frontShooter.getVelocity());
                    telemetry.addData("BackVelo", backShooter.getVelocity());
                    telemetry.addData("TargetVelo", Velocity);
                    telemetry.update();
                } else if (tapper.getPosition() == 1) {
                    tapper.setPosition(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    backShooter.setVelocity(Velocity);
                    frontShooter.setVelocity(Velocity);
                    telemetry.addData("FrontVelo",frontShooter.getVelocity());
                    telemetry.addData("BackVelo", backShooter.getVelocity());
                    telemetry.addData("TargetVelo", Velocity);
                    telemetry.update();
                }
                sleep(200);

                shooterPower = 0;
                backShooter.setPower(0);
                frontShooter.setPower(0);
            }
        }

        if (gamepad2.left_trigger > 0.5 && shooterPower == 1) {
            stopperTime.reset();
            while (stopperTime.milliseconds() < 400) {
                stopper.setPosition(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                backShooter.setVelocity(Velocity);
                frontShooter.setVelocity(Velocity);
                telemetry.addData("FrontVelo",frontShooter.getVelocity());
                telemetry.addData("BackVelo", backShooter.getVelocity());
                telemetry.addData("TargetVelo", Velocity);
                telemetry.update();
            }
            shooterTime.reset();
            for (int count = 0; count < 2; count++) {
                if (tapper.getPosition() == 0) {
                    tapper.setPosition(1);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    backShooter.setVelocity(Velocity);
                    frontShooter.setVelocity(Velocity);
                    telemetry.addData("FrontVelo", frontShooter.getVelocity());
                    telemetry.addData("BackVelo", backShooter.getVelocity());
                    telemetry.addData("TargetVelo", Velocity);
                    telemetry.update();
                } else if (tapper.getPosition() == 1) {
                    tapper.setPosition(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    backShooter.setVelocity(Velocity);
                    frontShooter.setVelocity(Velocity);
                    telemetry.addData("FrontVelo", frontShooter.getVelocity());
                    telemetry.addData("BackVelo", backShooter.getVelocity());
                    telemetry.addData("TargetVelo", Velocity);
                    telemetry.update();
                }
                sleep(150);

                shooterPower = 0;
                backShooter.setPower(0);
                frontShooter.setPower(0);
            }
        }
    }
}