package org.firstinspires.ftc.teamcode.preRoadRunner;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

@Disabled
@TeleOp
public class HbotYeetTest extends LinearOpMode {
    //Creates motors and sets to null (0).
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotorEx backShooter;
    DcMotorEx frontShooter;
    DcMotor convey;
    DcMotor intake;
    Servo stopper;
    Servo tapper;
    Servo leftPivot;
    Servo rightPivot;

    int shooterPower;


    final ElapsedTime timer = new ElapsedTime();
    final double totalTime = 120;
    ElapsedTime stopperTime = new ElapsedTime();
    ElapsedTime shooterTime = new ElapsedTime();
    ElapsedTime tapperTime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Gets the motors from the rev hub configuration.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        stopper = hardwareMap.get(Servo.class, "stopper");
        tapper = hardwareMap.get(Servo.class, "tapper");
        convey = hardwareMap.get(DcMotor.class, "convey");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");

        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        convey.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);


        shooterPower = 0;

        waitForStart();

        timer.reset();

        tapper.setPosition(0);
        stopper.setPosition(1);
        leftPivot.setPosition(1);//change later
        rightPivot.setPosition(0);

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()) {


            //Uses the setPower method to determine the speed of the motors using the left and right sticks
            setPower(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

            if (gamepad2.y && shooterPower == 0) {
                backShooter.setVelocity(2175);//change later 2196
                frontShooter.setVelocity(2175);
                shooterPower = 1;
                sleep(200);
            } else if (gamepad2.y && shooterPower == 1) {
                backShooter.setPower(0);
                frontShooter.setPower(0);
                shooterPower = 0;
                sleep(200);
            }

            if (gamepad2.right_trigger > 0.5 && shooterPower == 1) {
                stopperTime.reset();
                while (stopperTime.milliseconds() < 1300) {
                    stopper.setPosition(0);
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                    backShooter.setVelocity(2175);
                    frontShooter.setVelocity(2175);
                }

                //change
                for (int tapMove = 0; tapMove < 6; tapMove++) {
                    telemetry.addData("TapMove", tapMove);
                    telemetry.addData("FrontShooter", frontShooter.getVelocity());
                    telemetry.addData("BackShooter", backShooter.getVelocity());
                    telemetry.update();
                    if (tapper.getPosition() == 0) {
                        tapper.setPosition(0.5);
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                        backShooter.setVelocity(2175);
                        frontShooter.setVelocity(2175);

                    } else if (tapper.getPosition() == 0.5) {
                        tapper.setPosition(0);
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                        backShooter.setVelocity(2175);
                        frontShooter.setVelocity(2175);
                    }

                    if (tapMove == 3) {
                        tapperTime.reset();
                        while (tapperTime.milliseconds() < 500) {
                            backShooter.setVelocity(2175);
                            frontShooter.setVelocity(2175);
                        }
                    } else {
                        sleep(200);
                    }
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
                    backShooter.setPower(1);
                    frontShooter.setPower(1);
                }
                for (int count = 0; count < 2; count++) {
                    if (tapper.getPosition() == 0) {
                        tapper.setPosition(1);
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                    } else if (tapper.getPosition() == 1) {
                        tapper.setPosition(0);
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                    }
                    sleep(100);
                }
                shooterPower = 0;
                backShooter.setPower(0);
                frontShooter.setPower(0);
            }

            if (gamepad2.dpad_up) {
                if (leftPivot.getPosition() == 1 && rightPivot.getPosition() == 0) {
                    leftPivot.setPosition(0);
                    rightPivot.setPosition(1);
                } else if (leftPivot.getPosition() == 0 && rightPivot.getPosition() == 1) {
                    leftPivot.setPosition(1);
                    rightPivot.setPosition(0);
                }
                sleep(200);
            }

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

            stopper.setPosition(1);
            Teleboi();

            while (gamepad1.right_bumper) {
                setPower05(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

                if (gamepad2.y && shooterPower == 0) {
                    backShooter.setPower(1);
                    frontShooter.setPower(1);
                    shooterPower = 1;
                    sleep(200);
                } else if (gamepad2.y && shooterPower == 1) {
                    backShooter.setPower(0);
                    frontShooter.setPower(0);
                    shooterPower = 0;
                    sleep(200);
                }
                if (gamepad2.right_trigger > 0.5 && shooterPower == 1) {
                    stopperTime.reset();
                    while (stopperTime.milliseconds() < 1300) {
                        stopper.setPosition(0);
                        frontLeft.setPower(0);
                        frontRight.setPower(0);
                        backLeft.setPower(0);
                        backRight.setPower(0);
                        backShooter.setPower(1);
                        frontShooter.setPower(1);
                    }
                    for (int tapMove = 0; tapMove < 6; tapMove++) {
                        if (tapper.getPosition() == 0) {
                            tapper.setPosition(1);
                            frontLeft.setPower(0);
                            frontRight.setPower(0);
                            backLeft.setPower(0);
                            backRight.setPower(0);
                        } else if (tapper.getPosition() == 1) {
                            tapper.setPosition(0);
                            frontLeft.setPower(0);
                            frontRight.setPower(0);
                            backLeft.setPower(0);
                            backRight.setPower(0);
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
                        backShooter.setPower(1);
                        frontShooter.setPower(1);
                    }
                    shooterTime.reset();
                    for (int count = 0; count < 2; count++) {
                        if (tapper.getPosition() == 0) {
                            tapper.setPosition(1);
                            frontLeft.setPower(0);
                            frontRight.setPower(0);
                            backLeft.setPower(0);
                            backRight.setPower(0);
                        } else if (tapper.getPosition() == 1) {
                            tapper.setPosition(0);
                            frontLeft.setPower(0);
                            frontRight.setPower(0);
                            backLeft.setPower(0);
                            backRight.setPower(0);
                        }
                        sleep(150);

                        shooterPower = 0;
                        backShooter.setPower(0);
                        frontShooter.setPower(0);
                    }
                }

                if (gamepad2.dpad_up) {
                    if (leftPivot.getPosition() == 1 && rightPivot.getPosition() == 0) {
                        leftPivot.setPosition(0);
                        rightPivot.setPosition(1);
                    } else if (leftPivot.getPosition() == 0 && rightPivot.getPosition() == 1) {
                        leftPivot.setPosition(1);
                        rightPivot.setPosition(0);
                    }
                    sleep(200);
                }

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

                stopper.setPosition(1);
                Teleboi();
            }
        }
    }


    private void Teleboi (){
        //Displays data to the driver station controller, to show the stick input and motor speeds
        double timeLeft = totalTime - timer.seconds();

        telemetry.addData("Timer", timeLeft);
        telemetry.addData("FL", frontLeft.getPower());
        telemetry.addData("FR", frontRight.getPower());
        telemetry.addData("BR", backRight.getPower());
        telemetry.addData("BL", backLeft.getPower());
        telemetry.addData("intake", intake.getPower());
        telemetry.addData("convey", convey.getPower());
        telemetry.addData("FrontShooter", frontShooter.getVelocity());
        telemetry.addData("BackShooter", backShooter.getVelocity());
        telemetry.update();
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
        frontRight.setPower(-frontRightPower * 0.5);
        backLeft.setPower(backLeftPower * 0.5);
        backRight.setPower(-backLRightPower * 0.5);
    }
}
