package org.firstinspires.ftc.teamcode.roadRunnerStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

@TeleOp
public class HalcyonNoDriving extends LinearOpMode {
    SampleMecanumDrive d;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer;
    boolean a1Pressed;
    boolean x1Pressed;
    boolean y1Pressed;
    boolean rightDpad1Pressed;
    double speed = 0;

    private final FtcDashboard dash = FtcDashboard.getInstance();

    gripperArm grip = gripperArm.gripOpen;
    gripperArm arm = gripperArm.armUp;
    shooterPower shooterPow = shooterPower.zeroPower;
    intakePos intakePosition = intakePos.down;

    @Override
    public void runOpMode() throws InterruptedException {
        d = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        d.frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        d.backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        d.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        d.frontShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.backShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        d.tapper.setPosition(0);
        d.stopper.setPosition(1);
        d.leftPivot.setPosition(0);
        d.rightPivot.setPosition(1);
        d.Gripper.setPosition(0.55);
        d.armPivot.setPosition(1);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            if (x1Pressed) {
                switch (grip) {
                    case gripClosed:
                        d.Gripper.setPosition(0.55);
                        grip = gripperArm.gripOpen;
                        break;
                    case gripOpen:
                        d.Gripper.setPosition(0);
                        grip = gripperArm.gripClosed;
                        break;
                }
            }

            if (a1Pressed) {
                switch (arm) {
                    case armUp:
                        d.armPivot.setPosition(0);
                        arm = gripperArm.armDown;
                        break;
                    case armDown:
                        d.armPivot.setPosition(1);
                        arm = gripperArm.armUp;
                        break;
                }
            }

            if (y1Pressed) {
                if (shooterPow == shooterPower.fullPower || shooterPow == shooterPower.halfPower) {
                    shooterPow = shooterPower.zeroPower;
                } else if (shooterPow == shooterPower.zeroPower){
                    shooterPow = shooterPower.fullPower;
                }
            }

            if (gamepad1.dpad_up) {
                d.rightPivot.setPosition(0);
                d.leftPivot.setPosition(1);
                shooterPow = shooterPower.fullPower;

            }

            if (gamepad1.dpad_down) {
                d.rightPivot.setPosition(0);
                d.leftPivot.setPosition(1);
                shooterPow = shooterPower.halfPower;

            }

            if (gamepad1.left_trigger > 0.5) {
                d.stopper.setPosition(0.35);
            } else {
                d.stopper.setPosition(1);
            }

            if (gamepad1.right_trigger > 0.5 && d.stopper.getPosition() == 0.35) {
                d.tapper.setPosition(1);
            } else {
                d.tapper.setPosition(0);
            }

            if (gamepad1.b && gamepad1.right_trigger > 0.5) {
                d.convey.setPower(-1);
            } else if (gamepad1.b) {
                d.convey.setPower(1);
            } else {
                d.convey.setPower(0);
            }

            if (rightDpad1Pressed) {
                switch (intakePosition) {
                    case up:
                        d.rightPivot.setPosition(0);
                        d.leftPivot.setPosition(1);
                        intakePosition = intakePos.down;
                        break;
                    case down:
                        d.rightPivot.setPosition(1);
                        d.leftPivot.setPosition(0);
                        intakePosition = intakePos.up;
                        break;
                }
            }
            // public static setShooterPower(shooterPow)
            // switch(shooterPow)
            // setShooterPow(halfPower)
            switch (shooterPow) {
                case zeroPower:
                    speed = 0;
                    d.frontShooter.setPower(speed);
                    d.backShooter.setPower(speed);
                    break;
                case fullPower:
                    speed = 1;
                    d.frontShooter.setPower(speed);
                    d.backShooter.setPower(speed);
                    break;
                case halfPower:
                    speed = 0.75;
                    d.frontShooter.setPower(speed);
                    d.backShooter.setPower(speed);
                    break;
            }



            checkInput();
        }
    }

    private enum intakePos {
        up,
        down
    }

    private enum shooterPower {
        zeroPower,
        halfPower,
        fullPower
    }

    private enum gripperArm {
        armUp,
        armDown,
        gripClosed,
        gripOpen
    }

    private void checkInput() {
        x1Pressed = ifPressed(gamepad1.x);
        a1Pressed = ifPressed(gamepad1.a);
        y1Pressed = ifPressed(gamepad1.y);
        rightDpad1Pressed = ifPressed(gamepad1.dpad_right);
        booleanIncrementer = 0;
    }

    private boolean ifPressed(boolean button) {
        boolean output = false;
        if (booleanArray.size() == booleanIncrementer) {
            booleanArray.add(false);
        }

        boolean buttonWas = booleanArray.get(booleanIncrementer);

        //noinspection PointlessBooleanExpression
        if (button != buttonWas && button == true) {
            output = true;
        }
        booleanArray.set(booleanIncrementer, button);

        booleanIncrementer = booleanIncrementer + 1;
        return output;
    }
}
