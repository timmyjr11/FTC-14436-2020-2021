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

/*
    Version two of Halcyon Teleop
 */
@TeleOp
public class Halcyon_version_two extends LinearOpMode {
    // Declares the drive to be able to use the motors
    SampleMecanumDrive d;

    // Declares the arrays, booleans for button toggles, boolean incrementer,
    // and a variable for speed
    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer;
    boolean a2Pressed;
    boolean x2Pressed;
    boolean y2Pressed;
    boolean rightDpad2Pressed;
    double speed = 0;

    // Declares the FTC dashboard
    private final FtcDashboard dash = FtcDashboard.getInstance();

    // Initializes the Finite State Machines
    gripperArm grip = gripperArm.gripOpen;
    gripperArm arm = gripperArm.armUp;
    shooterPower shooterPow = shooterPower.zeroPower;
    intakePos intakePosition = intakePos.down;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes the SampleMecanumDrive
        d = new SampleMecanumDrive(hardwareMap);

        // Initializes the telemetry for the FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        // Sets the motors to the correct direction and mode
        d.frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        d.backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        d.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        d.frontShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.backShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        // Puts robot in starting position
        d.tapper.setPosition(0);
        d.stopper.setPosition(1);
        d.leftPivot.setPosition(0);
        d.rightPivot.setPosition(1);
        d.Gripper.setPosition(0.55);
        d.armPivot.setPosition(1);

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            // Function that makes the robot move
            power();

            // If x is pressed, toggle the gripper open and closed
            if (x2Pressed) {
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

            // If a is pressed, toggle the arm up and down
            if (a2Pressed) {
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

            // If y is pressed, toggle and motors on and off
            if (y2Pressed) {
                if (shooterPow == shooterPower.fullPower || shooterPow == shooterPower.halfPower) {
                    shooterPow = shooterPower.zeroPower;
                } else if (shooterPow == shooterPower.zeroPower){
                    shooterPow = shooterPower.fullPower;
                }
            }

            // If dpad-up is pressed, move shooter up and set power to full
            if (gamepad2.dpad_up) {
                d.rightPivot.setPosition(0);
                d.leftPivot.setPosition(1);
                shooterPow = shooterPower.fullPower;

            }

            // If dpad-down is pressed, move the shooter up and set power to half
            if (gamepad2.dpad_down) {
                d.rightPivot.setPosition(0);
                d.leftPivot.setPosition(1);
                shooterPow = shooterPower.halfPower;

            }

            // If left trigger is pulled, Move the stopper up
            // Otherwise leave down
            if (gamepad2.left_trigger > 0.5) {
                d.stopper.setPosition(0.35);
            } else {
                d.stopper.setPosition(1);
            }

            // If right trigger is pulled and the stopper is up,
            // Then shoot
            if (gamepad2.right_trigger > 0.5 && d.stopper.getPosition() == 0.35) {
                d.tapper.setPosition(1);
            } else {
                d.tapper.setPosition(0);
            }

            // If left bumper is pressed, then outtake
            // Else if right bumper is pressed, then intake
            if (gamepad2.left_bumper) {
                d.convey.setPower(-1);
            } else if (gamepad2.right_bumper) {
                d.convey.setPower(1);
            } else {
                d.convey.setPower(0);
            }

            // If right on dpad is pressed, then toggle the shooter up and down
            if (rightDpad2Pressed) {
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

            // Switch case that changes the speed variable based on the
            // Current state
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

    // Enum to track the intake position
    private enum intakePos {
        up,
        down
    }

    // Enum to track the power of the shooter
    private enum shooterPower {
        zeroPower,
        halfPower,
        fullPower
    }

    // Enum to track both the position of the arm and the gripper
    private enum gripperArm {
        armUp,
        armDown,
        gripClosed,
        gripOpen
    }

    // Checks if there is toggle input
    private void checkInput() {
        x2Pressed = ifPressed(gamepad2.x);
        a2Pressed = ifPressed(gamepad2.a);
        y2Pressed = ifPressed(gamepad2.y);
        rightDpad2Pressed = ifPressed(gamepad2.dpad_right);
        booleanIncrementer = 0;
    }

    // Function that checks if the button is pressed then toggles
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

    // Function that controls how much power
    // Is in the drive motors
    private void power() {
        double power = 1;

        if (gamepad1.right_bumper) {
            power = 0.5;
        } else if (gamepad1.left_bumper) {
            power = 0.25;
        }

        Pose2d poseEstimate = d.getPoseEstimate();

        // Read pose
        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * power,
                -gamepad1.left_stick_x * power
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        d.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
        // Update everything. Odometry. Etc.
        d.update();
    }

}
