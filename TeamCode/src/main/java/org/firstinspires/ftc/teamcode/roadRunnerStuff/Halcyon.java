package org.firstinspires.ftc.teamcode.roadRunnerStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
public class Halcyon extends LinearOpMode {
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


    ElapsedTime timer = new ElapsedTime();

    PIDFCoefficients PID = DriveConstants.MOTOR_VELO_PID;

    double frontVelocity;
    double backVelocity;
    int shooterPower;


    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontShooter.setVelocityPIDFCoefficients(20, 0, 10, 13.4);
        backShooter.setVelocityPIDFCoefficients(7, 0, 4, 12);

        frontLeft.setVelocityPIDFCoefficients(PID.p, PID.i, PID.d, PID.f);
        frontRight.setVelocityPIDFCoefficients(PID.p, PID.i, PID.d, PID.f);
        backRight.setVelocityPIDFCoefficients(PID.p, PID.i, PID.d, PID.f);
        backRight.setVelocityPIDFCoefficients(PID.p, PID.i, PID.d, PID.f);

        shooterPower = 0;

        waitForStart();

        tapper.setPosition(0);
        stopper.setPosition(1);
        leftPivot.setPosition(0);
        rightPivot.setPosition(1);
        Gripper.setPosition(0.45);
        armPivot.setPosition(1);

        while (opModeIsActive() && !isStopRequested()) {
            Driving();
            Action();
            teleBoi();
            zeroPowerBehavior();
        }
    }

    private void Action(){

        if(gamepad1.a){
            PowerShotAutoRR();
        }

        if (gamepad2.dpad_down){
            leftPivot.setPosition(1);
            rightPivot.setPosition(0);
            frontVelocity = 1300;
            backVelocity = 1950;
            shooterPower = 1;
            speed();
            Driving();
            sleep(200);
        }

        if (gamepad2.dpad_up){
            leftPivot.setPosition(1);
            rightPivot.setPosition(0);
            frontVelocity = 1140;
            backVelocity = 1710;
            shooterPower = 1;
            speed();
            Driving();
            sleep(200);
        }

        if (gamepad2.dpad_right){
            if (leftPivot.getPosition() == 1 && rightPivot.getPosition() == 0) {
                leftPivot.setPosition(0);
                rightPivot.setPosition(1);
                Driving();
            } else if (leftPivot.getPosition() == 0 && rightPivot.getPosition() == 1){
                leftPivot.setPosition(1);
                rightPivot.setPosition(0);
                Driving();
            }
            sleep(200);
        }

        if (gamepad2.y){
            if (shooterPower == 1) {
                frontShooter.setVelocity(0);
                backShooter.setVelocity(0);
                shooterPower = 0;
                Driving();
            } else if(shooterPower == 0){
                frontShooter.setVelocity(frontVelocity);
                backShooter.setVelocity(backVelocity);
                shooterPower = 1;
                Driving();
            }
            sleep(200);
        }

        if (gamepad2.right_trigger > 0.5) {
            shootingThreeRings();
            Driving();
        }

        if (gamepad2.left_trigger > 0.5) {
            ShootingOneRing();
            Driving();
        }


        if (gamepad2.right_bumper) {
            convey.setPower(1);
            intake.setPower(1);
            Driving();
        }

        if (gamepad2.left_bumper) {
            convey.setPower(-1);
            intake.setPower(-1);
            Driving();
        }

        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            convey.setPower(0);
            intake.setPower(0);
            Driving();
        }

        if (gamepad2.b){
            if (Gripper.getPosition() == 0){
                Gripper.setPosition(0.45);
                Driving();
            } else if (Gripper.getPosition() == 0.45){
                Gripper.setPosition(0);
                Driving();
            }
            sleep(200);
        }

        if (gamepad2.a){
            if (armPivot.getPosition() == 0){
                armPivot.setPosition(1);
                Driving();
            } else if (armPivot.getPosition() == 1) {
                armPivot.setPosition(0);
                Driving();
            }
            sleep(200);
        }

        stopper.setPosition(1);
        tapper.setPosition(0);
        Driving();
    }

    private void shootingThreeRings(){
        timer.reset();
        speed();
        stopper.setPosition(0);
        Driving();
        while (timer.milliseconds() < 750){
            teleBoi();
            Driving();
        }

        tapper.setPosition(1);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }

        tapper.setPosition(0);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }

        tapper.setPosition(1);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }

        tapper.setPosition(0);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }

        tapper.setPosition(1);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }

        tapper.setPosition(0);
        stopper.setPosition(1);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }
        frontVelocity = 0;
        backVelocity = 0;
        shooterPower = 0;
        frontShooter.setVelocity(frontVelocity);
        backShooter.setVelocity(backVelocity);
    }

    private void ShootingOneRing(){
        timer.reset();
        speed();
        stopper.setPosition(0);
        Driving();
        while (timer.milliseconds() < 750){
            teleBoi();
            Driving();
        }

        tapper.setPosition(1);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }

        tapper.setPosition(0);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }
        tapper.setPosition(0);
        stopper.setPosition(1);
        speed();
        timer.reset();
        Driving();
        while (timer.milliseconds() < 150){
            teleBoi();
            Driving();
        }
        frontVelocity = 0;
        backVelocity = 0;
        shooterPower = 0;
        tapper.setPosition(0);
        frontShooter.setVelocity(frontVelocity);
        backShooter.setVelocity(backVelocity);
    }


    private void speed() {
        frontShooter.setVelocity(frontVelocity);
        backShooter.setVelocity(backVelocity);
    }

    private void teleBoi(){
        telemetry.addData("VelocityF", frontVelocity);
        telemetry.addData("VelocityB", backVelocity);
        telemetry.addData("FrontShooter Speed", frontShooter.getVelocity());
        telemetry.addData("backShooter Speed", backShooter.getVelocity());
        telemetry.addData("Front Right Speed", frontRight.getVelocity());
        telemetry.addData("Front Left Speed", frontLeft.getVelocity());
        telemetry.addData("Back Right Speed", backRight.getVelocity());
        telemetry.addData("Back Left Speed", backLeft.getVelocity());
        telemetry.update();
    }

    private void Driving(){
        if (gamepad1.right_bumper) {
            setPower05(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else if (gamepad1.left_bumper) {
            setPower025(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }
    }

    private void setPower(double y, double x, double rot) {
        double frontLeftPower = y + x + rot;
        double backLeftPower = y - x + rot;
        double frontRightPower = y - x - rot;
        double backLRightPower = y + x - rot;


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backLRightPower);
    }

    private void setPower05(double y, double x, double rot) {
        double frontLeftPower = y + x + rot;
        double backLeftPower = y - x + rot;
        double frontRightPower = y - x - rot;
        double backLRightPower = y + x - rot;


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower * 0.5);
        frontRight.setPower(frontRightPower * 0.5);
        backLeft.setPower(backLeftPower * 0.5);
        backRight.setPower(backLRightPower * 0.5);
    }

    private void setPower025(double y, double x, double rot) {
        double frontLeftPower = y + x + rot;
        double backLeftPower = y - x + rot;
        double frontRightPower = y - x - rot;
        double backLRightPower = y + x - rot;


        //Sets the power of the motors to the motors, which allows variable speed and movement in every direction
        frontLeft.setPower(frontLeftPower * 0.25);
        frontRight.setPower(frontRightPower * 0.25);
        backLeft.setPower(backLeftPower * 0.25);
        backRight.setPower(backLRightPower * 0.25);
    }

    private void zeroPowerBehavior(){
        if (gamepad1.right_trigger > 0.5){
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } else {
            frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void PowerShotAutoRR(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.intake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontVelocity = 1140;
        backVelocity = 1710;


        drive.tapper.setPosition(0);
        drive.stopper.setPosition(1);
        drive.leftPivot.setPosition(1);
        drive.rightPivot.setPosition(0);
        drive.armPivot.setPosition(1);
        drive.Gripper.setPosition(0);

        drive.frontShooter.setVelocityPIDFCoefficients(20, 0, 10, 13.4);
         drive.backShooter.setVelocityPIDFCoefficients(7, 0, 4, 12);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d start = new Pose2d(-71, -35);

        drive.setPoseEstimate(start);


        Trajectory moveToShootPower = drive.trajectoryBuilder(start)
                .splineToConstantHeading(new Vector2d(-20, -6), Math.toRadians(0))

                .addTemporalMarker(0.01, () -> {
                    drive.frontShooter.setVelocity(frontVelocity);
                    drive.backShooter.setVelocity(backVelocity);
                    drive.stopper.setPosition(0);
                })


                .addTemporalMarker(3, () -> {
                    drive.tapper.setPosition(1);
                    drive.frontShooter.setVelocity(frontVelocity);
                    drive.backShooter.setVelocity(backVelocity);
                })

                .build();


        Trajectory shootPowerTwo = drive.trajectoryBuilder(moveToShootPower.end())
                .strafeRight(7)

                .addTemporalMarker(0.1, () -> {
                    drive.tapper.setPosition(0);
                    drive.frontShooter.setVelocity(frontVelocity);
                    drive.backShooter.setVelocity(backVelocity);
                })

                .addDisplacementMarker(() -> {
                    drive.tapper.setPosition(1);
                    drive.frontShooter.setVelocity(frontVelocity);
                    drive.backShooter.setVelocity(backVelocity);
                })

                .build();

        Trajectory shootPowerThree = drive.trajectoryBuilder(shootPowerTwo.end())
                .strafeRight(7)

                .addTemporalMarker(0.1, () -> {
                    drive.tapper.setPosition(0);
                    drive.frontShooter.setVelocity(frontVelocity);
                    drive.backShooter.setVelocity(backVelocity);

                })

                .addDisplacementMarker(() -> {
                    drive.tapper.setPosition(1);
                    drive.frontShooter.setVelocity(frontVelocity);
                    drive.backShooter.setVelocity(backVelocity);
                })


                .build();



            drive.followTrajectory(moveToShootPower);
            drive.followTrajectory(shootPowerTwo);
            drive.followTrajectory(shootPowerThree);

            drive.stopper.setPosition(1);
            drive.leftPivot.setPosition(1);
            drive.rightPivot.setPosition(0);
            drive.armPivot.setPosition(0);
            drive.Gripper.setPosition(0.45);
            drive.frontShooter.setVelocity(0);
            drive.backShooter.setVelocity(0);
            shooterPower = 0;
    }
}