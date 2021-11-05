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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
@Disabled
public class HelcyonShoot extends LinearOpMode {

    ElapsedTime timer = new ElapsedTime();


    double frontVelocity;
    double backVelocity;
    int shooterPower;

    double ringTwo = 8;
    double ringThree = 8;
    double x = -20;
    double y = -3.5;

    SampleMecanumDrive drive;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive.frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.frontShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Initialize SampleMecanumDrive
        // We want to turn off velocity control for tele-op
        // Velocity control per wheel is not necessary outside of motion profiled ;

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        drive.setPoseEstimate(PoseStorage.autoPose);

        waitForStart();

        drive.tapper.setPosition(0);
        drive.stopper.setPosition(1);
        drive.leftPivot.setPosition(0);
        drive.rightPivot.setPosition(1);
        drive.Gripper.setPosition(0.45);
        drive.armPivot.setPosition(1);


        if (isStopRequested()) return;

        drive.frontShooter.setVelocityPIDFCoefficients(20, 0, 10, 13.4);
        drive.backShooter.setVelocityPIDFCoefficients(7, 0, 4, 12);

        shooterPower = 0;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            driving();
            Action();
            teleBoi();

        }
    }

    private void driving(){
        if (gamepad1.right_bumper){
            power05();
        } else if (gamepad1.left_bumper){
            power025();
        } else {
            power();
        }
    }

    private void power() {

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Create a vector from the gamepad x/y inputs
        // Then, rotate that vector by the inverse of that heading
        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ).rotated(-poseEstimate.getHeading());

        // Pass in the rotated input + right stick value for rotation
        // Rotation is not part of the rotated input thus must be passed in separately
        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x
                )
        );
        // Update everything. Odometry. Etc.
        drive.update();
    }

    private void power05() {
        Pose2d poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * 0.5,
                -gamepad1.left_stick_x * 0.5
        ).rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x * 0.5
                )
        );

        drive.update();
    }

    private void power025() {

        Pose2d poseEstimate = drive.getPoseEstimate();

        Vector2d input = new Vector2d(
                -gamepad1.left_stick_y * 0.25,
                -gamepad1.left_stick_x * 0.25
        ).rotated(-poseEstimate.getHeading());

        drive.setWeightedDrivePower(
                new Pose2d(
                        input.getX(),
                        input.getY(),
                        -gamepad1.right_stick_x * 0.25
                )
        );

        drive.update();
    }



    private void Action(){

        if(gamepad1.a){
            PowerShotAutoRR();
        }

        if (gamepad2.dpad_down){
            drive.leftPivot.setPosition(1);
            drive.rightPivot.setPosition(0);
            frontVelocity = 1300;
            backVelocity = 1950;
            shooterPower = 1;
            speed();
            driving();
            sleep(200);
        }

        if (gamepad2.dpad_up){
            drive.leftPivot.setPosition(1);
            drive.rightPivot.setPosition(0);
            frontVelocity = 1140;
            backVelocity = 1710;
            shooterPower = 1;
            speed();
            driving();
            sleep(200);
        }

        if (gamepad2.dpad_right){
            if (drive.leftPivot.getPosition() == 1 && drive.rightPivot.getPosition() == 0) {
                drive.leftPivot.setPosition(0);
                drive.rightPivot.setPosition(1);
            } else if (drive.leftPivot.getPosition() == 0 && drive.rightPivot.getPosition() == 1){
                drive.leftPivot.setPosition(1);
                drive.rightPivot.setPosition(0);
            }
            driving();
            sleep(200);
        }

        if (gamepad2.y){
            if (shooterPower == 1) {
                drive.frontShooter.setVelocity(0);
                drive.backShooter.setVelocity(0);
                shooterPower = 0;
            } else if(shooterPower == 0){
                drive.frontShooter.setVelocity(frontVelocity);
                drive.backShooter.setVelocity(backVelocity);
                shooterPower = 1;
            }
            driving();
            sleep(200);
        }

        if (gamepad2.right_trigger > 0.5 && shooterPower == 1) {
            shootingThreeRings();
            driving();
        }

        if (gamepad2.left_trigger > 0.5 && shooterPower == 1) {
            ShootingOneRing();
            driving();
        }


        if (gamepad2.right_bumper) {
            drive.convey.setPower(1);
            drive.intake.setPower(1);
            driving();
        }

        if (gamepad2.left_bumper) {
            drive.convey.setPower(-1);
            drive.intake.setPower(-1);
            driving();
        }

        if (!gamepad2.left_bumper && !gamepad2.right_bumper) {
            drive.convey.setPower(0);
            drive.intake.setPower(0);
            driving();
        }

        if (gamepad2.b){
            if (drive.Gripper.getPosition() == 0){
                drive.Gripper.setPosition(0.45);
            } else if (drive.Gripper.getPosition() == 0.45){
                drive.Gripper.setPosition(0);
            }
            driving();
            sleep(200);
        }

        if (gamepad2.a){
            if (drive.armPivot.getPosition() == 0){
                drive.armPivot.setPosition(1);
            } else if (drive.armPivot.getPosition() == 1) {
                drive.armPivot.setPosition(0);
            }
            driving();
            sleep(200);
        }

        drive.stopper.setPosition(1);
        driving();
    }

    private void shootingThreeRings(){
        timer.reset();
        speed();
        drive.stopper.setPosition(0);
        while (timer.milliseconds() < 500){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(1);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(0);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(1);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(0);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(1);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(0);
        drive.stopper.setPosition(1);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }
        frontVelocity = 0;
        backVelocity = 0;
        shooterPower = 0;
        drive.frontShooter.setVelocity(frontVelocity);
        drive.backShooter.setVelocity(backVelocity);
        driving();
    }

    private void ShootingOneRing(){
        timer.reset();
        speed();
        drive.stopper.setPosition(0);
        while (timer.milliseconds() < 500){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(1);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(0);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }

        drive.tapper.setPosition(0);
        drive.stopper.setPosition(1);
        speed();
        timer.reset();
        while (timer.milliseconds() < 100){
            teleBoi();
            driving();
        }
        frontVelocity = 0;
        backVelocity = 0;
        shooterPower = 0;
        drive.tapper.setPosition(0);
        drive.frontShooter.setVelocity(frontVelocity);
        drive.backShooter.setVelocity(backVelocity);
        driving();
    }


    private void speed() {

        drive.frontShooter.setVelocity(frontVelocity);
        drive.backShooter.setVelocity(backVelocity);
        driving();
    }

    private void teleBoi(){

        Pose2d poseEstimate = drive.getPoseEstimate();

        telemetry.addData("VelocityF", frontVelocity);
        telemetry.addData("VelocityB", backVelocity);
        telemetry.addData("FrontShooter Speed", drive.frontShooter.getVelocity());
        telemetry.addData("backShooter Speed", drive.backShooter.getVelocity());
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();
        drive.update();
    }

    private void PowerShotAutoRR(){

        drive.frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.intake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontVelocity = 1140;
        backVelocity = 1938;


        drive.tapper.setPosition(0);
        drive.stopper.setPosition(1);
        drive.leftPivot.setPosition(1);
        drive.rightPivot.setPosition(0);
        drive.armPivot.setPosition(1);
        drive.Gripper.setPosition(0);

        drive.frontShooter.setVelocityPIDFCoefficients(20, 0, 10, 13.4);
        drive.backShooter.setVelocityPIDFCoefficients(7, 0, 4, 12.5);

        waitForStart();

        if (isStopRequested()) return;

     Pose2d start =  drive.getPoseEstimate();

     drive.setPoseEstimate(start);


        Trajectory moveToShootPower = drive.trajectoryBuilder(start)
                .splineToConstantHeading(new Vector2d(x, y), Math.toRadians(0))

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
                .strafeRight(ringTwo)

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
                .strafeRight(ringThree)

                .addTemporalMarker(0.1, () -> {
                    drive.tapper.setPosition(0);
                    drive.frontShooter.setVelocity(1150);
                    drive.backShooter.setVelocity(1720);

                })

                .addDisplacementMarker(() -> {
                    drive.tapper.setPosition(1);
                    drive.frontShooter.setVelocity(1150);
                    drive.backShooter.setVelocity(1720);
                })


                .build();

        drive.followTrajectory(moveToShootPower);
        drive.followTrajectory(shootPowerTwo);
        drive.followTrajectory(shootPowerThree);

        drive.setPoseEstimate(PoseStorage.telePower);

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
