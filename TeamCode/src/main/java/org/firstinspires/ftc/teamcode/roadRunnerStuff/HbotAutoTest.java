package org.firstinspires.ftc.teamcode.roadRunnerStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
@Disabled
public class HbotAutoTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    double frontVelocity;
    double backVelocity;
    int shooterPower;

    public static double x = 0;

    public static double y = -20;

    public static double RingTwo = 6.7;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

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
        drive.backShooter.setVelocityPIDFCoefficients(7, 0, 4, 12.5);

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
                    .strafeRight(9)

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


            if (gamepad1.a) {
                drive.followTrajectory(moveToShootPower);
                drive.followTrajectory(shootPowerTwo);
                drive.followTrajectory(shootPowerThree);
            }
        }
    }
