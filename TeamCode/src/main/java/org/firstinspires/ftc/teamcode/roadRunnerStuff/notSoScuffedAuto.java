package org.firstinspires.ftc.teamcode.roadRunnerStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@Autonomous
@Config
public class notSoScuffedAuto extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //Creates the phone camera as an object
    OpenCvCamera phoneCam;

    //creates ring count as an object
    static double ringCount;

    public static double ringTwo = 8;
    public static double ringThree = 8;
    public static double x = -20;
    public static double y = -3.5;
    public static double secWobbleGrab = -61;
    public static double forwardWobbleZero = 5.755;
    public static double forwardWobbleOne = 7;
    public static double forwardWobbleFour = 5.755;


    //Creates the rectangles
    int rect1x = 179; //May need to be adjusted //80
    int rect1y = 107; // May need to be adjust


    double frontVelocity;
    double backVelocity;

    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());


        drive.frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.intake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontVelocity = 1140;
        backVelocity = 1840;

        ElapsedTime timer = new ElapsedTime();

        drive.tapper.setPosition(0);
        drive.stopper.setPosition(1);
        drive.leftPivot.setPosition(1);
        drive.rightPivot.setPosition(0);
        drive.armPivot.setPosition(1);
        drive.Gripper.setPosition(0);

        drive.frontShooter.setVelocityPIDFCoefficients(20, 0, 10, 13.4);
        drive.backShooter.setVelocityPIDFCoefficients(7, 0, 4, 12.5);

        //Allows the camera to be used
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);

        //Detector
        phoneCam.setPipeline(new notSoScuffedAuto.RingDetectingPipeline());

        //Allows the camera to turn on
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        FtcDashboard.getInstance().startCameraStream(phoneCam, 30);

        Pose2d start = new Pose2d(-71, -35);

        drive.setPoseEstimate(start);

        Trajectory moveToShootPower = drive.trajectoryBuilder(start)
                .splineToConstantHeading(new Vector2d(x, y), Math.toRadians(0))

                .addTemporalMarker(0.01, () -> {
                    drive.frontShooter.setVelocity(frontVelocity);
                    drive.backShooter.setVelocity(backVelocity);
                    drive.stopper.setPosition(0.35);
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
                    drive.backShooter.setVelocity(1840);

                })

                .addDisplacementMarker(() -> {
                    drive.tapper.setPosition(1);
                    drive.frontShooter.setVelocity(1150);
                    drive.backShooter.setVelocity(1840);
                })


                .build();


        Trajectory goalZeroRingsP1 = drive.trajectoryBuilder(shootPowerThree.end())
                .splineToConstantHeading(new Vector2d(-15, -62), Math.toRadians(0),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(110.8),
                                        new MecanumVelocityConstraint(30.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.50, () -> drive.tapper.setPosition(0))

                .addTemporalMarker(2.50, () -> drive.armPivot.setPosition(0.1))

                .addTemporalMarker(3.25, () -> drive.Gripper.setPosition(0.75))

                .addTemporalMarker(4, () -> drive.armPivot.setPosition(1))

                .build();

        //Work later on  Trajectory goalZeroRingP2

        Trajectory goalOneRingP1 = drive.trajectoryBuilder(shootPowerThree.end())
                .splineToConstantHeading(new Vector2d(10, -36), Math.toRadians(0))

                .addTemporalMarker(0.25, () -> drive.tapper.setPosition(0))


                .addTemporalMarker(1, () -> drive.armPivot.setPosition(0.1))

                .addTemporalMarker(1.5, () -> drive.Gripper.setPosition(0.75))

                .addTemporalMarker(2.5, () -> drive.armPivot.setPosition(1))

                .addTemporalMarker(3, () -> drive.Gripper.setPosition(0))

                .build();

        //Work later on Trajectory goalOneRingP2

        Trajectory goalFourRingP1 = drive.trajectoryBuilder(shootPowerThree.end())
                .splineToConstantHeading(new Vector2d(35, -60), Math.toRadians(0))

                .addTemporalMarker(0.25, () -> drive.tapper.setPosition(0))

                .addTemporalMarker(2.5, () -> drive.armPivot.setPosition(0.1))

                .addTemporalMarker(3, () -> drive.Gripper.setPosition(0.75))

                .addTemporalMarker(3.7, () -> drive.armPivot.setPosition(1))

                .addTemporalMarker(4, () -> drive.Gripper.setPosition(0))

                .build();

        Trajectory movingToRingOneRing = drive.trajectoryBuilder(goalOneRingP1.end())
                .lineToLinearHeading(new Pose2d(-10, -40, Math.toRadians(182)),

                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(40.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addDisplacementMarker(1, () -> {
                    drive.convey.setPower(1);
                    drive.intake.setPower(1);

                    drive.leftPivot.setPosition(0);
                    drive.rightPivot.setPosition(1);

                    drive.tapper.setPosition(0);
                    drive.stopper.setPosition(1);
                    drive.frontShooter.setVelocity(0);
                    drive.backShooter.setVelocity(0);
                })

                .build();

        Trajectory movingtoFourRings = drive.trajectoryBuilder(goalFourRingP1.end())
                .lineToLinearHeading(new Pose2d(-10, -40, Math.toRadians(182)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(50.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )


                .addDisplacementMarker(1, () -> {
                    drive.convey.setPower(1);
                    drive.intake.setPower(1);

                    drive.leftPivot.setPosition(0);
                    drive.rightPivot.setPosition(1);

                    drive.tapper.setPosition(0);
                    drive.stopper.setPosition(1);
                    drive.frontShooter.setVelocity(0);
                    drive.backShooter.setVelocity(0);
                })

                .build();


        Trajectory grabRing1 = drive.trajectoryBuilder(movingToRingOneRing.end())
                .forward(17,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(40.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(() -> {
                    timer.reset();
                    while (timer.milliseconds() < 250) {

                    }
                })
                .build();

        Trajectory grabRing2 = drive.trajectoryBuilder(grabRing1.end())
                .forward(3,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(40.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(() -> {
                    timer.reset();
                    while (timer.milliseconds() < 250) {

                    }
                })
                .build();

        Trajectory grabRing3 = drive.trajectoryBuilder(grabRing2.end())
                .forward(3,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(40.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(() -> {
                    timer.reset();
                    while (timer.milliseconds() < 250) {

                    }
                })
                .build();

        Trajectory moveToWobbleOne = drive.trajectoryBuilder(grabRing1.end())
                .lineToLinearHeading(new Pose2d(-35.5, secWobbleGrab, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(20.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.5, () -> {
                    drive.armPivot.setPosition(0);
                    drive.Gripper.setPosition(0.55);
                })
                .build();

        Trajectory movetoWobbleFour = drive.trajectoryBuilder(grabRing3.end())
                .lineToLinearHeading(new Pose2d(-35.5, -63.5, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(20.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.3, () -> {
                    drive.armPivot.setPosition(0);
                    drive.Gripper.setPosition(0.55);
                })
                .build();

        Trajectory movetoWobbleZero = drive.trajectoryBuilder(goalZeroRingsP1.end())
                .lineToLinearHeading(new Pose2d(-35.5, -57.5, Math.toRadians(180)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(30.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.1, () -> drive.armPivot.setPosition(1))

                .addTemporalMarker(0.5, () -> {
                    drive.armPivot.setPosition(0);
                    drive.Gripper.setPosition(0.55);
                })
                .build();

        Trajectory grabWobbleGoal = drive.trajectoryBuilder(movetoWobbleZero.end())
                .forward(forwardWobbleZero,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(10.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )
                .addTemporalMarker(0.6, () -> drive.Gripper.setPosition(0))

                .build();

        Trajectory grabWobbleGoal1ring = drive.trajectoryBuilder(moveToWobbleOne.end())
                .forward(forwardWobbleOne,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(10.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.6, () -> {
                    drive.Gripper.setPosition(0);
                    drive.intake.setPower(0);
                })


                .build();

        Trajectory grabWobbleGoal4ring = drive.trajectoryBuilder(movetoWobbleFour.end())
                .forward(forwardWobbleFour,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(10.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.6, () -> {
                    drive.Gripper.setPosition(0);
                    drive.intake.setPower(0);
                })

                .build();

        Trajectory placeWobbleGoal2Zero = drive.trajectoryBuilder(grabWobbleGoal.end())
                .lineToLinearHeading(new Pose2d(-17, -45, Math.toRadians(0) + 1e-6),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(110.8),
                                        new MecanumVelocityConstraint(30.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.01, () -> drive.armPivot.setPosition(0.5))

                .addTemporalMarker(2.75, () -> drive.armPivot.setPosition(0))

                .addTemporalMarker(5, () -> drive.Gripper.setPosition(0.55))

                .build();

        Trajectory placeWobbleGoal2One = drive.trajectoryBuilder(grabWobbleGoal1ring.end())
                .lineToLinearHeading(new Pose2d(3, -17, Math.toRadians(0) + 1e-6),
                        new MinVelocityConstraint(
                        Arrays.asList(
                                new AngularVelocityConstraint(166.2),
                                new MecanumVelocityConstraint(40.6125, DriveConstants.TRACK_WIDTH)
                        )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.01, () -> {
                    drive.armPivot.setPosition(0.5);
                    drive.convey.setPower(0);
                    drive.rightPivot.setPosition(0);
                    drive.leftPivot.setPosition(1);
                })

                .addTemporalMarker(3.5, () -> drive.armPivot.setPosition(0))

                .addTemporalMarker(5, () -> drive.Gripper.setPosition(0.55))

                .build();

        Trajectory MovetoShootFourRings = drive.trajectoryBuilder(grabWobbleGoal4ring.end())
                .lineToLinearHeading(new Pose2d(-16, -32, Math.toRadians(0) + 1e-6))

                .addDisplacementMarker(1, () -> {
                    drive.leftPivot.setPosition(1);
                    drive.rightPivot.setPosition(0);

                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
        })

                .addTemporalMarker(0.75, () -> drive.stopper.setPosition(0.35))

                .addTemporalMarker(0.5, () -> drive.armPivot.setPosition(1))

                .build();


        Trajectory MovetoShootOneRing = drive.trajectoryBuilder(placeWobbleGoal2One.end())
                .lineToLinearHeading(new Pose2d(-16, -32, Math.toRadians(0)),
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(30.6125, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addTemporalMarker(0.1, () -> {

                    drive.intake.setPower(0);

                    drive.leftPivot.setPosition(1);
                    drive.rightPivot.setPosition(0);

                    drive.stopper.setPosition(0.35);

                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })

                .addTemporalMarker(0.5, () -> drive.armPivot.setPosition(1))

                .addTemporalMarker(0.75, () -> drive.Gripper.setPosition(0))

                .build();

        Trajectory ShootIntoGoal2 = drive.trajectoryBuilder(MovetoShootOneRing.end())
                .strafeRight(0.5,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(0.2, () -> {
                    drive.tapper.setPosition(1);
                    drive.convey.setPower(0);
                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })

                .addDisplacementMarker(() -> {
                    drive.tapper.setPosition(0);
                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })
                .build();

        Trajectory ShootIntoGoal3 = drive.trajectoryBuilder(ShootIntoGoal2.end())
                .strafeRight(0.5,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(0.2, () -> {
                    drive.tapper.setPosition(1);
                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })

                .addDisplacementMarker(() -> {
                    drive.tapper.setPosition(0);
                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })
                .build();

        Trajectory ShootIntoGoal4 = drive.trajectoryBuilder(ShootIntoGoal3.end())
                .strafeRight(1,
                        new MinVelocityConstraint(
                                Arrays.asList(
                                        new AngularVelocityConstraint(221.6),
                                        new MecanumVelocityConstraint(10, DriveConstants.TRACK_WIDTH)
                                )), new ProfileAccelerationConstraint(DriveConstants.MAX_ACCEL)
                )

                .addDisplacementMarker(0.2, () -> {
                    drive.tapper.setPosition(1);
                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })

                .addDisplacementMarker(() -> {
                    drive.tapper.setPosition(0);
                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })
                .build();

        Trajectory placeWobbleGoal2Four = drive.trajectoryBuilder(ShootIntoGoal4.end())
                .lineToLinearHeading(new Pose2d(35, -50, Math.toRadians(0) + 1e-6))

                .addTemporalMarker(0.01, () -> {
                    drive.armPivot.setPosition(0.5);
                    drive.convey.setPower(0);
                    drive.rightPivot.setPosition(0);
                    drive.leftPivot.setPosition(1);
                })

                .addTemporalMarker(2, () -> drive.armPivot.setPosition(0))

                .addTemporalMarker(3, () -> drive.Gripper.setPosition(0.55))

                .build();


        Trajectory parkZeroRing = drive.trajectoryBuilder(placeWobbleGoal2Zero.end())
                .lineToConstantHeading(new Vector2d(0, -29))
                .addTemporalMarker(0.5, () -> {
                    drive.armPivot.setPosition(1);
                    drive.stopper.setPosition(0);
                })
                .build();

        Trajectory parkOneRing = drive.trajectoryBuilder(ShootIntoGoal2.end())
                .lineToConstantHeading(new Vector2d(0, -29))
                .addTemporalMarker(0.5, () -> {
                    drive.armPivot.setPosition(1);
                    drive.stopper.setPosition(0);
                })                .build();

        Trajectory parkFourRings = drive.trajectoryBuilder(placeWobbleGoal2Four.end())
                .lineToConstantHeading(new Vector2d(0, -29))
                .addTemporalMarker(0.5, () -> {
                    drive.armPivot.setPosition(1);
                    drive.stopper.setPosition(0);
                })
                .build();

        waitForStart();

        if (isStopRequested()) return;

        phoneCam.closeCameraDevice();

        drive.followTrajectory(moveToShootPower);
        drive.followTrajectory(shootPowerTwo);
        drive.followTrajectory(shootPowerThree);

        if (ringCount == 4) {
            drive.followTrajectory(goalFourRingP1);
            drive.followTrajectory(movingtoFourRings);
            drive.followTrajectory(grabRing1);
            drive.followTrajectory(grabRing2);
            drive.followTrajectory(grabRing3);
            drive.followTrajectory(movetoWobbleFour);
            drive.followTrajectory(grabWobbleGoal4ring);
            drive.followTrajectory(MovetoShootFourRings);
            drive.followTrajectory(ShootIntoGoal2);
            drive.followTrajectory(ShootIntoGoal3);
            drive.followTrajectory(ShootIntoGoal4);
            drive.followTrajectory(placeWobbleGoal2Four);
            drive.followTrajectory(parkFourRings);
        } else if (ringCount == 1) {
            drive.followTrajectory(goalOneRingP1);
            drive.followTrajectory(movingToRingOneRing);
            drive.followTrajectory(grabRing1);
            drive.followTrajectory(moveToWobbleOne);
            drive.followTrajectory(grabWobbleGoal1ring);
            drive.followTrajectory(placeWobbleGoal2One);
            drive.followTrajectory(MovetoShootOneRing);
            drive.followTrajectory(ShootIntoGoal2);
            drive.followTrajectory(parkOneRing);
        } else {
            drive.followTrajectory(goalZeroRingsP1);
            drive.followTrajectory(movetoWobbleZero);
            drive.followTrajectory(grabWobbleGoal);
            drive.followTrajectory(placeWobbleGoal2Zero);
            drive.followTrajectory(parkZeroRing);
        }
    }

    //Class used to create the pipeline
    class RingDetectingPipeline extends OpenCvPipeline {

        //Creates the YCbCr color space as a mat
        Mat YCbCr = new Mat();

        //Creates output as a mat
        Mat outPut = new Mat();

        // Creates the lower part of the rectangle as a mat
        Mat lowerCrop = new Mat();


        //Used to process the frame from the camera
        @Override
        public Mat processFrame(Mat input) {
            //Converts the input that is RGB into YCbCr
            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

            //Copies the conversion into output
            input.copyTo(outPut);

            //Creating the longer rectangle
            Rect rect1 = new Rect(rect1x, rect1y, 47, 40); //May need to be adjusted


            Scalar rectangleColor = new Scalar(0, 0, 255);

            //Drawing the rectangles on the screen
            Imgproc.rectangle(outPut, rect1, rectangleColor, 2);


            //Cropping the image for stack height

            //cropping YCbCr, putting it on loweCrop mat
            lowerCrop = YCbCr.submat(rect1); //May need to be rect1

            //Extracting the orange color, placing it on a mat
            Core.extractChannel(lowerCrop, lowerCrop, 2);

            //Creates an average using raw data, puts data on a Scalar variable
            Scalar lowerAverageOrange = Core.mean(lowerCrop);

            //Taking the first value of the average and putting it in a variable
            double finalAverage = lowerAverageOrange.val[0];

            //Comparing average values to calculate ring count
            //Needs to be adjusted through test runs
            if (finalAverage > 109) {
                ringCount = 0;
            } else if (finalAverage > 97) {
                ringCount = 1;
            } else {
                ringCount = 4;
            }

            //Outputs the average onto the driver station
            telemetry.addData("Average", finalAverage);
            telemetry.addLine("There are " + ringCount + " rings.");
            telemetry.update();

            //Returns the output into the main code
            return outPut;
        }
    }
}