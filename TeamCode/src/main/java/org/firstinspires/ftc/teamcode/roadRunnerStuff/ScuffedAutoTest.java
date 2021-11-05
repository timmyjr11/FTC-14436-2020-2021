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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Arrays;

@TeleOp
@Config
@Disabled
public class ScuffedAutoTest extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    //Creates the phone camera as an object
    OpenCvCamera phoneCam;

    //creates ring count as an object
    static double ringCount;

    public static double ringTwo = 9;
    public static double ringThree = 7;
    public static double x = -20;
    public static double y = -3.5;


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
        backVelocity = 1710;

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
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Detector
        phoneCam.setPipeline(new ScuffedAutoTest.RingDetectingPipeline());

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

        waitForStart();

        phoneCam.closeCameraDevice();

        if (isStopRequested()) return;

        Pose2d start = new Pose2d(-71, -35);

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

        Trajectory movingToRing = drive.trajectoryBuilder(shootPowerThree.end())
                .splineToLinearHeading(new Pose2d(-11, -33.5, Math.toRadians(182)), Math.toRadians(0))
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

        Trajectory grabRing1 = drive.trajectoryBuilder(movingToRing.end())
                .forward(16,
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
                .forward(5,
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
                .forward(4,
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


        Trajectory ShootIntoGoal = drive.trajectoryBuilder(grabRing3.end())
                .lineToLinearHeading(new Pose2d(-17.5, -32.5, Math.toRadians(0)))

                .addTemporalMarker(0.5, () -> {
                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })

                .addTemporalMarker(1, () -> {
                    drive.intake.setPower(0);

                    drive.leftPivot.setPosition(1);
                    drive.rightPivot.setPosition(0);

                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })

                .addTemporalMarker(1.2, () -> {
                    drive.stopper.setPosition(0);
                    drive.frontShooter.setVelocity(1300);
                    drive.backShooter.setVelocity(1950);
                })
                .build();

        Trajectory ShootIntoGoal2 = drive.trajectoryBuilder(ShootIntoGoal.end())
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


        Trajectory goalZeroRingsP1 = drive.trajectoryBuilder(shootPowerThree.end())
                .splineToConstantHeading(new Vector2d(-15, -60), Math.toRadians(0))

                .addTemporalMarker(1.25, () -> drive.armPivot.setPosition(0.1))

                .addTemporalMarker(1.75, () -> drive.Gripper.setPosition(0.7))

                .addTemporalMarker(2.25, () -> drive.armPivot.setPosition(1))

                .addTemporalMarker(2.75, () -> drive.Gripper.setPosition(0))
                .build();

        //Work later on  Trajectory goalZeroRingP2

        Trajectory goalOneRingP1 = drive.trajectoryBuilder(shootPowerThree.end())
                .splineToConstantHeading(new Vector2d(10, -32), Math.toRadians(0))

                .addTemporalMarker(1, () -> drive.armPivot.setPosition(0.1))

                .addTemporalMarker(1.5, () -> drive.Gripper.setPosition(0.7))

                .addTemporalMarker(2, () -> drive.armPivot.setPosition(1))

                .addTemporalMarker(2.5, () -> drive.Gripper.setPosition(0))

                .build();

        //Work later on Trajectory goalOneRingP2

        Trajectory goalFourRingP1 = drive.trajectoryBuilder(shootPowerThree.end())
                .splineToConstantHeading(new Vector2d(35, -55), Math.toRadians(0))

                .addTemporalMarker(1.5, () -> drive.armPivot.setPosition(0.1))

                .addTemporalMarker(2, () -> drive.Gripper.setPosition(0.7))

                .addTemporalMarker(2.5, () -> drive.armPivot.setPosition(1))

                .addTemporalMarker(3, () -> drive.Gripper.setPosition(0))

                .build();


        Trajectory park = drive.trajectoryBuilder(ShootIntoGoal.end())
                .lineTo(new Vector2d(0, -29))
                .build();


        drive.followTrajectory(moveToShootPower);
        drive.followTrajectory(shootPowerTwo);
        drive.followTrajectory(shootPowerThree);


        if (ringCount == 4) {
            drive.followTrajectory(goalFourRingP1);
        } else if (ringCount == 1) {
            drive.followTrajectory(goalOneRingP1);
        } else {
            drive.followTrajectory(goalZeroRingsP1);
        }

        if (ringCount == 4) {
            drive.followTrajectory(movingToRing);
            drive.followTrajectory(grabRing1);
            drive.followTrajectory(grabRing2);
            drive.followTrajectory(grabRing3);
            drive.followTrajectory(ShootIntoGoal);
            drive.followTrajectory(ShootIntoGoal2);
            drive.followTrajectory(ShootIntoGoal3);
            drive.followTrajectory(ShootIntoGoal4);

        } else if (ringCount == 1) {
            drive.followTrajectory(movingToRing);
            drive.followTrajectory(grabRing1);
            drive.followTrajectory(ShootIntoGoal);
            drive.followTrajectory(ShootIntoGoal2);
        }

        drive.followTrajectory(park);
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
            if (finalAverage > 107) {
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