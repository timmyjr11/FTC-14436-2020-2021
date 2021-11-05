package org.firstinspires.ftc.teamcode.testAndCopies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
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

@Disabled
@Autonomous
public class scuffedAutov1 extends LinearOpMode {

    //Creates the phone camera as an object
    OpenCvCamera phoneCam;

    //creates ring count as an object
    static double ringCount;

    //Creates the rectangles
    int rect1x = 80; //May need to be adjusted //75
    int rect1y = 105; // May need to be adjusted

    //Creates motors and sets to null (0)
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor backShooter;
    DcMotor frontShooter;
    DcMotor convey;
    DcMotor intake;
    Servo stopper;
    Servo tapper;
    Servo leftPivot;
    Servo rightPivot;

    //Creates timers to be used
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime countdown = new ElapsedTime();

    @Override
    public void runOpMode() {
        //Gets the motors from the rev hub configuration.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        backShooter = hardwareMap.get(DcMotor.class, "backShooter");
        frontShooter = hardwareMap.get(DcMotor.class, "frontShooter");
        stopper = hardwareMap.get(Servo.class, "stopper");
        tapper = hardwareMap.get(Servo.class, "tapper");
        convey = hardwareMap.get(DcMotor.class, "convey");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        convey.setDirection(DcMotorSimple.Direction.REVERSE);

        tapper.setPosition(0);
        stopper.setPosition(1);
        leftPivot.setPosition(1);
        rightPivot.setPosition(0);


        //Allows the camera to be used
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Detector
        phoneCam.setPipeline(new scuffedAutov1.RingDetectingPipeline());

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

        telemetry.addLine("Ready to scan...");

        waitForStart();

        while (opModeIsActive() && countdown.milliseconds() < 150) {
            telemetry.clearAll();
            telemetry.addLine("Scanning...");
            telemetry.update();
        }

        if (opModeIsActive() && countdown.milliseconds() > 150) {
            phoneCam.closeCameraDevice();

            if (ringCount == 1) {//one ring
                frontRight.setPower(0.5);
                backRight.setPower(-0.5);
                frontLeft.setPower(-0.5);
                backLeft.setPower(0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)) {
                    telemetry.addLine("1 rings, going right");
                    telemetry.update();
                }

                frontRight.setPower(0.8);
                frontLeft.setPower(0.8);
                backLeft.setPower(0.8);
                backRight.setPower(0.8);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1200)){
                    telemetry.addLine("Going brrrrrr");
                    telemetry.update();
                }

                frontRight.setPower(-0.5);
                backRight.setPower(0.5);
                frontLeft.setPower(0.5);
                backLeft.setPower(-0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1500)) {
                    telemetry.addLine("Moving left to middle square");
                    telemetry.update();
                }

                frontRight.setPower(0.5);
                backRight.setPower(0.5);
                frontLeft.setPower(0.5);
                backLeft.setPower(0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 500)){
                    telemetry.addLine("Moving forward to middle square");
                    telemetry.update();
                }

                backRight.setPower(0);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                leftPivot.setPosition(0);
                rightPivot.setPosition(1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)){
                    telemetry.addLine("Dropping");
                    telemetry.update();
                }

                backRight.setPower(0);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                leftPivot.setPosition(1);
                rightPivot.setPosition(0);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)) {
                    telemetry.addLine("Going up to drop");
                    telemetry.update();
                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);
                frontLeft.setPower(0);

            } else if (ringCount == 4) {//4 rings
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
                frontLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)) {
                    telemetry.addLine("4 rings, going right");
                    telemetry.update();
                }

                backLeft.setPower(1);
                backRight.setPower(1);
                frontRight.setPower(1);
                frontLeft.setPower(1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1800)){
                    telemetry.addLine("Going brrrrrr");
                    telemetry.update();
                }

                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
                frontLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 300)){
                    telemetry.addLine("Moving right to disposit");
                    telemetry.update();
                }

                backLeft.setPower(-1);
                backRight.setPower(-1);
                frontRight.setPower(-1);
                frontLeft.setPower(-1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 50)){
                    telemetry.addLine("Braking");
                    telemetry.update();
                }

                backLeft.setPower(0);
                backRight.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                leftPivot.setPosition(0);
                rightPivot.setPosition(1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)){
                    telemetry.addLine("Dropping");
                    telemetry.update();
                }

                backRight.setPower(0);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                leftPivot.setPosition(1);
                rightPivot.setPosition(0);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)) {
                    telemetry.addLine("Going up to drop");
                    telemetry.update();
                }

                backLeft.setPower(-1);
                frontRight.setPower(-1);
                rightPivot.setPosition(0);
                leftPivot.setPosition(1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)){
                    telemetry.addLine("Getting out of the way");
                    telemetry.update();
                }

                frontRight.setPower(-0.5);
                backRight.setPower(-0.5);
                frontLeft.setPower(-0.5);
                backLeft.setPower(-0.5);
                rightPivot.setPosition(0);
                leftPivot.setPosition(1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 250)){
                    telemetry.addLine("Backing up");
                    telemetry.update();
                }

                backRight.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                frontLeft.setPower(0);

            } else {//0 rings
                frontRight.setPower(0.5);
                backRight.setPower(-0.5);
                frontLeft.setPower(-0.5);
                backLeft.setPower(0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)) {
                    telemetry.addLine("0 rings, going right");
                    telemetry.update();
                }

                frontRight.setPower(0.8);
                frontLeft.setPower(0.8);
                backLeft.setPower(0.8);
                backRight.setPower(0.8);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1150)){
                    telemetry.addLine("Going brrrrrr");
                    telemetry.update();
                }

                frontRight.setPower(0.5);
                frontLeft.setPower(-0.5);
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 400)){
                    telemetry.addLine("Moving right to line up");
                    telemetry.update();
                }

                frontRight.setPower(-1);
                frontLeft.setPower(-1);
                backLeft.setPower(-1);
                backRight.setPower(-1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 50)){
                    telemetry.addLine("Braking");
                    telemetry.update();
                }

                backRight.setPower(0);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                leftPivot.setPosition(0);
                rightPivot.setPosition(1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)){
                    telemetry.addLine("Dropping");
                    telemetry.update();
                }

                backRight.setPower(0);
                backLeft.setPower(0);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                leftPivot.setPosition(1);
                rightPivot.setPosition(0);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)){
                    telemetry.addLine("Going up to drop");
                    telemetry.update();
                }

                backRight.setPower(-0.5);
                backLeft.setPower(-0.5);
                frontLeft.setPower(-0.5);
                frontRight.setPower(-0.5);
                rightPivot.setPosition(0);
                leftPivot.setPosition(1);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 500)){
                    telemetry.addLine("Backing up");
                    telemetry.update();
                }

                rightPivot.setPosition(0);
                leftPivot.setPosition(1);
                frontRight.setPower(-0.5);
                backRight.setPower(0.5);
                frontLeft.setPower(0.5);
                backLeft.setPower(-0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)) {
                    telemetry.addLine("Going left to park");
                    telemetry.update();
                }

                frontRight.setPower(0.5);
                frontLeft.setPower(0.5);
                backLeft.setPower(0.5);
                backRight.setPower(0.5);
                telemetry.clearAll();
                runtime.reset();

                while (opModeIsActive() && (runtime.milliseconds() < 1000)){
                    telemetry.addLine("Going forward to park");
                    telemetry.update();
                }


            }
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
            Rect rect1 = new Rect(rect1x, rect1y, 55,40); //May need to be adjusted


            Scalar rectangleColor = new Scalar(0,0,255);

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
            if(finalAverage > 120 ){
                ringCount = 0;
            } else if (finalAverage > 101.5){
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
