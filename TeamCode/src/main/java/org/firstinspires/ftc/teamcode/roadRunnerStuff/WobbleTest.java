package org.firstinspires.ftc.teamcode.roadRunnerStuff;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
@TeleOp
@Disabled
public class WobbleTest extends LinearOpMode {

   double wobbleBlack = 0;
   double wobbleBlue = 0;

    //Creates the phone camera as an object
    OpenCvCamera phoneCam;

    //creates ring count as an object
    static double ringCount;

    int rect1x = 179; //May need to be adjusted //80
    int rect1y = 107; // May need to be adjust

    int rect2x = 30;
    int rect2y = 30;


    @Override
    public void runOpMode() throws InterruptedException {
        //Allows the camera to be used
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Detector
        phoneCam.setPipeline(new WobbleTest.RingDetectingPipeline());

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

        phoneCam.stopRecordingPipeline();

        while (opModeIsActive()) {

            phoneCam.setPipeline(new WobbleTest.wobbleDetectingPipeline());
        }
    }

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

        class wobbleDetectingPipeline extends OpenCvPipeline {

            Mat YCbCr = new Mat();

            //Creates output as a mat
            Mat OutPut = new Mat();

            // Creates the lower part of the rectangle as a mat
            Mat lowerCrop = new Mat();

            Mat upperCrop = new Mat();


            @Override
            public Mat processFrame(Mat input) {

                Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);

                input.copyTo(OutPut);

                Rect rect1 = new Rect(rect1x, rect1y, 47, 40); //May need to be adjusted
                Rect rect2 = new Rect(rect2x, rect2y, 10, 50);

                Scalar black = new Scalar(0, 0, 0);
                Scalar blue = new Scalar(255, 0, 0);

                Imgproc.rectangle(OutPut, rect1, black, 2);
                Imgproc.rectangle(OutPut, rect2, blue, 2);

                upperCrop = YCbCr.submat(rect2); //May need to be rect1
                lowerCrop = YCbCr.submat(rect1); //May need to be rect1

                Scalar blackAverage = Core.mean(lowerCrop);
                Scalar blueAverage = Core.mean(upperCrop);

                double blackFinalAverage = blackAverage.val[0];
                double blueFinalAverage = blueAverage.val[0];

                return OutPut;


            }
        }
    }
