package org.firstinspires.ftc.teamcode.testAndCopies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

//Comments are meant for the code below, not above.
@Autonomous
@Disabled
public class OpenCvTestJan13 extends OpMode {

    //Creates the phone camera as an object
    OpenCvCamera phoneCam;

    //creates ring count as an object
    static double ringCount;

    //Creates the rectangles
    int rect1x = 60; //May need to be adjusted
    int rect1y = 160; // May need to be adjusted
    int rect2x = 60; //May need to be adjusted
    int rect2y = 160; // May need to be adjusted

    @Override
    public void init() {
        //Allows the camera to be used
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);

        //Detector
        phoneCam.setPipeline(new RingDetectingPipeline());

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

    }

    @Override
    public void loop() {
        //Tells how many rings there are in the driver station based on what the camera sees and calculates
        if (ringCount == 1){
            telemetry.addLine("There is 1 ring");
        } else if (ringCount == 4){
            telemetry.addLine("There are 4 rings");
        } else {
            telemetry.addLine("There are no rings or I can't see the rings");
        }

    }

    //Class used to create the pipeline
    class RingDetectingPipeline extends OpenCvPipeline{

        //Creates the YCbCr color space as a mat
        Mat YCbCr = new Mat();

        //Creates output as a mat
        Mat outPut = new Mat();

        //Creates the upper part of the rectangle as a mat
        Mat upperCrop = new Mat();

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
            Rect rect1 = new Rect(rect1x, rect1y, 119,69); //May need to be adjusted
            //Creating the shorter rectangle
            Rect rect2 = new Rect(rect2x, rect2y, 119, 20); // May need to be adjusted

            Scalar rectangleColor = new Scalar(0,0,255);

            //Drawing the rectangles on the screen
            Imgproc.rectangle(outPut, rect1, rectangleColor, 2);
            Imgproc.rectangle(outPut, rect2, rectangleColor, 2);

            //Cropping the image for stack height

            //cropping YCbCr, putting it on loweCrop mat
            lowerCrop = YCbCr.submat(rect1); //May need to be rect1
            //cropping YCbCr, putting it on upperCrop mat
            upperCrop = YCbCr.submat(rect2); //May need to be rect2

            //Extracting the orange color, placing it on a mat
            Core.extractChannel(lowerCrop, lowerCrop, 2);
            Core.extractChannel(upperCrop, upperCrop, 2);

            //Creates an average using raw data, puts data on a Scalar variable
            Scalar lowerAverageOrange = Core.mean(lowerCrop);
            Scalar upperAverageOrange = Core.mean(upperCrop);

            //Taking the first value of the average and putting it in a variable
            double finalLowerAverage = lowerAverageOrange.val[0];
            double finalUpperAverage = upperAverageOrange.val[0];

            //Comparing average values to calculate ring count
            //Needs to be adjusted through test runs
            if(finalLowerAverage > 15 && finalLowerAverage < 130 && finalUpperAverage < 130 ){
                ringCount = 4;
            } else if (finalLowerAverage > 10 && finalUpperAverage < 15 && finalLowerAverage > 10 && finalUpperAverage < 15){
                ringCount = 1;
            } else {
                ringCount = 0;
            }

            //Outputs the average onto the driver station
            telemetry.addData("Lower Average", finalLowerAverage);
            telemetry.addData("Upper Average", finalUpperAverage);
            telemetry.update();

            //Returns the output into the main code
            return outPut;
        }
    }

}
