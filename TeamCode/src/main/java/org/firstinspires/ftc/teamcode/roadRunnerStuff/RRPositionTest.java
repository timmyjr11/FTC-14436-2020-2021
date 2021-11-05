package org.firstinspires.ftc.teamcode.roadRunnerStuff;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
@Disabled
public class RRPositionTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;

        Pose2d start = new Pose2d(-68,-62);

        drive.setPoseEstimate(start);

        while (opModeIsActive() && !isStopRequested()){
            telemetry.addLine("I go wait LMAO");
            telemetry.update();
        }


    }
}
