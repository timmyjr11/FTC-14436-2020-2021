package org.firstinspires.ftc.teamcode.testAndCopies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Disabled
public class HTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("ahhhhhh help");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()){
        telemetry.addLine("habooboo");
        telemetry.update();
        }
    }
}
