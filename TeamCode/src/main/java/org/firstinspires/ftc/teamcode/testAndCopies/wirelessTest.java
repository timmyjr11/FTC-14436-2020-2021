package org.firstinspires.ftc.teamcode.testAndCopies;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class wirelessTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        while (opModeIsActive()){
            telemetry.addLine("Yo wat,  it works?");
            telemetry.update();
        }
    }
}
