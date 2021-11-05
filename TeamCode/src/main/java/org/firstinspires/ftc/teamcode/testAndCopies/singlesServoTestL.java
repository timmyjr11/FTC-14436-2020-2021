package org.firstinspires.ftc.teamcode.testAndCopies;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class singlesServoTestL extends LinearOpMode {

    Servo leftPivot;

    @Override
    public void runOpMode() throws InterruptedException {
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        leftPivot.setPosition(1);
        //points up

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                if (leftPivot.getPosition() == 1) {
                    leftPivot.setPosition(0);
                } else if (leftPivot.getPosition() == 0) {
                    leftPivot.setPosition(1);
                }
                sleep(200);
            }
        }
    }
}
