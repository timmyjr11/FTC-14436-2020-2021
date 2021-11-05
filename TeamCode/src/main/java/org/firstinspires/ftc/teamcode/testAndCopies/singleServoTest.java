package org.firstinspires.ftc.teamcode.testAndCopies;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class singleServoTest extends LinearOpMode {

    Servo rightPivot;

    @Override
    public void runOpMode() {
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        rightPivot.setPosition(0);
        //Points the shooter up

        waitForStart();
        while (opModeIsActive()) {

            if (gamepad2.dpad_up) {
                if (rightPivot.getPosition() == 0) {
                    rightPivot.setPosition(1);
                } else if (rightPivot.getPosition() == 1) {
                    rightPivot.setPosition(0);
                }
                sleep(200);
            }
        }
    }
}
