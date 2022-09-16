package org.firstinspires.ftc.teamcode.testAndCopies.servoAndGripper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class servoTest extends LinearOpMode {

    Servo leftPivot;
    Servo rightPivot;

    @Override
    public void runOpMode() {
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");

        leftPivot.setPosition(1);
        rightPivot.setPosition(0);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                if (leftPivot.getPosition() == 1 && rightPivot.getPosition() == 0) {
                    leftPivot.setPosition(0);
                    rightPivot.setPosition(1);
                } else if (leftPivot.getPosition() == 0 && rightPivot.getPosition() == 1) {
                    leftPivot.setPosition(1);
                    rightPivot.setPosition(0);
                }
                sleep(200);
            }

        }
    }
}
