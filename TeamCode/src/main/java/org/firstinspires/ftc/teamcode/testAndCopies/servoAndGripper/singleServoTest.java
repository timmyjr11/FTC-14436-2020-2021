package org.firstinspires.ftc.teamcode.testAndCopies.servoAndGripper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Tests the Right Pivot servo independent from the Left Pivot servo
 */
@Disabled
@TeleOp
public class singleServoTest extends LinearOpMode {

    // Declares the servo
    Servo rightPivot;

    @Override
    public void runOpMode() {
        // Initializes the servo
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");

        // Points the right pivot up
        rightPivot.setPosition(0);

        waitForStart();
        while (opModeIsActive()) {
            // If dpad-up is pressed, if the left pivot is up then move down
            // else move up
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
