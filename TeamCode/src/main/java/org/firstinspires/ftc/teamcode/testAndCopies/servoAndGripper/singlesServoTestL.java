package org.firstinspires.ftc.teamcode.testAndCopies.servoAndGripper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Tests the Left Pivot servo independent from the Right Pivot servo
 */
@Disabled
@TeleOp
public class singlesServoTestL extends LinearOpMode {

    // Declares the servo
    Servo leftPivot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes the servo
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");

        //Points the left Pivot up
        leftPivot.setPosition(1);

        waitForStart();

        while (opModeIsActive()) {
            // If dpad-up is pressed, if the left pivot is up then move down
            // else move up
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
