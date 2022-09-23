package org.firstinspires.ftc.teamcode.testAndCopies.servoAndGripper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Tests the Pivot servos
 */
@Disabled
@TeleOp
public class servoTest extends LinearOpMode {

    // Declares the servos
    Servo leftPivot;
    Servo rightPivot;

    @Override
    public void runOpMode() {
        // Initializes the servos
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");

        // Points the servos up
        leftPivot.setPosition(1);
        rightPivot.setPosition(0);
        waitForStart();

        while (opModeIsActive()) {
            // If dpad-up is pressed, if the pivot is up then move the pivot down
            // else then move the pivot up
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
