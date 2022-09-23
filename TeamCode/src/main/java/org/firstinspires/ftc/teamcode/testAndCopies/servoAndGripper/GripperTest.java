package org.firstinspires.ftc.teamcode.testAndCopies.servoAndGripper;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/*
    Tests the gripper and arm
 */
@Disabled
@TeleOp
public class GripperTest extends LinearOpMode {
    // Declares the servos
   Servo Gripper;
   Servo armPivot;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes the servos
        Gripper = hardwareMap.get(Servo.class,"Gripper");
        armPivot = hardwareMap.get(Servo.class, "armPivot");

        // Sets the positions to zero
        Gripper.setPosition(0);
        armPivot.setPosition(0);
        waitForStart();

        while (opModeIsActive()){
            // If b is pressed, if the gripper position is set to zero
            // then set the gripper to one, else set to zero
            if (gamepad2.b){
                if (Gripper.getPosition() == 0){
                    Gripper.setPosition(1);
                } else if (Gripper.getPosition() == 1){
                    Gripper.setPosition(0);
                }
                sleep(200);
            }

            // If a is pressed, if the arm is set to zero
            // then set the arm to one, else set to zero
            if (gamepad2.a){
                if (armPivot.getPosition() == 0){
                    armPivot.setPosition(1);
                } else if (armPivot.getPosition() == 1) {
                    armPivot.setPosition(0);
                }
                sleep(200);
            }
        }

    }
}
