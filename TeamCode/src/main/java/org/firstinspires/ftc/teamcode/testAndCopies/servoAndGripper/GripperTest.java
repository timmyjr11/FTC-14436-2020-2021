package org.firstinspires.ftc.teamcode.testAndCopies.servoAndGripper;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class GripperTest extends LinearOpMode {
   Servo Gripper;
   Servo armPivot;

    @Override
    public void runOpMode() throws InterruptedException {
        Gripper = hardwareMap.get(Servo.class,"Gripper");
        armPivot = hardwareMap.get(Servo.class, "armPivot");

        Gripper.setPosition(0);
        armPivot.setPosition(0);
        waitForStart();

        while (opModeIsActive()){
            if (gamepad2.b){
                if (Gripper.getPosition() == 0){
                    Gripper.setPosition(1);
                } else if (Gripper.getPosition() == 1){
                    Gripper.setPosition(0);
                }
                sleep(200);
            }

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
