package org.firstinspires.ftc.teamcode.testAndCopies;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp
public class shooterTest extends LinearOpMode {

    DcMotor backShooter;
    DcMotor frontShooter;
    DcMotor convey;
    Servo stopper;
    Servo tapper;

    @Override
    public void runOpMode() throws InterruptedException {
       backShooter = hardwareMap.get(DcMotor.class, "backShooter");
       frontShooter = hardwareMap.get(DcMotor.class, "frontShooter");
       stopper = hardwareMap.get(Servo.class, "stopper");
       tapper = hardwareMap.get(Servo.class, "tapper");
       convey = hardwareMap.get(DcMotor.class, "convey");

       frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
       backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
       convey.setDirection(DcMotorSimple.Direction.REVERSE);

       tapper.setPosition(0);
       stopper.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive() && !isStopRequested()){
            if (gamepad1.y) {
                backShooter.setPower(0.7);
                frontShooter.setPower(0.7);
            }  else {
                backShooter.setPower(0);
                frontShooter.setPower(0);
            }

            if (gamepad1.a){
                for (int count = 0; count < 6; count++) {
                    if (tapper.getPosition() == 0) {
                        tapper.setPosition(1);
                    } else if (tapper.getPosition() == 1){
                        tapper.setPosition(0);
                    }
                    sleep(250);
                }
            }

            if (gamepad1.b){
                convey.setPower(1);
            } else{
                convey.setPower(0);
            }
            if (gamepad1.x){
                stopper.setPosition(1);
            }
        }
    }
}
