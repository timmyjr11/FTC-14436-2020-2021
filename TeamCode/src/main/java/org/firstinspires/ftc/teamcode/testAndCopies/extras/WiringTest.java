package org.firstinspires.ftc.teamcode.testAndCopies.extras;
        ;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class WiringTest extends LinearOpMode {

    DcMotorEx frontRight;
    DcMotorEx frontLeft;
    DcMotorEx backRight;
    DcMotorEx backLeft;
    DcMotorEx backShooter;
    DcMotorEx frontShooter;
    Servo stopper;
    Servo tapper;
    DcMotor convey;
    DcMotor intake;
    Servo leftPivot;
    Servo rightPivot;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        stopper = hardwareMap.get(Servo.class, "stopper");
        tapper = hardwareMap.get(Servo.class, "tapper");
        convey = hardwareMap.get(DcMotor.class, "convey");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");

        waitForStart();

        tapper.setPosition(0);
        stopper.setPosition(1);
        leftPivot.setPosition(0);
        rightPivot.setPosition(1);

        while (opModeIsActive()){

            telemetry.addLine("a = FrontLeft & y = frontRight");
            telemetry.addLine("b = backRight & x = backLeft");
            telemetry.addLine("Dpad up = Left Pivot & Dpad left = tapper");
            telemetry.addLine("Dpad right = Stopper & right bumper = front shooter");
            telemetry.addLine("left bumper = back shooter & left trigger = convey");
            telemetry.addLine("right trigger = Intake");
            telemetry.update();

            if (gamepad1.a){
                frontLeft.setPower(1);
                telemetry.addLine("Front Left");
            }

            if (gamepad1.y){
                frontRight.setPower(1);
                telemetry.addLine("Front Right");
            }

            if (gamepad1.b){
                backRight.setPower(1);
                telemetry.addLine("Back Right");
            }

            if (gamepad1.x){
                backLeft.setPower(1);
                telemetry.addLine("Back Left");
            }

            if (gamepad1.dpad_up){
                leftPivot.setPosition(1);
                telemetry.addLine("Left Pivot");
            } else {
                leftPivot.setPosition(0);
            }

            if (gamepad1.dpad_left){
                tapper.setPosition(1);
                telemetry.addLine("Tapper");
            } else {
                tapper.setPosition(0);
            }

            if (gamepad1.dpad_right){
                stopper.setPosition(0);
                telemetry.addLine("Stopper");
            } else {
                stopper.setPosition(1);
            }

            if (gamepad1.right_bumper){
                frontShooter.setPower(0.5);
                telemetry.addLine("Front Shooter");
            }

            if (gamepad1.left_bumper){
                backShooter.setPower(0.5);
                telemetry.addLine("Back Shooter");
            }

            if (gamepad1.left_trigger > 0.5){
                convey.setPower(1);
                telemetry.addLine("Convey");
            }

            if (gamepad1.right_trigger > 0.5){
                intake.setPower(1);
                telemetry.addLine("Intake");
            }
        }
    }
}
