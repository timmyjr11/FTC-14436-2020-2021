package org.firstinspires.ftc.teamcode.testAndCopies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous
public class timedAuto extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    private ElapsedTime runTime = new ElapsedTime();


    @Override
    public void runOpMode() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);



        telemetry.addLine("Ready to die!");
        telemetry.update();

        waitForStart();

        frontRight.setPower(1);
        backRight.setPower(1);
        frontLeft.setPower(1);
        backLeft.setPower(1);
        runTime.reset();
        telemetry.clearAll();
        while (opModeIsActive() && (runTime.milliseconds() < 1000)){
            telemetry.addLine("Hehe, I go brrrrrrr");
            telemetry.update();
        }

        frontRight.setPower(0);
        backRight.setPower(0);
        frontLeft.setPower(0);
        backLeft.setPower(0);
        runTime.reset();
        telemetry.clearAll();
        while (opModeIsActive() && (runTime.milliseconds() < 1000)){
            telemetry.addLine("LMAO I am dead inside");
            telemetry.update();
        }

        frontRight.setPower(-1);
        backRight.setPower(-1);
        frontLeft.setPower(1);
        backLeft.setPower(1);
        runTime.reset();
        telemetry.clearAll();
        while (opModeIsActive() && (runTime.milliseconds() < 1000)){
            telemetry.addLine("Help, I'm having a stroke");
            telemetry.update();
        }
        while (opModeIsActive()) {
            telemetry.addData("Time", runTime);
            telemetry.update();
        }
    }
}
