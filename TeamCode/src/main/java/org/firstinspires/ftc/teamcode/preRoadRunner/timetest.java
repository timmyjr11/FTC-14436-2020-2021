package org.firstinspires.ftc.teamcode.preRoadRunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp
@Config
public class timetest extends LinearOpMode {

    DcMotorEx backShooter;
    DcMotorEx frontShooter;
    DcMotor convey;
    DcMotor intake;
    Servo stopper;
    Servo tapper;
    Servo leftPivot;
    Servo rightPivot;
    Servo Gripper;
    Servo armPivot;

    ElapsedTime timer = new ElapsedTime();
    ElapsedTime StopperTime = new ElapsedTime();

    public static double Velocity;
    public static double Velocity2;

    public static PIDFCoefficients PIDF = new PIDFCoefficients(20,0,10, 13.5);
    public static PIDFCoefficients PIDB = new PIDFCoefficients(7,0,4,12);

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {



        backShooter = hardwareMap.get(DcMotorEx.class, "backShooter");
        frontShooter = hardwareMap.get(DcMotorEx.class, "frontShooter");
        stopper = hardwareMap.get(Servo.class, "stopper");
        tapper = hardwareMap.get(Servo.class, "tapper");
        convey = hardwareMap.get(DcMotor.class, "convey");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPivot = hardwareMap.get(Servo.class, "leftPivot");
        rightPivot = hardwareMap.get(Servo.class, "rightPivot");
        Gripper = hardwareMap.get(Servo.class, "Gripper");
        armPivot = hardwareMap.get(Servo.class, "armPivot");

        frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        backShooter.setDirection(DcMotorSimple.Direction.REVERSE);

        frontShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDF);
        backShooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, PIDB);

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        Velocity = 1340;
        Velocity2 = 2040;


        waitForStart();

        rightPivot.setPosition(0);
        leftPivot.setPosition(1);
        Gripper.setPosition(0.5);
        armPivot.setPosition(1);
        tapper.setPosition(0);
        stopper.setPosition(1);
        ;


        while (opModeIsActive()){
            if (gamepad2.dpad_down){
                shootingTest();
            }
            Tele();
        }

    }
    private void shootingTest(){
            timer.reset();
            speed();
            stopper.setPosition(0);
            while (timer.milliseconds() < 2500){
                Tele();
            }

            tapper.setPosition(1);
            speed();
            timer.reset();
            while (timer.milliseconds() < 200){
                Tele();
            }

            tapper.setPosition(0);
            speed();
            timer.reset();
            while (timer.milliseconds() < 400){
                Tele();
            }

            tapper.setPosition(1);
            speed();
            timer.reset();
            while (timer.milliseconds() < 200){
                Tele();
            }

            tapper.setPosition(0);
            speed();
            timer.reset();
            while (timer.milliseconds() < 400){
                Tele();
            }

            tapper.setPosition(1);
            speed();
            timer.reset();

            while (timer.milliseconds() < 200){
                Tele();
            }

            tapper.setPosition(0);
            stopper.setPosition(1);
            speed();
            timer.reset();
            while (timer.milliseconds() < 200){
                Tele();
            }

            frontShooter.setVelocity(0);
            backShooter.setVelocity(0);
    }

    private void speed() {
        frontShooter.setVelocity(Velocity);
        backShooter.setVelocity(Velocity2);
    }

    private void Tele(){
        telemetry.addData("FrontVelo",frontShooter.getVelocity());
        telemetry.addData("BackVelo", backShooter.getVelocity());
        telemetry.addData("TargetVelo", Velocity);
        telemetry.addData("BackTargetVelo", Velocity2);
        telemetry.addData("Ticks FS", frontShooter.getCurrentPosition());
        telemetry.addData("Ticks BS", backShooter.getCurrentPosition());
        telemetry.addData("Front Error", frontShooter.getVelocity() - Velocity);
        telemetry.addData("back Error", backShooter.getVelocity() - Velocity2);

        telemetry.update();
    }
}
