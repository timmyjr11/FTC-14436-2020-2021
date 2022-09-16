package org.firstinspires.ftc.teamcode.roadRunnerStuff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;

public class Halcyon_version_two extends LinearOpMode {
    SampleMecanumDrive d;

    ArrayList<Boolean> booleanArray = new ArrayList<>();
    int booleanIncrementer;
    boolean a2Pressed;
    boolean x2Pressed;
    boolean y2Pressed;
    boolean rightDpad2Pressed;

    private final FtcDashboard dash = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        d = new SampleMecanumDrive(hardwareMap);

        telemetry = new MultipleTelemetry(telemetry, dash.getTelemetry());

        d.frontShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        d.backShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        d.intake.setDirection(DcMotorSimple.Direction.REVERSE);
        d.frontShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        d.backShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        d.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        d.tapper.setPosition(0);
        d.stopper.setPosition(1);
        d.leftPivot.setPosition(0);
        d.rightPivot.setPosition(1);
        d.Gripper.setPosition(0.55);
        d.armPivot.setPosition(1);

        if (isStopRequested()) return;




    }
    private enum shooterPower {
        zeroPower,
        halfPower,
        fullPower
    }

    private enum gripperArm {
        armUp,
        armDown,
        gripClosed,
        gripOpen
    }

    private enum intakePosition {
        up,
        down
    }

    private enum driveSpeed {
        fullSpeed,
        halfSpeed,
        quarterSpeed
    }


}
