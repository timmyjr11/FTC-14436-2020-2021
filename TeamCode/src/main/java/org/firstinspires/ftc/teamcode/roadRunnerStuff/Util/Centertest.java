package org.firstinspires.ftc.teamcode.roadRunnerStuff.Util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadRunnerStuff.SampleMecanumDrive;
@Disabled
@Autonomous
public class Centertest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d start = new Pose2d(0,0);

        Trajectory test = drive.trajectoryBuilder(start)
                .strafeTo(new Vector2d(-58, -37))
                .build();

        drive.followTrajectory(test);

    }
}
