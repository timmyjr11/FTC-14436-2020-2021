package org.firstinspires.ftc.teamcode.testAndCopies.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@Autonomous
public class ThermalEquilibriumPIDTuner extends LinearOpMode {

    DcMotorEx testMotor;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    public static double reference = 100;
    ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        testMotor = hardwareMap.get(DcMotorEx.class, "frontShooter");
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        while (opModeIsActive()) {
            double power = PIDControl(reference, testMotor.getVelocity());
            telemetry.addData("Reference",reference);
            telemetry.addData("current Velocity", testMotor.getVelocity());
            telemetry.update();
            testMotor.setPower(power);
        }
    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;
        timer.reset();

        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}
