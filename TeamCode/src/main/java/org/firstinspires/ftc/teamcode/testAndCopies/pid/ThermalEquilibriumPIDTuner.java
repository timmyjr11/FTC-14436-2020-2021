package org.firstinspires.ftc.teamcode.testAndCopies.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
    A PID tuner to use for a PID loop on motors, made from Thermal Equilibrium
 */
@Disabled
@Config
@Autonomous
public class ThermalEquilibriumPIDTuner extends LinearOpMode {

    // Declares the motor
    DcMotorEx testMotor;

    // Declares the FTC Dashboard
    private final FtcDashboard dashboard = FtcDashboard.getInstance();


    // Creates the variables to be used
    double integralSum = 0;
    public static double Kp = 0;
    public static double Ki = 0;
    public static double Kd = 0;
    public static double Kf = 0;
    public static double reference = 100;
    private double lastError = 0;

    // Creates a timer
    ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        // Initializes the Dashboard and motors and sets up the motor configuration
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        testMotor = hardwareMap.get(DcMotorEx.class, "frontShooter");
        testMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            // Power is set to the output of the PIDControl function
            double power = PIDControl(reference, testMotor.getVelocity());

            // Shows data about the reference variable and the current velocity
            // then sets the power to the power variable
            telemetry.addData("Reference",reference);
            telemetry.addData("current Velocity", testMotor.getVelocity());
            telemetry.update();
            testMotor.setPower(power);
        }
    }

    // Function that uses calculates the PID, uses two double arguments
    public double PIDControl(double reference, double state) {

        // Error is created from where is should be - what it is currently at
        double error = reference - state;
        // Integral sum is created from adding the integral sum to error times the current time
        integralSum += error * timer.seconds();
        // Derivative is created from the current error - last error divided by time
        double derivative = (error - lastError) / timer.seconds();
        // Make the last error the current error
        lastError = error;
        // Reset the time
        timer.reset();

        // Create the output then return the output
        double output = (error * Kp) + (derivative * Kd) + (integralSum * Ki) + (reference * Kf);
        return output;
    }
}
