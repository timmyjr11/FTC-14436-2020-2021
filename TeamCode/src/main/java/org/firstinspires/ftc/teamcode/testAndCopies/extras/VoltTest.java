package org.firstinspires.ftc.teamcode.testAndCopies.extras;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
@Disabled
@TeleOp
public class VoltTest extends LinearOpMode {


    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() {
        VoltageSensor voltageSensor = hardwareMap.voltageSensor.iterator().next();


        waitForStart();
        while (opModeIsActive()) {
            telemetry.addLine(String.format("Voltage: %.1f", voltageSensor.getVoltage()));
            telemetry.update();
        }
    }
}
