package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Sachin on 9/1/2017.
 */

public class ODS_Test extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    OpticalDistanceSensor ods;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status:", "Initialized");
        telemetry.update();

        ods = hardwareMap.opticalDistanceSensor.get("ods");
        ods.enableLed(true);
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            telemetry.addData("Light Detected: ", ods.getLightDetected());
            telemetry.update();
        }
    }
}
