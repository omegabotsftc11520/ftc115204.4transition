package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by Sachin on 8/2/2017.
 */

@TeleOp(name = "Wall Follow", group = "Omegabots")
//@Disabled
public class optical_Distance_Sensor extends LinearOpMode {

    OpticalDistanceSensor ods;

    //Motors
    DcMotor leftMotor;
    DcMotor rightMotor;

    //Raw value is between 0 and 1
    double odsReadingRaw;

    // odsReadingRaw to the power of (-0.5)
    static double odsReadingLinear;

    @Override
    public void runOpMode() throws InterruptedException {

        ods = hardwareMap.opticalDistanceSensor.get("ods");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {

            odsReadingRaw = ods.getRawLightDetected() / 5;
            odsReadingLinear = Math.pow(odsReadingRaw, 0.5);
            leftMotor.setPower(odsReadingLinear * 2);
            rightMotor.setPower(0.5 - (odsReadingLinear * 2));

            telemetry.addData("0 ODS Raw", odsReadingRaw);
            telemetry.addData("1 ODS linear", odsReadingLinear);
            telemetry.addData("2 Motor Left", leftMotor.getPower());
            telemetry.addData("3 Motor Right", rightMotor.getPower());
            telemetry.update();
        }
    }
}