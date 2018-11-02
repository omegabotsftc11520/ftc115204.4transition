package org.firstinspires.ftc.teamcode.Intelitek;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Sachin on 9/5/2016.
 */
@Disabled
public class DriveWTouchSensor extends OpMode {
    //Varibles
    DcMotor leftMotor;
    DcMotor rightMotor;
    TouchSensor touchSensor;
    @Override
    public void init() {
        //Register/Init
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        touchSensor = hardwareMap.touchSensor.get("touchSensor");

    }
//TouchSensor Data
    @Override
    public void loop() {
        if (touchSensor.isPressed()) {
            leftMotor.setPower(0);
            rightMotor.setPower(0);
        }else {
            leftMotor.setPower(0.5);
            rightMotor.setPower(0.5);
        }
        telemetry.addData("isPressed", String.valueOf(touchSensor.isPressed()));

    }
}
