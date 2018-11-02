package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sachin on 10/16/2016.
 */

@TeleOp
@Disabled
public class ShooterTest extends OpMode {
    DcMotor rightMotor;
    DcMotor leftMotor;


    @Override
    public void init() {
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {
        leftMotor.setPower(-10.0);
        rightMotor.setPower(10.0);


    }
}
