package org.firstinspires.ftc.teamcode.Intelitek;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sachin on 9/5/2016.
 */
@Disabled
public class CircleDrive extends OpMode {
    //name the classes
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {
        //register motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {
        //setting power
        leftMotor.setPower(0.75);
        rightMotor.setPower(0.15);

    }
}
