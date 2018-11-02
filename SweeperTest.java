package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sachin on 9/5/2016.
 */
@TeleOp
@Disabled
public class SweeperTest extends OpMode {
    DcMotor flickerMotor;
    //DcMotor rightMotor;

    @Override
    public void init() {
        //This gives reference to the motors from the hardware map
        flickerMotor = hardwareMap.dcMotor.get("flickerMotor");
        //rightMotor = hardwareMap.dcMotor.get("rightMotor");

        //Direction of motor
        //rightMotor.setDirection(DcMotor.Direction.REVERSE);
        flickerMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {
        //Gamepad joysticks
        float xValue = -gamepad1.left_stick_x;
        float yValue = -gamepad1.right_stick_y;
        //calculate the power needed
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;

        leftPower = Range.clip(leftPower, -1, 1);
        //+-rightPower = Range.clip(rightPower, -1, 1);

        //set the power
        flickerMotor.setPower(leftPower);
        //rightMotor.setPower(rightPower);

    }
}
