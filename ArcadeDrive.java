package org.firstinspires.ftc.teamcode.DrivesDrivetrain;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sachin on 9/5/2016.
 */
@Disabled
public class ArcadeDrive extends OpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;

    @Override
    public void init() {
        //This gives reference to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        //Direction of motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
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
        rightPower = Range.clip(rightPower, -1, 1);

        //set the power
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

    }
}
