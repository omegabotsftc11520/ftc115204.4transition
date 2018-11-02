package org.firstinspires.ftc.teamcode.Intelitek;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sachin on 9/5/2016.
 */
@Disabled
public class FullControlTest extends OpMode {
    final double LEFT_OPEN_POSITION = 0.0;
    final double LEFT_CLOSED_POSITION = 0.5;
    final double RIGHT_OPEN_POSITION = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.5;

    DcMotor leftMotor;
    DcMotor rightMotor;
    Servo leftClaw;
    Servo rightClaw;
    DcMotor armMotor;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
        armMotor = hardwareMap.dcMotor.get("armMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

    }

    @Override
    public void loop() {
        float xValue = gamepad1.left_stick_x;
        float yValue = gamepad1.right_stick_y;

        float leftPower = yValue + xValue;
        float rightPower = yValue + xValue;

        leftPower = Range.clip(leftPower, -1,1);
        rightPower = Range.clip(rightPower, -1, 1);

        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);

        if(gamepad1.y) {
            armMotor.setPower(0.1);
        }
        else if(gamepad1.b) {
            armMotor.setPower(-0.1);
        }
        else {
            armMotor.setPower(0);
        }

        if (gamepad1.x) {
            leftClaw.setPosition(LEFT_OPEN_POSITION);
            rightClaw.setPosition(RIGHT_OPEN_POSITION);
        }
        if (gamepad1.a){
            leftClaw.setPosition(LEFT_CLOSED_POSITION);
            rightClaw.setPosition(RIGHT_CLOSED_POSITION);
        }


    }
}
