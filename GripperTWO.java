package org.firstinspires.ftc.teamcode.Intelitek;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sachin on 9/5/2016.
 */
@Disabled
public class GripperTWO extends OpMode {
    final double LEFT_OPEN_POSITION = 0.0;
    final double LEFT_CLOSED_POSITION = 0.5;
    final double RIGHT_OPEN_POSITION = 1.0;
    final double RIGHT_CLOSED_POSITION = 0.5;

    Servo leftClaw;
    Servo rightClaw;

    @Override
    public void init() {
        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");

    }

    @Override
    public void loop() {
        if(gamepad1.a) {
            leftClaw.setPosition(LEFT_OPEN_POSITION);
            rightClaw.setPosition(RIGHT_OPEN_POSITION);
        }else {
            leftClaw.setPosition(LEFT_CLOSED_POSITION);
            rightClaw.setPosition(RIGHT_CLOSED_POSITION);
        }

    }
}
