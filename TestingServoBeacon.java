package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Sachin on 11/13/2016.
 */

@Disabled
public class TestingServoBeacon extends OpMode {
    Servo beaconPresserServo;
    @Override
    public void init() {
        beaconPresserServo = hardwareMap.servo.get("beaconPresserServo");

    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            beaconPresserServo.setPosition(180);
        }
        if (gamepad1.a){
            beaconPresserServo.setPosition(-180);
        }

    }
}
