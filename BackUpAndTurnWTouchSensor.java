package org.firstinspires.ftc.teamcode.Intelitek;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Sachin on 9/5/2016.
 */
@Disabled
public class BackUpAndTurnWTouchSensor extends LinearOpMode {
    //Varibles
    DcMotor leftMotor;
    DcMotor rightMotor;
    TouchSensor touchSensor;

    int BACKUP_TIME = 1000;
    int TURN_TIME = 250;

    @Override
    public void runOpMode() throws InterruptedException {
        //Regiester Motors
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        //setting the direction

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        touchSensor = hardwareMap.touchSensor.get("touchSensor");

        waitForStart();

        //touch sensor data
        while(true) {
            if (touchSensor.isPressed()) {
                leftMotor.setPower(-0.25);
                rightMotor.setPower(-0.25);
                telemetry.addData("State", "Backing Up");
                sleep(BACKUP_TIME);

                leftMotor.setPower(0.25);
                rightMotor.setPower(-0.25);
                telemetry.addData("State", "Turning");
                sleep(TURN_TIME);
            }else {
                leftMotor.setPower(0.5);
                rightMotor.setPower(0.5);
                telemetry.addData("State", "Driving");
            }


        }
    }

}
