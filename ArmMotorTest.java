package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sachin on 9/5/2016.
 */
@Disabled
public class ArmMotorTest extends OpMode {
    //Write down varibles

    DcMotor armMotor;

    @Override
    public void init() {
        //init the motors
        armMotor = hardwareMap.dcMotor.get("armMotor");

    }

    @Override
    public void loop() {
        //this will make the arm go up and down
        // the arm will be using buttons y(up) b(down)
        //setting the power
        if(gamepad1.y){
            armMotor.setPower(0.1);
        }
        else if(gamepad1.b){
            armMotor.setPower(-0.1);
        }
        else {
            armMotor.setPower(0);
        }

    }
}
