package org.firstinspires.ftc.teamcode.Intelitek;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sachin on 10/4/2016.
 */

@Autonomous
@Disabled
public class BasicAutonomous extends LinearOpMode {
    public DcMotor leftMotor;
    public DcMotor rightMotor;

    @Override
    public void runOpMode() throws InterruptedException{
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor = hardwareMap.dcMotor.get("leftMotor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        rightMotor.setPower(0.5);
        leftMotor.setPower(0.5);

        sleep(3000);

        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }
}
