package org.firstinspires.ftc.teamcode.Intelitek;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Sachin on 9/5/2016.
 */
@Disabled
public class ForwardMoveTest extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        leftMotor.setPower(0.5);
        rightMotor.setPower(0.5);

        sleep(2000);

        leftMotor.setPower(0.5);
        rightMotor.setPower(-0.5);

        leftMotor.setPower(0);
        rightMotor.setPower(0);


    }
}
