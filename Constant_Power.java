package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Sachin on 8/3/2017.
 */
/** @TeleOp(name = "Constant Power", group = "Omegabots")
@Disabled
public class Constant_Power extends LinearOpMode {
    DcMotor leftMotor;
    DcMotor rightmotor;
    @Override
    public void runOpMode()  {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightmotor = hardwareMap.dcMotor.get("rightMotor");
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            waitForStart();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        leftMotor.setPower(0.15);
        rightmotor.setPower(0.15);

        while (opModeIsActive()){
            telemetry.addData("Text", "Run at Power");
            telemetry.addData("Left Power", leftMotor.getPower());
            telemetry.addData("Right Power", rightmotor.getPower());
            telemetry.addData("Left Position", leftMotor.getCurrentPosition());
            telemetry.addData("Right Position", rightmotor.getCurrentPosition());
            telemetry.update();
        }
        leftMotor.setPower(0);
        rightmotor.setPower(0);



    }
}
*/