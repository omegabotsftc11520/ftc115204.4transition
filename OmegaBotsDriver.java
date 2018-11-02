package org.firstinspires.ftc.teamcode.GitHub;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sachin on 9/5/2016.
 */
@TeleOp(name="LarsonFTC: Driver Controlled", group="LarsonFTCRobot")
@Disabled
public class OmegaBotsDriver extends OpMode {
    //Name the Motors and Servos in use
    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor scooperMotor;
    //DcMotor leftShotMotor;
    //DcMotor rightShotMotor;
    Servo beaconPresserServo;

    @Override
    public void init() {
        //This gives reference to the motors from the hardware map
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        scooperMotor = hardwareMap.dcMotor.get("scooperMotor");
        //leftShotMotor = hardwareMap.dcMotor.get("leftShotMotor");
        //rightShotMotor = hardwareMap.dcMotor.get("rightShotMotor");


        beaconPresserServo = hardwareMap.servo.get("beaconPresserServo");

        //Direction of motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        scooperMotor.setDirection(DcMotor.Direction.FORWARD);
        //leftShotMotor.setDirection(DcMotor.Direction.FORWARD);
        //rightShotMotor.setDirection(DcMotor.Direction.REVERSE);
        //Direction of Servo
        beaconPresserServo.setDirection(Servo.Direction.REVERSE );
        beaconPresserServo.setPosition(-0.5);

    }

    @Override
    public void loop() {
        //Gamepad joysticks being used
        float xValue = -gamepad1.left_stick_x;
        float yValue = -gamepad1.right_stick_y;
        float xValueShot = -gamepad2.left_stick_x;
        float yValueShot = -gamepad2.right_stick_y;


        //calculate the power needed
        float leftPower = yValue + xValue;
        float rightPower = yValue - xValue;
        float leftShotPower = yValueShot + xValueShot;
        float rightShotPower = yValueShot - xValueShot;

        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        leftShotPower = Range.clip(leftShotPower, -1, 1);
        rightShotPower = Range.clip(rightShotPower, -1,1);


        //set the power of motors with wheels
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        //leftShotMotor.setPower(leftShotPower);
        //rightShotMotor.setPower(rightShotPower);
        //Servo Position
        if (gamepad1.x){
            beaconPresserServo.setPosition(0.5);
            //beaconPresserServo.scaleRange(0,45);
        }
        if (gamepad1.a){
            beaconPresserServo.setPosition(-0.5);
            //beaconPresserServo.scaleRange(-45,0);
        }
        //Scooper Speed
        if (gamepad2.right_bumper) {
            scooperMotor.setPower(1.0);
        }
        if (gamepad2.left_bumper){
            scooperMotor.setPower(-1.0);
        }
        if (gamepad2.y){
            scooperMotor.setPower(0.0);
        }


    }
}
