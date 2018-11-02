package org.firstinspires.ftc.teamcode.RelicRecovery;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RelicRecovery.Constants;

/**
 * Created by Sachin on 8/29/2017.
 */

public class OmegabotsHardware implements Constants {
    /* Public OpMode members. */
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor leftBackMotor = null;
    public DcMotor glyphSlide = null;
    public ColorSensor colorSensorJewel = null;
    public ColorSensor colorSensorRightLineUp = null;
    public ColorSensor colorSensorLeftLineUp = null;
    public DeviceInterfaceModule deviceInterfaceModule = null;
    public Servo leftClaw = null;
    public Servo rightClaw = null;
    public Servo holdRelicClaw = null;
    public Servo wallRelicClaw = null;
    public Servo jewelPusher =  null;
    public DcMotor relicPlacer = null;
    public GyroSensor gyro = null;
    //public static final double MID_SERVO       =  0.5 ;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public OmegabotsHardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFrontMotor = hwMap.dcMotor.get("lFM");
        rightFrontMotor = hwMap.dcMotor.get("rFM");
        leftBackMotor = hwMap.dcMotor.get("lBM");
        rightBackMotor = hwMap.dcMotor.get("rBM");
        glyphSlide = hwMap.dcMotor.get("glyphSlide");
        relicPlacer = hwMap.dcMotor.get("relicPlacer");

        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors

        colorSensorJewel = hwMap.colorSensor.get("csJewel");
        deviceInterfaceModule = hwMap.deviceInterfaceModule.get("Device Interfaced Module 1");
        wallRelicClaw = hwMap.servo.get("wRC");
        holdRelicClaw = hwMap.servo.get("hRC");

        leftClaw = hwMap.servo.get("leftClaw");
        rightClaw = hwMap.servo.get("rightClaw");

        jewelPusher = hwMap.servo.get("jewelPusher");

        gyro = (ModernRoboticsI2cGyro)hwMap.gyroSensor.get("gyro");

        // Set all motors to zero power
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);


    }



}
