package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RelicRecovery.Constants;

/**
 * Created by Sachin on 8/29/2017.
 */

public class OmegabotsHardwareRover implements ConstantsRover {
    /* Public OpMode members. */
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor rightBackMotor = null;
    public DcMotor leftBackMotor = null;

    public DcMotor sampleSlide = null;
    public DcMotor latchHolder =  null;
    public DcMotor mainArm = null;
    public DcMotor collectorArm = null;

    public CRServo collector = null;

    public Servo teamMarker = null;

    //public DigitalChannel gyro = null;
    //public DigitalChannel touchSensor = null;
    //public static final double MID_SERVO       =  0.5 ;


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();


    /* Constructor */
    public OmegabotsHardwareRover(){

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
        sampleSlide = hwMap.dcMotor.get("sampleSlide");
        latchHolder = hwMap.dcMotor.get("latchHolder");
        mainArm = hwMap.dcMotor.get("mainArm");
        collectorArm = hwMap.dcMotor.get("collectorArm");
        teamMarker = hwMap.servo.get("teamMarker");


        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightBackMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        latchHolder.setDirection(DcMotorSimple.Direction.FORWARD);
        mainArm.setDirection(DcMotorSimple.Direction.REVERSE);
        collectorArm.setDirection(DcMotorSimple.Direction.FORWARD);




        collector = hwMap.crservo.get("collector");

       //gyro = hwMap.digitalChannel.get("gyro");
      // touchSensor = hwMap.digitalChannel.get("touch");

       //touchSensor.setMode(DigitalChannel.Mode.INPUT);

        // Set all motors to zero power
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);


    }



}
