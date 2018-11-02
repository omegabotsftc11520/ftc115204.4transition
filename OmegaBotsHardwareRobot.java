package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * This class defines all the specific hardware of the Larson FTC robot.
 *
 * This hardware class has the following device names that have been configured on the robot:

 * Motor channel:  Left  drive motor:        "leftMotor"
 * Motor channel:  Right drive motor:        "rightMotor
 * Motor channel:  Scoop drive motor:        "scooperMotor"
 * Motor Channel:  Left shooting ball motor: "leftShotMotor"
 * Motor Channel:  Right shooting ball motor: "rightShotMotor"
 */
public class OmegaBotsHardwareRobot implements OmegaBotsConstants {
    /* Dc Motors used in this OpMode */
    public DcMotor leftMotor = null;
    public DcMotor rightMotor = null;
    public DcMotor scooperMotor = null;
    public DcMotor leftShotMotor = null;
    public DcMotor rightShotMotor = null;
    public Servo beaconPresserServo =null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public OmegaBotsHardwareRobot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        try {
            leftMotor = hwMap.dcMotor.get(LEFT_MOTOR);
            rightMotor = hwMap.dcMotor.get(RIGHT_MOTOR);

            scooperMotor = hwMap.dcMotor.get(LEFT_SCOP_MOTOR);

            leftShotMotor = hwMap.dcMotor.get(LEFT_SHOT_MOTOR);
            rightShotMotor = hwMap.dcMotor.get(RIGHT_SHOT_MOTOR);
            beaconPresserServo = hwMap.servo.get("beaconPresserServo");
        } catch (IllegalArgumentException e) {

        }
        //setp with encoders, can change it later
        setupMotors(true);
    }

    /***
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long remaining = periodMs - (long) period.milliseconds();

        // sleep for the remaining portion of the regular cycle autoPeriod.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void setupMotors(Boolean useEncoders) {
        if (leftMotor != null) {
            leftMotor.setDirection(DcMotor.Direction.FORWARD);
            leftMotor.setPower(0);
            if (useEncoders) {
                leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (rightMotor != null) {
            rightMotor.setDirection(DcMotor.Direction.REVERSE);
            rightMotor.setPower(0);
            if (useEncoders) {
                rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }

        if (scooperMotor != null) {
            scooperMotor.setDirection(DcMotor.Direction.FORWARD);
            scooperMotor.setPower(0);
        }

        if (leftShotMotor != null) {
            leftShotMotor.setDirection(DcMotor.Direction.FORWARD);
            leftShotMotor.setPower(0);
        }
        if (rightShotMotor != null) {
            rightShotMotor.setDirection(DcMotor.Direction.FORWARD);
            rightShotMotor.setPower(0);
        }
    }

    public void stopMotors(int motorType) {
        switch (motorType) {
            case DRIVE_MOTORS:
                if (leftMotor != null && rightMotor != null) {
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }
                break;
            case SHOOT_MOTORS:
                //if (leftShotMotor != null && rightShotMotor != null) {
                    leftShotMotor.setPower(0);
                    rightShotMotor.setPower(0);
                //}
                break;
            default:
            break;
    }

    }

    public void setMotorPowers(double leftPower, double rightPower, int motorType) {
        // Normalize the values so neither exceed +/- 1.0
        //Double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        //if (maxPower > 1.0) {
            //leftPower /= maxPower;
            //rightPower /= maxPower;
        //}
        switch (motorType) {
            case DRIVE_MOTORS:
                if (leftMotor != null && rightMotor != null) {
                    leftMotor.setPower(Math.abs(leftPower));
                    rightMotor.setPower(Math.abs(rightPower));
                }
                break;
            case SHOOT_MOTORS:
                if (leftShotMotor != null && rightShotMotor != null) {
                    leftShotMotor.setPower(Math.abs(leftPower));
                    rightShotMotor.setPower(Math.abs(rightPower));
                }
                break;
            default:
                break;
        }
    }

    public void setDirection(String motorType, DcMotorSimple.Direction direc){

        DcMotor motor =null;

        switch (motorType){
            case LEFT_MOTOR:
                 motor = leftMotor;
                 break;
            case RIGHT_MOTOR:
                motor = rightMotor;
                break;
            case LEFT_SCOP_MOTOR:
                motor = scooperMotor;
                break;
            case LEFT_SHOT_MOTOR:
               motor = leftShotMotor;
               break;
            case RIGHT_SHOT_MOTOR:
                motor = rightShotMotor;
                break;
            default:
                break;
        }
         if(motor!=null){
             motor.setDirection(direc);
         }
    }

}

