package org.firstinspires.ftc.teamcode.Sensors;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Sachin on 8/6/2017.
 */
@Autonomous (name = "Controlling A Servo")
public class Controling_A_Servo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo;
        servo = hardwareMap.servo.get("servo");
        double servoPosition = 127;

        GyroSensor sensorGyro;
        ModernRoboticsI2cGyro mrGyro;
        int zAccumulated;
        sensorGyro = hardwareMap.gyroSensor.get("gyro");
        mrGyro = (ModernRoboticsI2cGyro) sensorGyro;

        servo.setPosition(servoPosition / 255);
        telemetry.addData("Servo", (servoPosition));
        telemetry.addData("Servo Position", (servoPosition));

        sleep(1000);

        mrGyro.calibrate();

        waitForStart();

        while (mrGyro.isCalibrating()) {

        }
        while (opModeIsActive()) {

            zAccumulated = mrGyro.getIntegratedZValue();

            servoPosition = (127 + ((zAccumulated * 128) / 90));

            servo.setPosition(Range.clip(servoPosition / 255, 0, 1));


            telemetry.addData("1. accu", String.format("%03d", zAccumulated));
            telemetry.addData("2. Servo", (servoPosition));
            telemetry.addData("3. Servo Position", servoPosition);


            waitOneFullHardwareCycle();

        }
    }
}
