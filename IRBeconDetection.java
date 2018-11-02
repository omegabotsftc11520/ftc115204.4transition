package org.firstinspires.ftc.teamcode.Sensors;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * Created by Sachin on 9/28/2016.
 */
@TeleOp
//@Disabled
public class IRBeconDetection extends LinearOpMode {
    ColorSensor colorSensor;
    TouchSensor touchSensor;
    DeviceInterfaceModule deviceInterfaceModule;
    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("color");
        touchSensor = hardwareMap.touchSensor.get("touch");
        deviceInterfaceModule = hardwareMap.deviceInterfaceModule.get("Device Interfaced Module 1");

        boolean touchState = false;
        boolean LEDState = true;

        waitForStart();

        colorSensor.enableLed(LEDState);

        float hsvValues[] = {0, 0, 0};

        while (opModeIsActive()){
            if (!touchState && touchSensor.isPressed()) {
                touchState = true;
                LEDState = !LEDState;
                colorSensor.enableLed(LEDState);
            }
            if (!touchSensor.isPressed()) {
                touchState = false;
            }

            Color.RGBToHSV(colorSensor.red() *8, colorSensor.green() *8, colorSensor.blue() *8, hsvValues);

            telemetry.addData("2 Clear", colorSensor.alpha());
            telemetry.addData("3 Red", colorSensor.red());
            telemetry.addData("4 Green", colorSensor.green());
            telemetry.addData("5 Blue", colorSensor.blue());
            telemetry.addData("6 Hue", hsvValues[0]);
            telemetry.addData("7 Touch", touchSensor.isPressed());

            if (colorSensor.red() > colorSensor.blue() && colorSensor.red() > colorSensor.green()){
                deviceInterfaceModule.setLED(1, true); // turn on the red color on DIM
                deviceInterfaceModule.setLED(0, false); // turn off the blue color on DIM
            }
            else if(colorSensor.blue() > colorSensor.red() && colorSensor.blue() > colorSensor.green()){
                deviceInterfaceModule.setLED(1, false); //Red Off
                deviceInterfaceModule.setLED(0, true);// Blue On
            }
            else{
                deviceInterfaceModule.setLED(1, false);// Red Off
                deviceInterfaceModule.setLED(0, false);
            }

            waitOneFullHardwareCycle();
        }

    }
}
