package org.firstinspires.ftc.teamcode.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.Servo;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.MagneticFlux;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Sachin on 11/23/2016.
 */
/**
@TeleOp(name = "Vuforia Test", group = "OmegaBota")
public class VuforiaOpTest extends LinearOpMode {
    DcMotor rightMotor;
    DcMotor leftMotor;
    double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * 3.1415);
    @Override
    public void runOpMode() throws InterruptedException {
        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");


        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();



        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        params.vuforiaLicenseKey = "ATyg8YD/////AAAAGW14XygtQUUjj26TSFOCsvVFeA7NgDy0sYn1SXGCV178po2Y3jzLwsVvUIY73ID/wxTOVpG83TU4E5ji2zuXQmcQ87PlAvCnNyVgy/3kwffLI759S3F+ctRRxlTGnShrcrcbug4Nt5oL+399Yq3JsPCs6uzj5s2VmRAHYVQTGlITXQb8ZlSIRaQNWUIptzu7HE2LmKmz2hHrF8+2OPOGjuy4huS31Q9cToI5HuHI481mC5CKj+tDPKh88FtprWouIjvvQtv7oEVF6n9bLSGa8OKO1C4GX0YKU2VvQa9zJPO8QF7yJjfMlAKKOmbQ2l4bn2KqLTj6heYxT6aBCzHcWXLs0VqA2brdJ5XG2w66tzu/";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");
        VuforiaTrackableDefaultListener wheels = (VuforiaTrackableDefaultListener) beacons.get(0).getListener();
        beacons.activate();

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftMotor.setPower(0.2);
        rightMotor.setPower(0.2);

        while (opModeIsActive() && wheels.getRawPose() == null){
            idle();
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        //Analyze Beacon

        VectorF angles = anglesFromTarget(wheels);

        VectorF trans = navOffWall(wheels.getPose().getTranslation(), Math.toDegrees(angles.get(0)) - 90, new VectorF(500,0,0));

         do {
            if (wheels.getPose() != null){
                trans = navOffWall((wheels.getPose().getTranslation()), Math.toDegrees(angles.get(0)) - 90, new VectorF(500,0,0));
                if (trans.get(0)> 0){
                    leftMotor.setPower(0.2);
                    rightMotor.setPower(-0.2);
                } else {
                    leftMotor.setPower(-0.2);
                    rightMotor.setPower(0.2);
                }
            }
            idle();

        } while (opModeIsActive() && Math.abs(trans.get(0)) > 30);

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Change 150
        leftMotor.setTargetPosition((int) (leftMotor.getCurrentPosition()+ ((Math.hypot(trans.get(0), trans.get(2) + COUNTS_PER_INCH)))));
        rightMotor.setTargetPosition((int) (rightMotor.getCurrentPosition()+ ((Math.hypot(trans.get(0), trans.get(2) + COUNTS_PER_INCH)))));

        leftMotor.setPower(0.3);
        rightMotor.setPower(0.3);

        while (opModeIsActive() && leftMotor.isBusy() && rightMotor.isBusy()){
            idle();
        }

        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive() && (wheels.getPose() == null || Math.abs(wheels.getPose().getTranslation().get(0)) > 10)){
            if (wheels !=null){
                if (wheels.getPose().getTranslation().get(0) > 0){
                    leftMotor.setPower(-0.3);
                    rightMotor.setPower(0.3);
                }else {
                    leftMotor.setPower(0.3);
                    rightMotor.setPower(-0.3);
                }
            }else {
                leftMotor.setPower(-0.3);
                rightMotor.setPower(0.3);
            }

        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPower(0.3);
        rightMotor.setPower(0.3);
        leftMotor.setTargetPosition((int) (2 * COUNTS_PER_INCH));
        rightMotor.setTargetPosition((int)(2 * COUNTS_PER_INCH));





    }
    public VectorF navOffWall(VectorF trans, double robotAngle, VectorF offWall){
        return new VectorF((float) (trans.get(0) - offWall.get(0) * Math.sin(Math.toRadians(robotAngle)) - offWall.get(2) * Math.cos(Math.toRadians(robotAngle))), trans.get(1), (float) (trans.get(2) + offWall.get(0) * Math.cos(Math.toRadians(robotAngle)) - offWall.get(2) * Math.sin(Math.toRadians(robotAngle))));
    }

    public VectorF anglesFromTarget(VuforiaTrackableDefaultListener image){
        float [] data = image.getRawPose().getData();
        float [] [] rotation = {{data[0], data[1]}, {data[4], data[5], data[6]}, {data[8], data[9], data[10]}};
        double thetaX = Math.atan2(rotation[2][1], rotation[2][2]);
        double thetaY = Math.atan2(-rotation[2][0], Math.sqrt(rotation[2][1] * rotation[2][1] + rotation[2][2] * rotation[2][2]));
        double thetaZ = Math.atan2(rotation[1][0], rotation[0][0]);
        return new VectorF((float)thetaX, (float)thetaY, (float)thetaZ);

    }
}
*/