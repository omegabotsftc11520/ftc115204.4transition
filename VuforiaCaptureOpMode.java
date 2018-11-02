package org.firstinspires.ftc.teamcode.Vuforia;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by Home_College on 10/2/2016.
 * @author Dattathreya MS
 * @since 10/03/2016
 */
@Autonomous(name="Mybot: Vuforia Drive", group="Mybot")
@Disabled
public class VuforiaCaptureOpMode extends LinearOpMode{
    DcMotor leftMotor  = hardwareMap.dcMotor.get("leftMotor");
    DcMotor rightMotor = hardwareMap.dcMotor.get("rightMotor");
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parms = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parms.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parms.vuforiaLicenseKey = "ATyg8YD/////AAAAGW14XygtQUUjj26TSFOCsvVFeA7NgDy0sYn1SXGCV178po2Y3jzLwsVvUIY73ID/wxTOVpG83TU4E5ji2zuXQmcQ87PlAvCnNyVgy/3kwffLI759S3F+ctRRxlTGnShrcrcbug4Nt5oL+399Yq3JsPCs6uzj5s2VmRAHYVQTGlITXQb8ZlSIRaQNWUIptzu7HE2LmKmz2hHrF8+2OPOGjuy4huS31Q9cToI5HuHI481mC5CKj+tDPKh88FtprWouIjvvQtv7oEVF6n9bLSGa8OKO1C4GX0YKU2VvQa9zJPO8QF7yJjfMlAKKOmbQ2l4bn2KqLTj6heYxT6aBCzHcWXLs0VqA2brdJ5XG2w66tzu/";
        parms.cameraMonitorFeedback =  VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parms);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);
        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Lego");
        beacons.get(3).setName("Gears");

        waitForStart();
        beacons.activate();

        while(opModeIsActive()){
            for(VuforiaTrackable beacn: beacons){
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)beacn.getListener()).getPose();
                 if (pose != null){
                     VectorF translate = pose.getTranslation();
                     telemetry.addData(beacn.getName() + "translation", translate);
                     //This is X and Y if it is vertical else use 0, 1;
                     double degTurn = Math.toDegrees(Math.atan2(translate.get(1),translate.get(2)));
                     telemetry.addData(beacn.getName() + "Degrees to turn", degTurn);
                }
            }
            telemetry.update();
        }

    }





}
