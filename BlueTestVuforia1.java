package org.firstinspires.ftc.teamcode.RelicRecovery;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Sachin on 11/22/2017.
 */
@Autonomous (name = "Blue 1 with Vuforia", group = "Omegabots")
@Disabled

public class BlueTestVuforia1 extends LinearOpMode implements Constants {
    OmegabotsHardware robot = new OmegabotsHardware();
    private ElapsedTime runtime = new ElapsedTime();

    VuforiaLocalizer vuforia;
    boolean Onetime = true;

    @Override
    public void runOpMode() throws InterruptedException {

        //Init the hardware
        robot.init(hardwareMap);
        //Tell Driver That the robot is resetting encoders
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        // Init the encoders
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.gyro.calibrate();
        while (!isStopRequested() && robot.gyro.isCalibrating()) {
            sleep(50);
            idle();
        }
        waitForStart();


            RelicRecoveryVuMark cryptoKey = readPictograph(5.0);
            knockJewelRed();
            if (cryptoKey == RelicRecoveryVuMark.CENTER) {
                gyroDrive(DRIVE_SPEED, 33.6, 0.0);
                gyroTurn(TURN_SPEED, 90.0);
                robot.leftClaw.setPosition(LEFT_OPEN_POSITION);
                robot.rightClaw.setPosition(RIGHT_OPEN_POSITION);
                gyroDrive(DRIVE_SPEED, 2, 0.0);
                gyroDrive(DRIVE_SPEED, -9, 0.0);

            } else if (cryptoKey == RelicRecoveryVuMark.LEFT) {
                gyroDrive(DRIVE_SPEED, 32.5, 0.0);
                gyroTurn(TURN_SPEED, 90.0);
                robot.leftClaw.setPosition(LEFT_OPEN_POSITION);
                robot.rightClaw.setPosition(RIGHT_OPEN_POSITION);
                gyroDrive(DRIVE_SPEED, -9, 0.0);

            } else if (cryptoKey == RelicRecoveryVuMark.RIGHT) {
                gyroDrive(DRIVE_SPEED, 38.5, 0.0);
                gyroTurn(TURN_SPEED, 90.0);
                robot.leftClaw.setPosition(LEFT_OPEN_POSITION);
                robot.rightClaw.setPosition(RIGHT_OPEN_POSITION);
                gyroDrive(DRIVE_SPEED, -9, 0.0)
                ;
            } else if (cryptoKey == RelicRecoveryVuMark.UNKNOWN) {
                robot.leftFrontMotor.setPower(0.0);
                robot.leftFrontMotor.setPower(0.0);
                robot.leftFrontMotor.setPower(0.0);
                robot.leftFrontMotor.setPower(0.0);
            }



    }



    private RelicRecoveryVuMark readPictograph(double seconds) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        RelicRecoveryVuMark vuMark = null;
        parameters.vuforiaLicenseKey = "Absutb//////AAAAGS/mLYTQnEBghpm1oCdkfGqK02MBnD2oxqqIddTosmbu3FmrqYwLUYYtyqh0IblY8o8Aa16RN57Xz0fC4YUcUhJIpIT/C0tH84s2pnJ612Oe1v8YTJNXHz4VAhxqmGJVZXy/yYuamRTEzmaLbrAMkJpPYgIeYE2kdSW9Vq6yEPgK5O/HWjRhR/HWJ7NBCSL/V5nAld7EP6fTKz9XBUu0lLoJGtKVVRjN4iCWGxQ5tCxK6rB7Y85L6LlzTOaer1EX+3DTB2Kk+YcDy0mD0sMJW01BnQLhi9YUjS2HeK4AuP2GD9YAeSWfUrBZAvXSznQydc9Y4cZGwbDSBJ+SvnNVD+to1URVf/ZM5dfi5sZJlNnr";
        boolean vuMarkFound = false;
    /*
     * We also indicate which camera on the RC that we wish to use.
     * Here we chose the back (HiRes) camera (for greater range), but
     * for a competition robot, the front camera might be more convenient.
     */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to scan Pictograph");
        telemetry.update();
        relicTrackables.activate();

        while (opModeIsActive() && !vuMarkFound) {

            vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                vuMarkFound = true;

            /* Found an instance of the template. In the actual game, you will probably
             * loop until this condition occurs, then move on to act accordingly depending
             * on which VuMark was visible. */
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                    telemetry.addData("Pictograph", "%s visible", vuMark);
                    telemetry.update();
                     /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
             * it is perhaps unlikely that you will actually need to act on this pose information, but
             * we illustrate it nevertheless, for completeness. */
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

            /* We further illustrate how to decompose the pose into useful rotational and
             * translational components */
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    } else {
                        telemetry.addData("Pictograph", "not visible");
                    }

                    telemetry.update();
                }
                //wait(3.0);
                //return vuMark;
            }


        }
        //return null;
        return vuMark;
    }
    public void knockJewelRed(){
        boolean LEDState = true;
        //So that the robot doesn't keep looping

        //Turn on Color Sensor LED
        robot.colorSensorJewel.enableLed(LEDState);
        //SO that the vales from red green blue to Hue, Saturation, Value
        float hsvValues[] = {0, 0, 0};

        if (Onetime && opModeIsActive()) {
            //Tell the driver the robot heading from gyro so we know it is 0
            telemetry.addData(">", "Robot Heading = %d", robot.gyro.getHeading());
            telemetry.update();

            robot.gyro.resetZAxisIntegrator();

            //Hold glyph
            robot.leftClaw.setPosition(LEFT_CLOSED_POSITION);
            robot.rightClaw.setPosition(RIGHT_CLOSED_POSITION);
            //lift the glyph
            robot.glyphSlide.setPower(0.5);
            sleep(1000);
            robot.glyphSlide.setPower(0.0);
            telemetry.addData("Holding", "Glyph");






                //LEDState = !LEDState;
                robot.colorSensorJewel.enableLed(LEDState);

                robot.jewelPusher.setPosition(JEWEL_PUSHER_DOWN_POSITION);
                telemetry.addData("Jewel Pusher", "in down position");
                sleep(1000);
                Color.RGBToHSV(robot.colorSensorJewel.red() * 8, robot.colorSensorJewel.green() * 8, robot.colorSensorJewel.blue() * 8, hsvValues);

                telemetry.addData("2 Clear", robot.colorSensorJewel.alpha());
                telemetry.addData("3 Red", robot.colorSensorJewel.red());
                telemetry.addData("4 Green", robot.colorSensorJewel.green());
                telemetry.addData("5 Blue", robot.colorSensorJewel.blue());
                telemetry.addData("6 Hue", hsvValues[0]);//If its red, then we move forward
                if (robot.colorSensorJewel.red() > robot.colorSensorJewel.blue() && robot.colorSensorJewel.red() > robot.colorSensorJewel.green()) {
                    sleep(1000);
                    telemetry.addData("See", "red");

                     robot.deviceInterfaceModule.setLED(1, true); // turn on the red color on DIM
                     robot.deviceInterfaceModule.setLED(0, false); // turn off the blue color on DIM




                    gyroDrive(DRIVE_SPEED, MOVE_UP_FOR_JEWEL, 0.0);
                    sleep(1000);
                    telemetry.addData("Jewel Blue", "is pushed");
                    robot.jewelPusher.setPosition(JEWEL_PUSHER_UP_POSITION);
                    gyroDrive(DRIVE_SPEED, 2, 0.0);
                    telemetry.addData("Back to", "Initial Position");
                    Onetime = false;






//(robot.colorSensorJewel.blue() > robot.colorSensorJewel.red() && robot.colorSensorJewel.blue() > robot.colorSensorJewel.green())
                } else  {
                    sleep(1000);

                    robot.deviceInterfaceModule.setLED(1, false); //Red Off
                     robot.deviceInterfaceModule.setLED(0, true);// Blue On

                    telemetry.addData("See", "blue");
                    gyroDrive(DRIVE_SPEED, BACK_UP_FOR_JEWEL, 0.0);
                    sleep(1000);
                    telemetry.addData("Jewel Red", "is pushed");
                    robot.jewelPusher.setPosition(JEWEL_PUSHER_UP_POSITION);
                    gyroDrive(DRIVE_SPEED, -2, 0.0);
                    sleep(1000);
                    telemetry.addData("Back to", "Initial Position");
                    Onetime = false;




                }

                // If you can't see anything, don't do anything
                /**
                 else {
                 robot.deviceInterfaceModule.setLED(1, false);// Red Off
                 robot.deviceInterfaceModule.setLED(0, false);
                 robot.glyphSlide.setPower(0.0);
                 robot.jewelPusher.setPosition(JEWEL_PUSHER_UP_POSITION);
                 robot.leftFrontMotor.setPower(-0.5);
                 robot.rightFrontMotor.setPower(-0.5);
                 robot.leftBackMotor.setPower(-0.5);
                 robot.rightBackMotor.setPower(-0.5);
                 sleep(2000);
                 gyroTurn(TURN_SPEED, 90);
                 while (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()){
                 idle();
                 }
                 sleep(1000);
                 robot.leftFrontMotor.setPower(-0.5);
                 robot.rightFrontMotor.setPower(-0.5);
                 robot.leftBackMotor.setPower(-0.5);
                 robot.rightBackMotor.setPower(-0.5);
                 sleep(25);
                 robot.leftClaw.setPosition(LEFT_OPEN_POSITION);
                 robot.rightClaw.setPosition(RIGHT_OPEN_POSITION);
                 stop();
                 }
                 */

                Onetime = false;



            Onetime = false;




            Onetime = false;

        }

    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    public void encoderDrive(double speed,
                             double leftInches, double rightInches, double leftBackInches, double rightBackInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        int newLeftBackTarget;
        int newRightBackTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftFrontMotor.setPower(Math.abs(speed));
            robot.rightFrontMotor.setPower(Math.abs(speed));
            robot.leftBackMotor.setPower(Math.abs(speed));
            robot.rightBackMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy()) && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget, newLeftBackTarget, newRightBackTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition(),
                        robot.leftBackMotor.getCurrentPosition(),
                        robot.rightBackMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    public void gyroHold( double speed, double angle, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();

        // keep looping while we have time remaining.
        holdTimer.reset();
        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Update telemetry & Allow time for other processes to run.
            onHeading(speed, angle, P_TURN_COEFF);
            telemetry.update();
        }

        // Stop all motion;
        robot.leftFrontMotor.setPower(0);
        robot.rightFrontMotor.setPower(0);
        robot.leftBackMotor.setPower(0);
        robot.rightBackMotor.setPower(0);
    }
    public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     newLeftBackTarget;
        int     newRightBackTarget;
        int     moveCounts;
        double  max;
        double maxBack;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;
        double  leftBackSpeed;
        double rightBackSpeed;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.leftFrontMotor.getCurrentPosition() + moveCounts;
            newRightTarget = robot.rightFrontMotor.getCurrentPosition() + moveCounts;
            newLeftBackTarget = robot.leftBackMotor.getCurrentPosition() + moveCounts;
            newRightBackTarget = robot.rightBackMotor.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.leftFrontMotor.setTargetPosition(newLeftTarget);
            robot.rightFrontMotor.setTargetPosition(newRightTarget);
            robot.leftBackMotor.setTargetPosition(newLeftBackTarget);
            robot.rightBackMotor.setTargetPosition(newRightBackTarget);

            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.leftFrontMotor.setPower(speed);
            robot.rightFrontMotor.setPower(speed);
            robot.leftBackMotor.setPower(speed);
            robot.rightBackMotor.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                    (robot.leftFrontMotor.isBusy() && robot.rightFrontMotor.isBusy() && robot.leftBackMotor.isBusy() && robot.rightBackMotor.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;
                leftBackSpeed = speed - steer;
                rightBackSpeed =  speed - steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                maxBack = Math.max(Math.abs(leftBackSpeed), Math.abs(rightBackSpeed));
                if (max > 1.0 && maxBack > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                    leftBackSpeed /= maxBack;
                    rightBackSpeed /= maxBack;
                }

                robot.leftFrontMotor.setPower(leftSpeed);
                robot.rightFrontMotor.setPower(rightSpeed);
                robot.leftBackMotor.setPower(rightSpeed);
                robot.rightBackMotor.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      robot.leftFrontMotor.getCurrentPosition(),
                        robot.rightFrontMotor.getCurrentPosition(), robot.leftBackMotor.getCurrentPosition(), robot.rightBackMotor.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed, leftBackSpeed, rightBackSpeed);
                telemetry.update();
            }

            // Stop all motion;
            robot.leftFrontMotor.setPower(0);
            robot.rightFrontMotor.setPower(0);
            robot.leftBackMotor.setPower(0);
            robot.rightBackMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - robot.gyro.getHeading();
        while (robotError > 180)  robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }
    public void gyroTurn (  double speed, double angle) {

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
            // Update telemetry & Allow time for other processes to run.
            telemetry.update();
        }
    }
    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;
        double leftBackSpeed;
        double rightBackSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            leftBackSpeed = 0.0;
            rightBackSpeed = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            leftSpeed   = -rightSpeed;
            leftBackSpeed = -rightSpeed;
            rightBackSpeed = speed * steer;
        }

        // Send desired speeds to motors.
        robot.leftFrontMotor.setPower(leftSpeed);
        robot.rightFrontMotor.setPower(rightSpeed);
        robot.leftBackMotor.setPower(leftBackSpeed);
        robot.rightBackMotor.setPower(rightBackSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
        telemetry.addData("Err/St", "%5.2f/%5.2f", error, steer);
        telemetry.addData("Speed.", "%5.2f:%5.2f", leftSpeed, rightSpeed, leftBackSpeed, rightBackSpeed);

        return onTarget;
    }
}
