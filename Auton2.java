package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Auton For Marker", group = "TESTING")
public class Auton2 extends LinearOpMode implements  ConstantsRover {
    //To Access Hardware
    OmegabotsHardwareRover robot = new OmegabotsHardwareRover();
    //To get how much time we have
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        //Init the hardware
        robot.init(hardwareMap);
        //Not needed
      //  robot.touchSensor.setMode(DigitalChannel.Mode.INPUT);


        //Tell Driver That the robot is resetting encoders
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        //Reset the encoders
        robot.leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.mainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        // Init the encoders
        robot.mainArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);







         telemetry.update();

         waitForStart();
        //encoderArm(0.75, -10, 5.0);
      /*
        robot.mainArm.setPower(-1.0);
        sleep(3000);
        robot.mainArm.setPower(0.0);

        sleep(500);
         //comeDown();
         //robot.latchHolder.setPower(0.75);
         //sleep(1000);
         //robot.latchHolder.setPower(0.0);

        robot.latchHolder.setPower(-1);
        sleep(1000);
        robot.latchHolder.setPower(0);

        sleep(500);

        robot.mainArm.setPower(1.0);
        sleep(2000);
        robot.mainArm.setPower(0.0);
        */

        //Turn the robot Slightly when it lands
        encoderDrive(0.75, -4,4,-4,4, 2.0);
        sleep(1000);
        //GO forward to team depot
         encoderDrive(0.75, -54, -54, -54,-54, 5.0);
         sleep(1000);







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
    //In progress so we don't have to use time
    public void comeDown(){
        /** boolean Onetime = true;
        while(opModeIsActive() && Onetime) {
            if (robot.touchSensor.getState() == false){
                telemetry.addData("Digital Touch", "Is Pressed");
                robot.mainArm.setPower(0);
                robot.latchHolder.setPower(0.75);
                sleep(1800);
                robot.latchHolder.setPower(0);
                telemetry.addData("State", "Came Down");
                Onetime = false;


            } else {
                telemetry.addData("Digital Touch", "Is Not Pressed");
                robot.mainArm.setPower(0.75);
                telemetry.update();

            }
            // send the info back to driver station using telemetry function.
            // if the digital channel returns true it's HIGH and the button is unpressed.




        } */
        encoderArm(1, 7, 5.0);


    }
    //In progress so we dont have to use time
    public void encoderArm (double speed,
                             double Amount,
                             double timeoutS) {
        int newMainTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newMainTarget = robot.mainArm.getCurrentPosition() + (int)(Amount * COUNTS_PER_INCH);

            // Turn On RUN_TO_POSITION
            robot.mainArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robot.mainArm.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.mainArm.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newMainTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.mainArm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.mainArm.setPower(0);


            // Turn off RUN_TO_POSITION
            robot.mainArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        }
    }
}