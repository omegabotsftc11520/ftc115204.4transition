/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.VelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file is for running the Larson FTC Robot in an autonomous mode.
 *
 * The code uses Vuforia and a sensor to accomplish the task.
 */

@Autonomous(name="LarsonFTC: Auto BeaconPress", group="LarsonFTCRobot")
@Disabled
public class REDOmegaBotsBeaconPressAutonomous extends LinearOpMode implements OmegaBotsConstants {

    int leftPosition = 0;
    int rightPosition =0;
    private int  leftEncoderTarget;
    private int  rightEncoderTarget;

    /* Declare OpMode members. */
    OmegaBotsHardwareRobot robot   = new OmegaBotsHardwareRobot();

    public ElapsedTime autoPeriod = new ElapsedTime();
    private ElapsedTime statRunTime = new ElapsedTime();  // Time into current state
    public  OmegaBotsVuforiaHelper vuforia;

    /* Declare state machine */
    public enum RobotStates{
        INITIAL_STATE,
        PUSH_BEACON_STATE,
        STOP_STATE
    }

    private RobotStates currentState;    // Current State Machine State.


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);
        resetDriveMotorEncoders();
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting Position:",  "%7d :%7d",robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();
        try {
            idle();
            // Send the current state info (state and time) back to first line of driver station telemetry.
            changeState(RobotStates.INITIAL_STATE);
            telemetry.addData("Run time:", String.format("%4.1f ", statRunTime.time()));
            telemetry.update();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
        }catch (Exception e){

        }
        while (opModeIsActive()) {
            runAutonomousOperations();
        }
    }


    /*  This state machine executes the current state.  Each STATE's case code waits for an event
     *  to change state. If no state change event found, it continues to perform the same code
     *  segment within that state.
     */
    public void runAutonomousOperations(){

         switch (currentState){
            case INITIAL_STATE:         // Stay in this state until encoders are both Zero.
                 changeState(RobotStates.PUSH_BEACON_STATE);
                 break;
            case PUSH_BEACON_STATE:
                 pushBeacon();
                 changeState(RobotStates.STOP_STATE);
                break;
            case STOP_STATE:
                robot.stopMotors(DRIVE_MOTORS);
                robot.stopMotors(SHOOT_MOTORS);
                break;
        }
    }

    /* Reset both drive motor encoders, and clear current encoder targets.*/
    public void resetDriveMotorEncoders(){
        setEncoderTarget(0, 0,false);
        setMotorModes(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /* Sets Absolute Encoder Position */
    void setEncoderTarget(int leftEncoder, int rightEncoder,boolean addTarget){
        if(addTarget==false) {
            robot.leftMotor.setTargetPosition(leftEncoderTarget = leftEncoder);
            robot.rightMotor.setTargetPosition(rightEncoderTarget = rightEncoder);
        }else{
            robot.leftMotor.setTargetPosition(leftEncoderTarget += leftEncoder);
            robot.rightMotor.setTargetPosition(rightEncoderTarget += rightEncoder);
        }
    }

   /* Set both drive motors to new mode if they need changing. */
    public void setMotorModes(DcMotor.RunMode mode){
        // Ensure the motors are in the correct mode.
        if (robot.leftMotor.getMode() != mode)
            robot.leftMotor.setMode(mode);

        if (robot.rightMotor.getMode() != mode)
            robot.rightMotor.setMode(mode);
    }

    private void changeState(RobotStates newState){
        // Reset the state time, and then change to next state.
        statRunTime.reset();
        currentState = newState;
    }


    public void pushBeacon(){
        telemetry.addData("Current State: ", currentState);
        telemetry.update();
        if (vuforia==null) {
            vuforia = new OmegaBotsVuforiaHelper(telemetry, PHONE_ALIGNMENT);
        }
        //Move straight 60" towards beacon
        //moveRobot(DRIVE_SPEED,BEACON_Y_DIST, BEACON_Y_DIST);
        //take turn

        //Set up vuforia to track beacon navigation image
        if (vuforia.allTrackables == null){
            vuforia.vuforiaSetup();
        }
        float[] gotValues =  vuforia.runVuforia();
        if (gotValues!=null){
            //sleep(1600);
            //For red target the X is negative distance
            try {
                setDriveMotorPowers(-0.25,0.25);
                double x = (double) gotValues[0];
                moveRobot(DRIVE_SPEED, Math.abs(x), Math.abs(x));
                if (robot.beaconPresserServo != null) {
                    robot.beaconPresserServo.setPosition(0.45);
                }
            }catch(Exception e){
                telemetry.addData("error","ERRor: Here1");
                telemetry.update();
            }
        }else{
            try {
                robot.setMotorPowers(DRIVE_SPEED, DRIVE_SPEED, DRIVE_MOTORS);
                robot.setMotorPowers(0, 0, DRIVE_MOTORS);
            }catch (Exception e){
                telemetry.addData("error","ERRor: Here2");
                telemetry.update();
            }


        }
    }



    public void moveRobot(double speed,
                          double leftInches, double rightInches){

        int newLeftTarget;
        int newRightTarget;

        int curLeftPos;
        int curRightPos;

        boolean destinationReached =false;
        // Turn On RUN_TO_POSITION
        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);

        while (!destinationReached) {
            robot.setMotorPowers(speed, speed, DRIVE_MOTORS);
            // Determine new target position, and pass to motor controller
            curLeftPos = robot.leftMotor.getCurrentPosition();
            curRightPos = robot.rightMotor.getCurrentPosition();

            leftEncoderTarget = curLeftPos;
            rightEncoderTarget = curRightPos;

            newLeftTarget = curLeftPos + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = curRightPos + (int)(rightInches * COUNTS_PER_INCH);
            telemetry.addData("Path1 Calculated:",  "%7d :%7d", newLeftTarget,  newRightTarget);
            telemetry.update();
            setEncoderTarget(newLeftTarget,newRightTarget,true);
            destinationReached = ((Math.abs(curLeftPos - newLeftTarget) < 10) &&
                    (Math.abs(curRightPos - newRightTarget) < 10));
            telemetry.addData("Actual Path",  "%7d :%7d",robot.leftMotor.getCurrentPosition(),
                    robot.rightMotor.getCurrentPosition());
            telemetry.update();
        }
        // Stop all motion;
        setDriveMotorPowers(0,0);
        // Turn off RUN_TO_POSITION
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setDriveMotorPowers(double leftPower, double rightPower){
        robot.leftMotor.setPower(Math.abs(leftPower));
        robot.rightMotor.setPower(Math.abs(rightPower));
        telemetry.addData("Current Power",  "Left: " + robot.leftMotor.getPower() + " Right: "
                + robot.rightMotor.getPower());
        telemetry.update();

    }

}
