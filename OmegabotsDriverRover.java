package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "ROVER TEST", group = "OMEGABOTS")

public class OmegabotsDriverRover extends OpMode implements ConstantsRover {
    OmegabotsHardwareRover robot   = new OmegabotsHardwareRover();

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {

        //Driving and Strafing
        float LFSpeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float LBSpeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        float RFSpeed = gamepad1.right_stick_y + gamepad1.left_stick_x;
        float RBSpeed = gamepad1.right_stick_y - gamepad1.left_stick_x;

        LFSpeed = Range.clip(LFSpeed, -1, 1);
        LBSpeed = Range.clip(LBSpeed, -1, 1);
        RFSpeed = Range.clip(RFSpeed, -1, 1);
        RBSpeed = Range.clip(RBSpeed, -1, 1);

        //For the Main Arm going up and down
        float MainUp = gamepad2.right_stick_y - gamepad2.right_stick_x;
        //for the collector arm going up and down
        float CollectorArm = gamepad2.left_stick_y - gamepad2.left_stick_x;

        //Not needed
        //float CollectorArmRight = gamepad2.right_trigger;
        //float CollectorArmLeft = gamepad2.left_trigger;

        //Not needed
        //CollectorArmRight = Range.clip(CollectorArmRight, 0, 1);
        //CollectorArmLeft = Range.clip(CollectorArmLeft, -1, 0);





        CollectorArm = Range.clip(CollectorArm, -1, 1);

        MainUp = Range.clip(MainUp, -1, 1);
        //MainDown = Range.clip(MainDown, -1, 1);

        //Set the speed of the motor
        robot.mainArm.setPower(MainUp);
        robot.collectorArm.setPower(CollectorArm);

        //robot.collectorArm.setPower(CollectorArmLeft);
        //robot.collectorArm.setPower(CollectorArmRight);


        //robot.collectorArm.setPower(gamepad2.right_trigger);
        //Set the speed of the motor
        robot.rightFrontMotor.setPower(RFSpeed);
        robot.rightBackMotor.setPower(RBSpeed);
        robot.leftFrontMotor.setPower(LFSpeed);
        robot.leftBackMotor.setPower(LBSpeed);

        //if(gamepad1.dpad_up){
       //     robot.mainArm.setPower(MAIN_ARM_SPEED);
        //} else if (gamepad1.dpad_down){
            //robot.mainArm.setPower(-MAIN_ARM_SPEED);
        //} else if (gamepad1.dpad_right){
            //robot.mainArm.setPower(STOP_MOTOR);
        //}

        //To open and close the latch

        if (gamepad2.a){
           robot.latchHolder.setPower(LATCH_SPEED);
        } else if (gamepad2.b) {
            robot.latchHolder.setPower(-LATCH_SPEED);
        } else if (gamepad2.x){
          robot.latchHolder.setPower(STOP_MOTOR);
        }

        //Speed of Linear SLide
        if (gamepad2.dpad_up){
            robot.sampleSlide.setPower(SLIDE_SPEED);
        } else if (gamepad2.dpad_down){
            robot.sampleSlide.setPower(-SLIDE_SPEED);
        } else if (gamepad2.dpad_right){
            robot.sampleSlide.setPower(STOP_MOTOR);
        }

        //Not needed

       // if (gamepad2.a){
         //   robot.collectorArm.setPower(0.25);
       // } else if (gamepad2.b){
           // robot.collectorArm.setPower(-0.9);
         //if (gamepad2.y){
            //robot.collectorArm.setPower(-0.25);

        // }
             // else if (gamepad2.y){
            //robot.collectorArm.setPower(0.7);
       // }
        //For collecting the minerals and gold
        if (gamepad1.left_bumper){
            robot.collector.setPower(1);
        } else if (gamepad1.right_bumper){
            robot.collector.setPower(-1);
        } else if (gamepad1.y){
            //change as needed
            robot.collector.setPower(0.0);
        }



    }
}
