package org.firstinspires.ftc.teamcode.RelicRecovery;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.util.Range;


/**
 * Created by Omegabots on 2/9/2017.
 */
@TeleOp(name = "DriverControlled", group = "OmegaBotsFTC")
@Disabled
public class MecunumDrive extends OpMode implements Constants {
    OmegabotsHardware robot   = new OmegabotsHardware();

    double          clawOffset      = 0;                       // Servo mid position
    final double    CLAW_SPEED      = 0.02 ;
    final double    CLAW_DOWN_SPEED = 0.0009;// sets rate to move servo


    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        robot.jewelPusher.setPosition(0.0);
       //Driving and Strafing
        float LFSpeed = gamepad1.left_stick_y - gamepad1.left_stick_x;
        float LBSpeed = gamepad1.left_stick_y + gamepad1.left_stick_x;
        float RFSpeed = gamepad1.right_stick_y + gamepad1.left_stick_x;
        float RBSpeed = gamepad1.right_stick_y - gamepad1.left_stick_x;

        LFSpeed = Range.clip(LFSpeed, -1, 1);
        LBSpeed = Range.clip(LBSpeed, -1, 1);
        RFSpeed = Range.clip(RFSpeed, -1, 1);
        RBSpeed = Range.clip(RBSpeed, -1, 1);


        robot.rightFrontMotor.setPower(RFSpeed);
        robot.rightBackMotor.setPower(RBSpeed);
        robot.leftFrontMotor.setPower(LFSpeed);
        robot.leftBackMotor.setPower(LBSpeed);
        //Holding the glyph
        //Not used
        if(gamepad2.left_bumper) {
            robot.leftClaw.setPosition(LEFT_OPEN_POSITION);
            robot.rightClaw.setPosition(RIGHT_OPEN_POSITION);
        }
        if (gamepad2.right_bumper) {
            robot.leftClaw.setPosition(LEFT_CLOSED_POSITION);
            robot.rightClaw.setPosition(RIGHT_CLOSED_POSITION);
        }

        // Use gamepad left & right Bumpers to open and close the claw
       /*
        if (gamepad2.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad2.left_bumper)
            clawOffset -= CLAW_SPEED;
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.leftClaw.setPosition(robot.MID_SERVO + clawOffset);
        robot.rightClaw.setPosition(robot.MID_SERVO - clawOffset);
        */
        //Moving up or down depending on where the glyph has to go
        if (gamepad1.dpad_up){
            robot.glyphSlide.setPower(GLYPH_LINEAR_SLIDE_POWER);
        }
        if (gamepad1.dpad_down){
            robot.glyphSlide.setPower(-GLYPH_LINEAR_SLIDE_POWER);
        }
        if (gamepad1.dpad_right){
            robot.glyphSlide.setPower(STOP_MOTOR);
        }
        if (gamepad1.dpad_left){
            robot.glyphSlide.setPower(STOP_MOTOR);
        }


        //End Game Relic Placing
        /*
         *Change Servo towards speed if work!!!!!!!!
         */

        float RSpeed = gamepad2.left_stick_y - gamepad2.left_stick_x;
        RSpeed = Range.clip(RSpeed, -1, 1);
        robot.relicPlacer.setPower(RSpeed);


        /*if (gamepad2.b){
           //Change this value
            robot.holdRelicClaw.setPosition(0.5);
        }
        */


         if (gamepad2.dpad_right){
            // Change this value
            robot.holdRelicClaw.setPosition(0.95);
        }
        if (gamepad2.dpad_left){
            robot.holdRelicClaw.setPosition(0.05);
        }

        if (gamepad2.y)
            clawOffset += CLAW_DOWN_SPEED;
        else if (gamepad2.b)
            clawOffset -= CLAW_DOWN_SPEED;
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.wallRelicClaw.setPosition(MID_SERVO + clawOffset);



    }
}
