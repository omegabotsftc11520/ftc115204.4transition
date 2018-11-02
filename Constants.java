package org.firstinspires.ftc.teamcode.RelicRecovery;

/**
 * Created by Sachin on 8/29/2017.
 */

public interface Constants {
    //For Encoders
    double     COUNTS_PER_MOTOR_REV    = 1126   ;    //Andymark Motor Encoder
    double     DRIVE_GEAR_REDUCTION    = 1.0;     //This is < 1.0 if geared UP
    double     WHEEL_DIAMETER_INCHES   = 4.0 ;     //For figuring circumference
    //How many Ticks per inch
    double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
            (WHEEL_DIAMETER_INCHES * 3.1415);
    //Drive Speed
    double     DRIVE_SPEED             = 0.5;
    //Turn Speed
    double     TURN_SPEED              = 0.5;


    double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    //For Claws
    double LEFT_OPEN_POSITION = 0.0;
    double LEFT_CLOSED_POSITION = 0.8;
    double RIGHT_OPEN_POSITION = 0.8;
    double RIGHT_CLOSED_POSITION = 0.0;
    double MID_SERVO       =  0.5 ;


    //Linear Slide Moving up

    double GLYPH_LINEAR_SLIDE_POWER = 0.25;

    //Motor Stop
    double STOP_MOTOR = 0.0;

    //For Jewels in Autonomous
    double JEWEL_PUSHER_DOWN_POSITION = 1.8;
    double JEWEL_PUSHER_UP_POSITION = 0.0;


   //How many inches we need to go in to the center cryptobox
    double AMT_INCHES_CENTER = -24;
    //How many inches we need to go in to the Left cryptobox
    double AMT_INCHES_LEFT = -28;
    //How many inches we need to go in to the Right cryptobox
    double AMT_INCHES_RIGHT = -20;

    //How much to back up for jewel
    double BACK_UP_FOR_JEWEL = 3.0;
    //How much to move up for jewel
    double MOVE_UP_FOR_JEWEL = -3.0;

    double RELIC_PLACING_POWER = -0.3;

}
