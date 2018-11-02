package org.firstinspires.ftc.teamcode.VelocityVortex;

/**
 * This interface creates all the constants used by the robotics tean.
 * so that if it is changed, can be changed in one place.
 *
 * @since 09/26/2016
 * @version 0.1
 *
 */
public interface OmegaBotsConstants {

     double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
     double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
     double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
     double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)/
                                                    (WHEEL_DIAMETER_INCHES * 3.1415);
     double     DRIVE_SPEED             = 0.3;
     double     SHOOT_SPEED             = 1.0;
     double     TO_SHOT_PART_DIST       = 18;  //34" position to shoot the particles.
     double     CAP_BALL_DIST           = 31;  //48" distance to the ball.
     double     CENTER_VORTEX_DIST      = 13;

     //Beacon stuff
    double BEACON_Y_DIST = 15; // 60" to the first white line
    double BEACON_X_DIST = 24; //24 inches before the white line
    double COLOR_INTENSITY = 0.5;
    double TURN_SPEED      = 0.25; //when turning one motor -0.25 and the other 0.25


     int DRIVE_MOTORS = 0;
     int SHOOT_MOTORS = 1;


     /* Vuforia licence key */
     String VUFORIA_KEY = "AXNqfgr/////AAAAGUTRQociw0HCtMNs15pZkxI" +
            "kZCQQSgmWpRcQegCOl3liNiw1SpVAK4hWHhmyuO" +
            "/O5mjfbfp7KTLw8eN2d/hUl00KSWHPv0GVMvWZmvsa1rOf79ujMS" +
            "C3e6x90Kfuw1DUFj22BAWFx3cpggfC0yHeSG7NOrJHxLAmY7" +
            "PH/JardB6eS58+mWGMA+M8en895K80nhDMbSsNYQgi0/aqMWCL++" +
            "9dnNPtZdfvY2g5LJMqWJbO6I6e2YznBMlDMXCH3qj" +
            "NIIlTnYA2D20XTHdxYILaNbkIUKnhRJXALbG3Pdst7PnKSLVq0kJ3C/" +
            "DN//57oFIT9r79bcX1fRZbbnNI0chznab15WONz2EX3qTipq0IEa10";

    String VUFORIA_TAG = "Vuforia";
    String VUFORIA_ASSET = "FTC_2016-17";

    float CLSR_IMG_LOC_WIDTH = 762; //30" * 25.4 = 762 mm
    float FRTHR_IMG_LOC_WIDTH = 1372; //54" * 25.4

    /* How long each state should run max */
    double INITIAL_RUN_TIME = 5;
    double SHOOT_PARTICLE_RUN_TIME = 2.5;
    double PUSH_CAPBALL_RUNTIME = 5;
    double PUSH_BEACON_RUNTIME = 9;
    double CENTER_VORTEX_RUNTIME = 6;

    /* determining which mode to run */
    /* How long each state should run max */
    boolean INITIAL_RUN = true;
    boolean SHOOT_PARTICLE_RUN = true;
    boolean PUSH_CAPBALL_RUN = true;
    boolean PUSH_BEACON_RUN = false;
    boolean CENTER_VORTEX_RUN = true;


    /* Vuforia image names */
    String WHEELS = "BLUE: Wheels";
    String LEGOS = "BLUE: Legos";
    String TOOLS = "RED: Tools";
    String GEARS = "RED: Gears";

    /* Camera */
    String CAMERA_DIRECTION_BACK = "BACK"; //Else FRONT
    String CAMERA_DIRECTION_FRONT = "FRONT"; //Else FRONT
    int PHONE_ALIGN_HORIZONTAL = 0;
    int PHONE_ALIGN_VERTICAL =1;

    /* Set this prior to running */
    String CAMERA_DIRECTION = CAMERA_DIRECTION_BACK;
    int PHONE_ALIGNMENT = PHONE_ALIGN_HORIZONTAL;

    /** Motor names **/
    String LEFT_MOTOR = "leftMotor";
    String RIGHT_MOTOR = "rightMotor";
    String LEFT_SCOP_MOTOR = "scooperMotor";
    String LEFT_SHOT_MOTOR = "leftShotMotor";
    String RIGHT_SHOT_MOTOR = "rightShotMotor";

}
