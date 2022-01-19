package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Parameters {
    public static class Preferences {

    }

//    public static class Parameters {
//
//    }

    public enum DriveMode {
        MECANUM_DRIVE,
        NO_DRIVE
    }

    public enum RunMode {
        INVALID,
        TELEOP,
        AUTONOMOUS,
        DISABLED,
        TEST
    }

    /**
     * Run Mode
     */
    static final DcMotor.RunMode DRIVE_MOTOR_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    static final DcMotor.RunMode SWIVEL_MOTOR_MODE = DcMotor.RunMode.RUN_USING_ENCODER;
    static final DcMotor.RunMode INTAKE_MOTOR_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;

    /**
     * GOBILDA 5203-0019 Motor Specifications
     */
    static final double GOBILDA_5203_PPR = ((((1+(46/17))) * (1+(46/11))) * 28);
    static final double GOBILDA_5203_PPR_ROUNDED = 537.7;
    static final double GOBILDA_5203_RPM = 312;
//    static final double GOLBILD_5203_PPS;

    /**
     * Wheel Specifications and Settings
     */
    static final double MECANUM_WHEEL_DIAMETER_IN_METERS = 0.096;
    static final double WHEEL_CIRCUMFERENCE_IN_METERS = MECANUM_WHEEL_DIAMETER_IN_METERS * Math.PI;
    static final double ENCODER_PPR_PER_METER = GOBILDA_5203_PPR / WHEEL_CIRCUMFERENCE_IN_METERS;


    /**
     * Strategic Positional Constants
     */
    static final double LEFT_SWIVEL_PLATE_POSITION = 0;
    static final double RIGHT_SWIVEL_PLATE_POSITION = 0;

    /**
     * PID CONTROLLER CONSTANTS
     */
    static final double SWIVEL_PID_P = 0.00004;
    static final double SWIVEL_PID_I = 0.00004;
    static final double SWIVEL_PID_D = 0.00004;

    static final double LIFT_PID_P = 0.00004;
    static final double LIFT_PID_I = 0.00003;
    static final double LIFT_PID_D = 0.00004;



    /**
     * Encoder Drive Error Constants
     */
    static final double MECANUM_VERTICAL_MULTIPLIER = 1.05;
    static final double MECANUM_HORIZONTAL_MULTIPLIER = 1.10;
}
