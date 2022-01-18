package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *  The code is written using a method called: encoderDrive(speed, xTranslation, yTranslation, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

public class RobotDrive {
    Robot drivenRobot;

    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;

    public RobotDrive (Robot robot) {
        drivenRobot = robot;

        LF = drivenRobot.LF;
        LB = drivenRobot.LB;
        RF = drivenRobot.RF;
        RB = drivenRobot.RB;

        LF.setMode(Parameters.DRIVE_MOTOR_MODE);
        LB.setMode(Parameters.DRIVE_MOTOR_MODE);
        RF.setMode(Parameters.DRIVE_MOTOR_MODE);
        RB.setMode(Parameters.DRIVE_MOTOR_MODE);
    }

    public double[] mecanumDrive(double analogX, double analogY, double analogZ) {
        double leftFrontPower, rightFrontPower, leftRearPower, rightRearPower;
        double x = analogX;
        double y = analogY;
        double turn  =  analogZ;

        // Sets the power for each motor and sets a limit using Range.clip()
        leftFrontPower = Range.clip(y - x - turn, -1.0, 1.0) ;
        rightFrontPower = Range.clip(y + x - turn, -1.0, 1.0) ;
        leftRearPower = Range.clip(y + x - turn, -1.0, 1.0);
        rightRearPower = Range.clip(y - x - turn, -1.0, 1.0);

        double[] mecanumPower = {leftFrontPower, rightFrontPower, leftRearPower, rightRearPower};

        LF.setPower(leftFrontPower);
        RF.setPower(rightFrontPower);
        LB.setPower(leftRearPower);
        RB.setPower(rightRearPower);
        return mecanumPower;
    }

    public void encoderDrive(double driveSpeed,
                             double xTranslationInMeter,
                             double timeoutS) {

        double newLFTarget;
        double newLBTarget;
        double newRFTarget;
        double newRBTarget;

        newLFTarget = xTranslationInMeter * Parameters.ENCODER_PPR_PER_METER;
        newLBTarget = xTranslationInMeter * Parameters.ENCODER_PPR_PER_METER;
        newRFTarget = xTranslationInMeter * Parameters.ENCODER_PPR_PER_METER;
        newRBTarget = xTranslationInMeter * Parameters.ENCODER_PPR_PER_METER;
    }
}
