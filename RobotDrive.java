package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mdidlib.PIDController;

/**
 * Drivetrain Control System that happens to enable use of PID Control with declarative advantages in the main system
 * Also enables easy Encoder Driving, adjusting with Gyros or even Localization.
 */

public class RobotDrive {
    Robot drivenRobot;

    DcMotor LF;
    DcMotor LB;
    DcMotor RF;
    DcMotor RB;

    int LFState;
    int LBState;
    int RFState;
    int RBState;

    PIDController LFPID = new PIDController(0.0005, 0, 0);
    PIDController LBPID = new PIDController(0.0005, 0, 0);
    PIDController RFPID = new PIDController(0.0005, 0, 0);
    PIDController RBPID = new PIDController(0.0005, 0, 0);

    boolean driveIsBusy;

    double chassisPower = 1.0;

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

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double[] mecanumDrive(double analogX, double analogY, double analogZ) {
        double leftFrontPower, rightFrontPower, leftBackPower, rightBackPower;
        double x = analogY;
        double y = analogX;
        double turn  =  analogZ;

        // Sets the power for each motor and sets a limit using Range.clip()
        leftFrontPower = Range.clip(x - y + turn, -1.0, 1.0);
        leftBackPower = Range.clip(x + y + turn, -1.0, 1.0);
        rightFrontPower = Range.clip(x + y - turn, -1.0, 1.0);
        rightBackPower = Range.clip(x - y - turn, -1.0, 1.0);

        double[] mecanumPower = {leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};

        LF.setPower(leftFrontPower);
        RF.setPower(rightFrontPower);
        LB.setPower(leftBackPower);
        RB.setPower(rightBackPower);
        return mecanumPower;
    }

    public void encoderDrive(double driveSpeed, double verticalTranslation, double horizontalTranslation) {
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double x = verticalTranslation / Math.cos(45);
        double y = horizontalTranslation / Math.cos(45);

        double newLFTarget = 0;
        double newLBTarget = 0;
        double newRFTarget = 0;
        double newRBTarget = 0;

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (verticalTranslation != 0 && horizontalTranslation != 0) {

            newLFTarget = (x) * Parameters.ENCODER_PPR_PER_METER;
            newLBTarget = (x) * Parameters.ENCODER_PPR_PER_METER;
            newRFTarget = (x) * Parameters.ENCODER_PPR_PER_METER;
            newRBTarget = (x) * Parameters.ENCODER_PPR_PER_METER;

            LF.setTargetPosition((int)newLFTarget);
            LB.setTargetPosition((int)newLBTarget);
            RF.setTargetPosition((int)newRFTarget);
            RB.setTargetPosition((int)newRBTarget);

            driveIsBusy = LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy();
            while (driveIsBusy) {
                LF.setPower(LFPID.update(newLFTarget, LF.getCurrentPosition()));
                LB.setPower(LBPID.update(newLBTarget, LB.getCurrentPosition()));
                RF.setPower(RFPID.update(newRFTarget, RF.getCurrentPosition()));
                RB.setPower(RBPID.update(newRBTarget, RB.getCurrentPosition()));
            }
            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);

            newLFTarget = (y) * Parameters.ENCODER_PPR_PER_METER;
            newLBTarget = (-y) * Parameters.ENCODER_PPR_PER_METER;
            newRFTarget = (-y) * Parameters.ENCODER_PPR_PER_METER;
            newRBTarget = (y) * Parameters.ENCODER_PPR_PER_METER;

            LF.setTargetPosition((int)newLFTarget);
            LB.setTargetPosition((int)newLBTarget);
            RF.setTargetPosition((int)newRFTarget);
            RB.setTargetPosition((int)newRBTarget);

            driveIsBusy = LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy();
            while (driveIsBusy) {
                LF.setPower(LFPID.update(newLFTarget, LF.getCurrentPosition()));
                LB.setPower(LBPID.update(newLBTarget, LB.getCurrentPosition()));
                RF.setPower(RFPID.update(newRFTarget, RF.getCurrentPosition()));
                RB.setPower(RBPID.update(newRBTarget, RB.getCurrentPosition()));
            }
            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);

        } else {
            newLFTarget = (x + y) * Parameters.ENCODER_PPR_PER_METER;
            newLBTarget = (x - y) * Parameters.ENCODER_PPR_PER_METER;
            newRFTarget = (x - y) * Parameters.ENCODER_PPR_PER_METER;
            newRBTarget = (x + y) * Parameters.ENCODER_PPR_PER_METER;

            LF.setTargetPosition((int)newLFTarget);
            LB.setTargetPosition((int)newLBTarget);
            RF.setTargetPosition((int)newRFTarget);
            RB.setTargetPosition((int)newRBTarget);

            driveIsBusy = LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy();
            while (driveIsBusy) {
                LF.setPower(LFPID.update(newLFTarget, LF.getCurrentPosition()));
                LB.setPower(LBPID.update(newLBTarget, LB.getCurrentPosition()));
                RF.setPower(RFPID.update(newRFTarget, RF.getCurrentPosition()));
                RB.setPower(RBPID.update(newRBTarget, RB.getCurrentPosition()));
            }
            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);
        }
    }

    public void diagnalDrive(int x, int y, int pulses) {
        boolean forward = Range.clip(x, -1.0, 1.0) > 0;
        boolean right = Range.clip(y, -1.0, 1.0) > 0;

        if (forward) {

        }
    }

    /**
     * Function that changes the orientation of the chassis based on meter count.
     * @param changePosition
     */

    public void encoderTurn(int changePosition) {
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LB.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RB.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double lfChangePosition = 0;
        double lbChangePosition = 0;
        double rfChangePosition = 0;
        double rbChangePosition = 0;

        double avgChassisChange = (lfChangePosition + lbChangePosition + rfChangePosition + rbChangePosition) / 4;

        changePosition *= Parameters.ENCODER_PPR_PER_METER;
        double absoluteChangePosition = Math.abs(changePosition);

        if (changePosition < 0) {
            LF.setTargetPosition(changePosition);
            LB.setTargetPosition(changePosition);
            RF.setTargetPosition(-changePosition);
            RB.setTargetPosition(-changePosition);

            LF.setPower(chassisPower);
            LB.setPower(chassisPower);
            RF.setPower(-chassisPower);
            RB.setPower(-chassisPower);
        } else if (changePosition > 0) {
            LF.setTargetPosition(-changePosition);
            LB.setTargetPosition(-changePosition);
            RF.setTargetPosition(changePosition);
            RB.setTargetPosition(changePosition);

            LF.setPower(-chassisPower);
            LB.setPower(-chassisPower);
            RF.setPower(chassisPower);
            RB.setPower(chassisPower);
        }

        driveIsBusy = LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy();
        while (Math.abs(avgChassisChange) < absoluteChangePosition || driveIsBusy) {
            driveIsBusy = LF.isBusy() || LB.isBusy() || RF.isBusy() || RB.isBusy();
            lfChangePosition = LF.getCurrentPosition();
            lbChangePosition = LB.getCurrentPosition();
            rfChangePosition = RF.getCurrentPosition();
            rbChangePosition = RB.getCurrentPosition();
            avgChassisChange = (lfChangePosition + lbChangePosition + rfChangePosition + rbChangePosition) / 4;
        }

        LF.setPower(0);
        LB.setPower(0);
        RF.setPower(0);
        RB.setPower(0);
    }
}
