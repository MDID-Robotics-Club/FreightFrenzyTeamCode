package org.firstinspires.ftc.teamcode.tests;

import android.graphics.drawable.GradientDrawable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Robot.Parameters;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.mdidlib.GyroController;

import java.util.Locale;

/**
 * Autonomous Mode Controller Program. Uses the Systems-Based approach that allows a declarative flow to be made.
 * For example, you can write manualCargo(int Position) to set the position of the Cargo Servo to a certain position
 * If you want, you can write raiseLift() or lowerLift()
 *
 * Debugging becomes easy when there are individual functions that are labeled in different systems.
 */

@Autonomous(name="Autonomous Test (FSM) With Gyro", group="Auto")
public class FSMAutonomous_With_Gyro extends OpMode {
    public Robot robot;
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;
    GyroController gyroController;
    Orientation angle;

    FtcDashboard dashboard;

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public enum Strategy {
        MARKER_ON_ONE,
        MARKER_ON_TWO,
        MARKER_ON_THREE,
        DUCK_ON_ONE,
        DUCK_ON_TWO,
        DUCK_ON_THREE,
    }

    public enum RunState {
        RUN_START,
        DRIVE_MOVE,
        DRIVE_STOP,
        INTAKE_SPIN,
        INTAKE_PAUSE
    }

    double Power;
    double intakePower;

    RunState runState = RunState.RUN_START;

    public void init() {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        gyroController = new GyroController(imu);

        robot = new Robot(Parameters.RunMode.AUTONOMOUS, Parameters.DriveMode.MECANUM_DRIVE, hardwareMap);

        telemetry.addData("Status", "Initialized");

    }

    public void start() {
        runtime.reset();
    }

    public void loop() {
        angle = gyroController.getAngle();
        switch(runState) {
            case RUN_START:
                if (angle.firstAngle > 10) {
                    nextState(RunState.DRIVE_STOP);
                    break;
                } else {
                     if (runtime.seconds() < 20.0) {
                        nextState(RunState.DRIVE_MOVE);
                        break;
                    } else {
                        nextState(RunState.DRIVE_STOP);
                        break;
                    }
                }
            case DRIVE_MOVE:
                Power = 1.0;
                nextState(RunState.INTAKE_SPIN);
                robot.robotDrive.mecanumDrive(0, 0, Power, false);
                robot.motorControl.manualLift(0.1);
                break;
            case DRIVE_STOP:
                Power = 0.0;
                nextState(RunState.INTAKE_PAUSE);
                robot.robotDrive.mecanumDrive(0, 0, Power, false);
                break;
            case INTAKE_SPIN:
                intakePower = -1.0;
                nextState(RunState.RUN_START);
                robot.intake.setPower(intakePower);
                break;
            case INTAKE_PAUSE:
                intakePower = 0.0;
                nextState(RunState.RUN_START);
                robot.intake.setPower(intakePower);
                robot.motorControl.manualLift(0.0);
                break;
            default:
                runState = RunState.RUN_START;
                break;
        }

        telemetry.addData("IMU Angle:", formatAngle(angle.angleUnit, angle.firstAngle));
        telemetry.addData("Running Mode:", "Autonomous");
        telemetry.addData("Time:", "%.2f", runtime.seconds());
    }

    public void nextState(RunState runstate) {
        runState = runstate;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public void cmdRedMarkerOnFirst() {

    }

    public void cmdRedMarkerOnSecond() {

    }

    public void cmdRedMarkerOnThird() {

    }

    public void cmdBlueMarkerOnFirst() {

    }

    public void cmdBlueMarkerOnSecond() {

    }

    public void cmdBlueMarkerOnThird() {

    }
}
