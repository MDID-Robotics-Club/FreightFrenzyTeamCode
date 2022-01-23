package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.Robot.Parameters;
import org.firstinspires.ftc.teamcode.Robot.Robot;

/**
 * Autonomous Mode Controller Program. Uses the Systems-Based approach that allows a declarative flow to be made.
 * For example, you can write manualCargo(int Position) to set the position of the Cargo Servo to a certain position
 * If you want, you can write raiseLift() or lowerLift()
 *
 * Debugging becomes easy when there are individual functions that are labeled in different systems.
 */

@Autonomous(name="Autonomous Test (FSM)", group="Auto")
public class FSMAutonomousTest extends OpMode {
    public Robot robot;
    private ElapsedTime runtime = new ElapsedTime();

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
        robot = new Robot(Parameters.RunMode.AUTONOMOUS, Parameters.DriveMode.MECANUM_DRIVE, hardwareMap);

        telemetry.addData("Status", "Initialized");

    }

    public void start() {
        runtime.reset();
    }

    public void loop() {

        switch(runState) {
            case RUN_START:
                if (runtime.seconds() < 3.0) {
                    Power = 1.0;
                    nextState(RunState.DRIVE_MOVE);
                    break;
                }
                nextState(RunState.DRIVE_STOP);
                break;
            case DRIVE_MOVE:
                Power = 1.0;
                nextState(RunState.INTAKE_SPIN);
                break;
            case DRIVE_STOP:
                Power = 0.0;
                nextState(RunState.INTAKE_PAUSE);
                break;
            case INTAKE_SPIN:
                intakePower = -1.0;
                nextState(RunState.RUN_START);
                break;
            case INTAKE_PAUSE:
                intakePower = 0.0;
                nextState(RunState.RUN_START);
                break;
            default:
                runState = FSMAutonomousTest.RunState.RUN_START;
                break;
        }
        double leftFrontPower = Range.clip(Power, -1.0, 1.0);
        double leftBackPower = Range.clip(Power, -1.0, 1.0);
        double rightFrontPower = Range.clip(Power, -1.0, 1.0);
        double rightBackPower = Range.clip(Power, -1.0, 1.0);

        robot.LF.setPower(leftFrontPower);
        robot.LB.setPower(rightFrontPower);
        robot.RF.setPower(leftBackPower);
        robot.RB.setPower(rightBackPower);

        robot.intake.setPower(intakePower);

        telemetry.addData("Running Mode:", "Autonomous");
        telemetry.addData("Time:", "%.2f", runtime.seconds());
    }

    public void nextState(RunState runstate) {
        runState = runstate;
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


