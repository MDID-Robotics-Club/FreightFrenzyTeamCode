package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.teamcode.Robot.Parameters;
import org.firstinspires.ftc.teamcode.Robot.Robot;
import org.firstinspires.ftc.teamcode.tests.FSMAutonomousTest;

/**
 * Autonomous Mode Controller Program. Uses the Systems-Based approach that allows a declarative flow to be made.
 * For example, you can write manualCargo(int Position) to set the position of the Cargo Servo to a certain position
 * If you want, you can write raiseLift() or lowerLift()
 *
 * Debugging becomes easy when there are individual functions that are labeled in different systems.
 */

@Autonomous(name="Autonomous FSM Linear", group="Auto")
@Disabled
public class LinearFSMTest extends LinearOpMode {
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
        INTAKE_SPIN
    }

    double Power;
    double intakePower;

    FSMAutonomousTest.RunState runState = FSMAutonomousTest.RunState.RUN_START;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Robot Initialized", "True");
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        robot = new Robot(Parameters.RunMode.AUTONOMOUS, Parameters.DriveMode.MECANUM_DRIVE, hardwareMap);


        runtime.reset();
        waitForStart();

        while (runtime.seconds() < 5.0) {
            switch(runState) {
                case RUN_START:
                    runState = FSMAutonomousTest.RunState.DRIVE_MOVE;
                case DRIVE_MOVE:
                    if (runtime.seconds() < 3.0) {
                        Power = 1.0;

                    }
                    if (runtime.seconds() > 5.0) {
                        Power = 0.0;
                    }
                    runState = FSMAutonomousTest.RunState.INTAKE_SPIN;
                case INTAKE_SPIN:
                    if (runtime.seconds() < 5.0) {
                        intakePower = -1.0;
                    } else {
                    }
                    if (runtime.seconds() > 5.0) {
                        intakePower = 0.0;
                    }
                    runState = FSMAutonomousTest.RunState.DRIVE_MOVE;
                default:
                    runState = FSMAutonomousTest.RunState.RUN_START;
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

        // Sends Telemetry Data to dashboard
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


