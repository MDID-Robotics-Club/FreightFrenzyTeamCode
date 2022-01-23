package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.Robot.Parameters;
import org.firstinspires.ftc.teamcode.Robot.Robot;

/**
 * Autonomous Mode Controller Program. Uses the Systems-Based approach that allows a declarative flow to be made.
 * For example, you can write manualCargo(int Position) to set the position of the Cargo Servo to a certain position
 * If you want, you can write raiseLift() or lowerLift()
 *
 * Debugging becomes easy when there are individual functions that are labeled in different systems.
 */

@Autonomous(name="Autonomous", group="Auto")
public class FtcAuto extends LinearOpMode {
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

    public FtcAuto(Alliance alliance, Strategy strategy) {
        robot = new Robot(Parameters.RunMode.AUTONOMOUS, Parameters.DriveMode.MECANUM_DRIVE, hardwareMap);
//        strategy = Strategy.ENCODER_DRIVE;
//        switch(strategy) {
//
//        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();


        runtime.reset();
        waitForStart();

        robot.robotDrive.timeDrive(10.0, 0.2);
        robot.robotDrive.timeDiagonalDrive(10.0, 0.1, true, false);
        robot.robotDrive.timeTurn(5.0, 0.3);

        // Sends Telemetry Data to dashboard
        telemetry.addData("Running Mode:", "Autonomous");
        telemetry.addData("Time:", "%.2f", runtime.seconds());

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


