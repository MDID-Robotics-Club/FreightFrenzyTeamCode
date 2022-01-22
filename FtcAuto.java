package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

    public enum Alliance {
        RED_ALLIANCE,
        BLUE_ALLIANCE
    }

    public enum Strategy {

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

    }
}


