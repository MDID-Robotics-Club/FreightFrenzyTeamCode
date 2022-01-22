package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.dashboard.FtcDashboard;

/**
 * Operator Mode Controller Program.
 */

@TeleOp(name="TeleOp", group="TeleOp")
//@Disabled
public class FtcTeleOp extends LinearOpMode {

    FtcDashboard dashboard;

    Parameters.RunMode runMode = Parameters.RunMode.TELEOP;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    protected Gamepad driverGamepad;
    protected Gamepad operatorGamepad;
//    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Custom defined method in Robot.java which takes in three parameters. This initializes the robot where the interface is the Robot object.
        // Everything to do with operating the robot is controlled through the robot class in this case
        // You just call robot.methodName();

        Robot robot = new Robot(runMode, Parameters.DriveMode.MECANUM_DRIVE, hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Getting Gamepad from Phone
        driverGamepad = gamepad1;
        operatorGamepad = gamepad2;

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

//        dashboard = FtcDashboard.getInstance();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // DRIVE TELEOP
            double[] drivePower = robot.robotDrive.mecanumDrive(driverGamepad.left_stick_x, driverGamepad.left_stick_y, driverGamepad.right_stick_x);

            // INTAKE TELEOP
            robot.motorControl.autoIntake(driverGamepad);

            // EXTENSION TELEOP
            robot.motorControl.autoExtension(operatorGamepad);

            // SWIVEL TELEOP
            robot.motorControl.autoSwivel(operatorGamepad);

            // LIFT TELEOP
            robot.motorControl.autoLift(operatorGamepad);

            // CARGO SERVO TELEOP
            robot.motorControl.autoCargo(operatorGamepad);

            // CAROUSEL SERVO TELEOP


//            robot.motorControl.raiseArm(operatorGamepad.dpad_up);
//            robot.motorControl.lowerArm(operatorGamepad.dpad_down);
//
//            robot.motorControl.extendArm(operatorGamepad.right_trigger);
//            robot.motorControl.retractArm(operatorGamepad.left_trigger);
//
//            if (operatorGamepad.left_bumper || operatorGamepad.right_bumper) {
//                if (robot.getCargoLoaderStatus() == 1.0) {
//                    robot.motorControl.dropCargo();
//                } else if (robot.getCargoLoaderStatus() == -1.0) {
//                    robot.motorControl.loadCargo();
//                } else {
//                    robot.motorControl.manualCargoControl(operatorGamepad);
//                }
//            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mecanum Drive Power Output", "LeftFront: %.2f LeftBack: %.2f RightFront: %.2f RightBack: %.2f",
                    drivePower[0], drivePower[1], drivePower[2], drivePower[3]);

            telemetry.addData("driverGamepad Output", "RJS-X: %.2f RJS-Y: %.2f, LJS-X: %.2f, LJS-Y: %.2f", driverGamepad.right_stick_x, driverGamepad.right_stick_y, driverGamepad.left_stick_x, driverGamepad.left_stick_y);
            telemetry.addData("operatorGamepad Output", "RJS-X: %.2f RJS-Y: %.2f, LJS-X: %.2f, LJS-Y: %.2f", operatorGamepad.right_stick_x, operatorGamepad.right_stick_y, operatorGamepad.left_stick_x, driverGamepad.left_stick_y);

            telemetry.addData("Extension Motor Rotated:", "%f", robot.extension.getPower());
            telemetry.addData("Extension Position:", "%d", robot.extension.getCurrentPosition());

            telemetry.addData("Lift Position:", "%d", robot.lift.getCurrentPosition());
            telemetry.addData("Lift Power", "%f", robot.lift.getPower());

            telemetry.addData("Swivel Position:", "%d", robot.swivel.getCurrentPosition());
            telemetry.addData("Swivel Power:", "%f", robot.swivel.getPower());

            telemetry.addData("Intake Drive:", "%f", robot.intake.getPower());
            telemetry.update();
        }
    }
}
