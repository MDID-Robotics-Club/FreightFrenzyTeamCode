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

    public DcMotor Swivel;

    protected Gamepad driverGamepad;
    protected Gamepad operatorGamepad;

    private double drivePowerScale = 1.0;
    private double swivelPowerScale = 0.8;
    private double armPowerScale = 1.0;

    double extensionPosition = 0;

    public int swivelPosition = 0;

    DcMotor Extension;
    DcMotor Intake;

    static final double SWIVEL_P = 0.00006;

    double chassisPower;

    private int extensionPower;
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

        // Getting Motors
        Intake = hardwareMap.get(DcMotor.class, "intakeMotor");

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
            double[] drivePower = robot.robotDrive.mecanumDrive(driverGamepad.left_stick_x, driverGamepad.left_stick_y, driverGamepad.right_stick_x);

            if (driverGamepad.right_trigger > 0 || driverGamepad.left_trigger > 0) {
                robot.motorControl.intakeDrive();
            } else {
                robot.motorControl.intakePause();
            }

            extensionPower = robot.motorControl.autoExtension(operatorGamepad);
            robot.motorControl.autoSwivel(operatorGamepad);
            robot.motorControl.holdArm();

            robot.motorControl.cargoMotor.setPosition(0.4);

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
            telemetry.addData("operatorGamepad Output", "RJS-X: %.2f RJS-Y: %.2f, LJS-X: %.2f", operatorGamepad.right_stick_x, operatorGamepad.right_stick_y, operatorGamepad.left_stick_x);
            telemetry.addData("Extension Motor Rotated:", "%d", extensionPower);
            telemetry.addData("Extension Position:", "%d", robot.extension.getCurrentPosition());
            telemetry.addData("Swivel Position:", "%d", swivelPosition);
            telemetry.addData("Swivel Power:", "%f", robot.swivel.getPower());
            telemetry.addData("Intake Drive:", "%f", robot.intake.getPower());
            telemetry.update();
        }
    }
}
