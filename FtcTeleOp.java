package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Operator Mode Controller Program.
 */

@TeleOp(name="TeleOp", group="TeleOp")
//@Disabled
public class FtcTeleOp extends LinearOpMode {

    Parameters.RunMode runMode = Parameters.RunMode.TELEOP;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotor leftFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightFront = null;
    public DcMotor rightRear = null;

    public DcMotor Lift1;
    public DcMotor Swivel;

    protected Gamepad driverGamepad;
    protected Gamepad operatorGamepad;

    private double drivePowerScale = 1.0;
    private double swivelPowerScale = 0.8;
    private double armPowerScale = 1.0;

    public int swivelPosition = 0;

    static final double SWIVEL_P = 0.00006;

    double chassisPower;

    int extensionPower;
//    FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Custom defined method in Robot.java which takes in three parameters. This initializes the robot where the interface is the Robot object.
        // Everything to do with operating the robot is controlled through the robot class in this case
        // You just call robot.methodName();

        Robot robot = new Robot(Parameters.RunMode.TELEOP, Parameters.DriveMode.MECANUM_DRIVE, hardwareMap);

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Getting Motors
        Swivel = hardwareMap.get(DcMotor.class, "swivelMotor");

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
            if (driverGamepad.right_trigger > 0) {
                robot.motorControl.intakeDrive();
            }

            testSwivelDrive();
            extensionPower = robot.motorControl.teleOpExtensionArm(operatorGamepad);

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
            telemetry.addData("Extension Arm Position:", extensionPower);
            telemetry.update();
        }
    }

    public void testSwivelDrive() {
        int leftLock = 12500;
        int rightLock = -12500;

        // If the Operator intends to turn the swivel
        if (operatorGamepad.left_stick_x > 0.2 && operatorGamepad.left_stick_x < -0.2) {
            // Conditioning Left or Right Turn

                // Right Condition
            if (operatorGamepad.left_stick_x > 0) {
                // If the current position is more than the locked position, the lock it at the bottom-line position
                if (Swivel.getCurrentPosition() < rightLock) {
                    swivelPosition = rightLock;
                    swivelLock(swivelPosition, Swivel.getCurrentPosition());
                } else {
                    // Otherwise give it power as intended
                    Swivel.setPower(1.0 * swivelPowerScale);
                }
                swivelPosition = Swivel.getCurrentPosition();

                // Left Condition
            } else if (operatorGamepad.left_stick_x < 0) {
                // If the current position is more than the locked position, the lock it at the bottom-line position
                if (Swivel.getCurrentPosition() > leftLock) {
                    swivelPosition = leftLock;
                    swivelLock(swivelPosition, Swivel.getCurrentPosition());
                } else {
                    // Otherwise give it power as intended
                    Swivel.setPower(-1.0 * swivelPowerScale);
                }
                swivelPosition = Swivel.getCurrentPosition();
            }

            // Press X Button to reset, controlled using swivelLock Controller
        } else if (operatorGamepad.x) {
            swivelPosition = 0;
            swivelLock(swivelPosition, Swivel.getCurrentPosition());
        }
        else {
            swivelLock(swivelPosition, Swivel.getCurrentPosition());
        }

    }

    public void swivelLock(double targetPosition, double currentPosition) {
        int error = (int)(targetPosition - currentPosition);
        double P = error * SWIVEL_P;
        double I = 0;
        double D = 0;
        double feedForward = P + I + D;
        Swivel.setPower(-feedForward);
    }
}
