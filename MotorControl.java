package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mdidlib.PIDController;

/**
 * Motor Control System that controls any control of motors that is not controlled by the Drivetrain.
 * For 2022 Freight Frenzy, this includes
 * DC MOTORS
 * 1. Extension Motor (for the Arm)
 * 2. Lift Motor (for the Arm)
 * 3. Swivel Motor, used for "swiveling" the entire intake and deploy system around.
 * 4. Intake Motor, used for sweeping in the cargo elements.
 *
 * SERVO MOTORS
 * 1. Cargo Motor, which allows the loading and deployment of cargo elements.
 * 2. Carousel Motor, which spins the carousel spinner to drop a Team Marker or Ducks
 */
public class MotorControl {
    Robot drivenRobot;

    boolean allowIntake;

    private boolean extensionIsBusy = false;
    private boolean liftIsBusy = false;
    private boolean intakeIsBusy = false;
    private boolean swivelIsBusy = false;

    private int liftPosition = 0;
    private int maxLiftPosition = 0;
    private int minLiftPosition = 0;

    private int extensionPosition = 0;
    private int minExtensionPosition = 0;
    private int maxExtensionPosition = 5000;

    private int swivelPosition = 0;
    private int minSwivelPosition = -12500;
    private int maxSwivelPosition = 12500;

    private double carouselPosition = 0;
    private double cargoLoaderPosition = 0;


    PIDController liftPID = new PIDController(0.004, 0, 0.00005);
    PIDController extensionPID = new PIDController(0.0004, 0, 0);
    PIDController swivelPID = new PIDController(0.0004, 0, 0.00005);

    // INTIALIZATION

    public MotorControl(Robot robot) {
        drivenRobot = robot;

        robot.intake.setDirection(DcMotor.Direction.REVERSE);

        cargoLoaderPosition = 1.0;
        robot.cargo.setPosition(cargoLoaderPosition);
    }


    // INTAKE CONTROL
    public void autoIntake(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            intakeDrive();
            intakeIsBusy = true;
        } else if (gamepad.left_trigger > 0) {
            intakeReverse();
            intakeIsBusy = true;
        } else {
            intakePause();
            intakeIsBusy = false;
        }
    }

    public void intakeDrive() {
        drivenRobot.intake.setPower(1.0);
    }

    public void intakeDriveForTime(double timeoutS) {
        ElapsedTime intakeRuntime = new ElapsedTime();
        while (intakeRuntime.seconds() < timeoutS) {
            intakeDrive();
            intakeIsBusy = true;
        }
        intakePause();
        intakeRuntime.reset();
    }

    public void intakeReverse() {
        drivenRobot.intake.setPower(-1.0);
    }

    public void intakePause() { drivenRobot.intake.setPower(0.0); }

    // SWIVEL PLATE CONTROL

    public int autoSwivel(Gamepad operatorGamepad) {
        // If the Operator intends to turn the swivel
        if (operatorGamepad.left_stick_x > 0) {
            swivelIsBusy = true;
            // If the current position is more than the locked position, the lock it at the bottom-line position
            if (drivenRobot.swivel.getCurrentPosition() > maxSwivelPosition) {
                drivenRobot.swivel.setPower(0);
            } else {
                // Otherwise give it power as intended
                drivenRobot.swivel.setPower(1.0);
            }
            swivelPosition = drivenRobot.swivel.getCurrentPosition();
            drivenRobot.lift.setPower(-liftPID.update(liftPosition, drivenRobot.lift.getCurrentPosition()));
            // Left Condition
        } else if (operatorGamepad.left_stick_x < 0) {
            swivelIsBusy = true;
            // If the current position is more than the locked position, the lock it at the bottom-line position
            if (drivenRobot.swivel.getCurrentPosition() < minSwivelPosition) {
                drivenRobot.swivel.setPower(0);
            } else {
                // Otherwise give it power as intended
                drivenRobot.swivel.setPower(-1.0);
            }

            swivelPosition = drivenRobot.swivel.getCurrentPosition();
            drivenRobot.lift.setPower(-liftPID.update(liftPosition, drivenRobot.lift.getCurrentPosition()));
        }
            // Press X Button to reset, controlled using PID Controller Controller
        else if (operatorGamepad.x) {
            swivelIsBusy = true;
            swivelPosition = 0;
            drivenRobot.swivel.setPower(swivelPID.update(swivelPosition, drivenRobot.swivel.getCurrentPosition()));
        } else {
            swivelIsBusy = false;
            drivenRobot.swivel.setPower(swivelPID.update(swivelPosition, drivenRobot.swivel.getCurrentPosition()));
        }
        return swivelPosition;
    }

    public void manualSwivel(int position) {

    }

    // ARM RAISE/LOWER CONTROL
    public double autoLift(Gamepad gamepad) {
        if (gamepad.dpad_down){
            liftIsBusy = true;
            liftPosition = drivenRobot.lift.getCurrentPosition();
            if(drivenRobot.lift.getCurrentPosition() < -2500){
                drivenRobot.lift.setPower(0);
            }else{
                drivenRobot.lift.setPower(0.8);
            }
        } else if (gamepad.dpad_up){
            liftIsBusy = true;
            liftPosition = drivenRobot.lift.getCurrentPosition();
            if(drivenRobot.lift.getCurrentPosition() > 2500){
                drivenRobot.lift.setPower(0);
            }else{
                drivenRobot.lift.setPower(-0.8);
            }
        } else {
            liftIsBusy = false;
            drivenRobot.lift.setPower(-liftPID.update(liftPosition, drivenRobot.lift.getCurrentPosition()));
        }
        return drivenRobot.lift.getPower();
    }

    public void raiseLift(int Position) {
        drivenRobot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPosition += Position;
        drivenRobot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivenRobot.lift.setTargetPosition(Position);
        drivenRobot.lift.setPower(1.0);
        while (drivenRobot.lift.isBusy()) {
            liftIsBusy = true;
        }
        drivenRobot.lift.setPower(0);
        liftIsBusy = false;
        drivenRobot.lift.setMode(Parameters.LIFT_MOTOR_MODE);
    }

    public void lowerLift(int Position) {
        drivenRobot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftPosition -= Position;
        drivenRobot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivenRobot.lift.setTargetPosition(Position);
        drivenRobot.lift.setPower(-1.0);
        while (drivenRobot.lift.isBusy()) {
            liftIsBusy = true;
        }
        drivenRobot.lift.setPower(0);
        liftIsBusy = false;
        drivenRobot.lift.setMode(Parameters.LIFT_MOTOR_MODE);

    }

    public double holdLift() {
        drivenRobot.lift.setPower(-liftPID.pUpdate(liftPosition, drivenRobot.lift.getCurrentPosition()));
        liftIsBusy = false;
        return drivenRobot.lift.getPower();
    }

    // ARM EXTENSION CONTROL

    public int autoExtension(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            extensionIsBusy = true;
            drivenRobot.extension.setPower(1.0);
            extensionPosition = drivenRobot.extension.getCurrentPosition();
        } else if (gamepad.left_trigger > 0) {
            extensionIsBusy = true;
            drivenRobot.extension.setPower(-1.0);
            extensionPosition = drivenRobot.extension.getCurrentPosition();
        } else {
            extensionIsBusy = false;
            drivenRobot.extension.setPower(extensionPID.update(extensionPosition, drivenRobot.extension.getCurrentPosition()));
        }
//        extensionMotor.setPower(extensionPID.update(extensionPosition, extensionMotor.getCurrentPosition()));
        return extensionPosition;
    }

    /**
     * Extends the Arm to
     * @param Position
     */

    public void extendArm(int Position) {
        if (Position < extensionPosition) {
            Position = extensionPosition;
        } else {
            drivenRobot.extension.setTargetPosition(Position);
            drivenRobot.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drivenRobot.extension.setPower(0.3);
            while (drivenRobot.extension.getCurrentPosition() < Position || drivenRobot.extension.isBusy()) {
                extensionIsBusy = true;
            }
            extensionIsBusy = false;
            drivenRobot.extension.setPower(0);
            drivenRobot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extensionPosition = drivenRobot.extension.getCurrentPosition();
        }
    }

    public void retractArm(int Position) {
        drivenRobot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (Position > extensionPosition) {
            Position = extensionPosition;
        }
        drivenRobot.extension.setTargetPosition(Position);
        drivenRobot.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivenRobot.extension.setPower(0.3);
        while (drivenRobot.extension.getCurrentPosition() < Position || drivenRobot.extension.isBusy()) {
            extensionIsBusy = true;
        }
        extensionIsBusy = false;
        drivenRobot.extension.setPower(0);
        drivenRobot.extension.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        extensionPosition = drivenRobot.extension.getCurrentPosition();
    }

    public void resetExtensionArm() {
        retractArm(0);
    }

    // CARGO SERVO RELATED OPERATIONS

    public void autoCargo(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            cargoLoaderPosition += 0.1;
        } else if (gamepad.left_bumper) {
            cargoLoaderPosition -= 0.1;
        }
        drivenRobot.cargo.setPosition(Range.clip(cargoLoaderPosition, -1.0, 1.0));
    }

    public void dropCargo() {
        // Maybe consider adjusting logic
        cargoLoaderPosition = -1.0;
        drivenRobot.cargo.setPosition(cargoLoaderPosition);
    }

    public void defaultCargo() {
        cargoLoaderPosition = 0.9;
        drivenRobot.cargo.setPosition(cargoLoaderPosition);
    }

    public void loadCargo() {
        // Could insert color sensor logic in here.
        cargoLoaderPosition = 0.7;
        drivenRobot.cargo.setPosition(cargoLoaderPosition);
    }

    public void manualCargo(double Power) {
        cargoLoaderPosition += Power;
        drivenRobot.cargo.setPosition(cargoLoaderPosition);
    }


    // CARAOUSEL CONTROL

    public void spinCarousel(double power) {
        carouselPosition = drivenRobot.carousel.getPosition();
        double changeInPosition = Range.clip(power, -3.0, 3.0);
        carouselPosition += changeInPosition;

        drivenRobot.carousel.setPosition(carouselPosition);
    }

    // TEAM MARK CONTROL
    public void teamMarkerLoad() {
        cargoLoaderPosition = -1.0;
        drivenRobot.cargo.setPosition(cargoLoaderPosition);
    }

    public void teamMarkerUnload() {
        cargoLoaderPosition = 1.0;
        drivenRobot.cargo.setPosition(cargoLoaderPosition);
    }
}