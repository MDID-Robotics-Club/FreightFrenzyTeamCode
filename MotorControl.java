package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mdidlib.PIDController;

public class MotorControl {
    Robot drivenRobot;

    DcMotorEx swivelMotor;
    DcMotorEx intakeMotor;
    DcMotorEx liftMotor;
    DcMotorEx extensionMotor;

    Servo cargoMotor;
    Servo carouselServo;

    boolean allowIntake;

    private int armTargetPosition = 0;
    private double cargoLoaderPosition = 0;
    private int minExtensionPosition = 0;
    private int maxExtensionPosition = 10000;

    private double carouselPosition = 0;
    private int extensionPosition = 0;

    PIDController liftPID = new PIDController(0.007, 0.0003, 0);
    PIDController extensionPID = new PIDController(0.0004, 0, 0);

    public MotorControl(Robot robot) {
        drivenRobot = robot;

        swivelMotor = robot.swivel;
        intakeMotor = robot.intake;
        liftMotor = robot.lift;
        extensionMotor = robot.extension;

        cargoMotor = robot.cargo;
        carouselServo = robot.carousel;
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        swivelMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        cargoLoaderPosition = -1.0;
        cargoMotor.setPosition(cargoLoaderPosition);

    }

    public void swivelDrive(double Power) {
        // Method defined in Tested Operator Mode
        allowIntake = false;
    }

    public boolean intakeDrive() {
//        int swivelPosition = swivelMotor.getCurrentPosition();
//        allowIntake = true;
//        if (swivelPosition < 30 || swivelPosition > -30) {
//            allowIntake = true;
//        } else {
//            allowIntake = false;
//        }
//
//        if (allowIntake) {
//            // May want to control with PID Controller....
//            intakeMotor.setPower(1.0);
//        } else {
//            intakeMotor.setPower(0);
//        }
        intakeMotor.setPower(1.0);
        return true;
    }

    public boolean intakePause() {
        intakeMotor.setPower(0.0);
        return false;
    }

    public void raiseArm(boolean dPad) {
        if (dPad) {
            armTargetPosition += 3;
            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor.setTargetPosition(armTargetPosition);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
        liftMotor.setPower(liftPID.update(armTargetPosition, liftMotor.getCurrentPosition()));
    }

    public void lowerArm(boolean dPad) {
        if (dPad) {
            armTargetPosition -= 3;
            liftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            liftMotor.setTargetPosition(armTargetPosition);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-1);
        }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
        // Optimization Room: adjusting for cos() because of gravity.
        liftMotor.setPower(liftPID.update(armTargetPosition, liftMotor.getCurrentPosition()));
    }

    public int teleOpExtensionArm(Gamepad gamepad) {
        extensionPosition = extensionMotor.getCurrentPosition();
        if (gamepad.right_trigger > 0) {
            extensionMotor.setPower(0.8);
            extensionPosition = extensionMotor.getCurrentPosition();
        } else if (gamepad.left_trigger > 0) {
            extensionMotor.setPower(-0.8);
            extensionPosition = extensionMotor.getCurrentPosition();
        } else {
            extensionMotor.setPower(extensionPID.update(extensionPosition, extensionMotor.getCurrentPosition()));
        }
//        extensionMotor.setPower(extensionPID.update(extensionPosition, extensionMotor.getCurrentPosition()));
        return extensionMotor.getCurrentPosition();
    }

    /**
     * Extends the Arm to
     * @param Position
     */

    public void extendArm(int Position) {
        if (Position < extensionPosition) {
            Position = extensionPosition;
        } else {
            extensionMotor.setTargetPosition(Position);
            extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            extensionMotor.setPower(0.3);
            while (extensionMotor.getCurrentPosition() < Position || extensionMotor.isBusy()) {

            }
            extensionMotor.setPower(0);
            extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            extensionPosition = extensionMotor.getCurrentPosition();
        }
    }

    public void retractArm(int Position) {
        if (Position > extensionPosition) {
            Position = extensionPosition;
        }
        extensionMotor.setTargetPosition(Position);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(0.3);
        while (extensionMotor.getCurrentPosition() < Position || extensionMotor.isBusy()) {

        }
        extensionMotor.setPower(0);
        extensionMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extensionPosition = extensionMotor.getCurrentPosition();
    }

    public void resetExtensionArm() {

    }

    public void dropCargo() {
        // Maybe consider adjusting logic
        cargoLoaderPosition = -1.0;
        cargoMotor.setPosition(cargoLoaderPosition);
    }

    public void loadCargo() {
        // Could insert color sensor logic in here.
        cargoLoaderPosition = 1.0;
        cargoMotor.setPosition(cargoLoaderPosition);
    }

    public void manualCargoControl(Gamepad gamepad) {
        if (gamepad.a) {
            cargoLoaderPosition -= 0.2;
        }
        if (gamepad.b) {
            cargoLoaderPosition += 0.2;
        }
        cargoMotor.setPosition(Range.clip(cargoLoaderPosition, -1.0, 1.0));
    }

    public void spinCarousel(double power) {
        carouselPosition = carouselServo.getPosition();
        double changeInPosition = Range.clip(power, -3.0, 3.0);
        carouselPosition += changeInPosition;

        carouselServo.setPosition(carouselPosition);
    }
}
