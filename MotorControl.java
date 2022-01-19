package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.mdidlib.PIDController;

public class MotorControl {
    Robot drivenRobot;

    DcMotor swivelMotor;
    DcMotor intakeMotor;
    DcMotor liftMotor;
    DcMotor extensionMotor;

    Servo cargoMotor;
    Servo carouselMotor;

    boolean allowIntake;

    private int armTargetPosition = 0;
    private double cargoLoaderPosition = 0;

    private double carouselPosition = 0;

    PIDController liftPID;

    public MotorControl(Robot robot) {
        drivenRobot = robot;

        swivelMotor = robot.swivel;
        intakeMotor = robot.intake;
        liftMotor = robot.lift;
        cargoMotor = robot.cargo;
        extensionMotor = robot.extension;
        carouselMotor = robot.carousel;

        swivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        cargoLoaderPosition = -1.0;
        cargoMotor.setPosition(cargoLoaderPosition);

        PIDController liftPID = new PIDController(0004, 0.0003, 0, liftMotor, 1000);
    }

    public void swivelDrive(double Power) {
        // Method defined in Tested Operator Mode
        allowIntake = false;
    }

    public void intakeDrive() {
        int swivelPosition = swivelMotor.getCurrentPosition();
        allowIntake = true;
        if (swivelPosition < 30 && swivelPosition > -30) {
            allowIntake = true;
        } else {
            allowIntake = false;
        }

        if (allowIntake) {
            // May want to control with PID Controller....
            intakeMotor.setPower(1.0);
        } else {
            intakeMotor.setPower(0);
        }
    }

    public void raiseArm(boolean dPad) {
        if (dPad) {
            armTargetPosition += 3;
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setTargetPosition(armTargetPosition);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
        liftMotor.setPower(liftPID.update(armTargetPosition, liftMotor.getCurrentPosition()));
    }

    public void lowerArm(boolean dPad) {
        if (dPad) {
            armTargetPosition -= 3;
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            liftMotor.setTargetPosition(armTargetPosition);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(-1);
        }
        liftMotor.setPower(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
        liftMotor.setPower(liftPID.update(armTargetPosition, liftMotor.getCurrentPosition()));
    }

    public void autoArm() {

    }

    public void extendArm(double trigger) {


    }

    public void retractArm(double trigger) {

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
        carouselPosition = carouselMotor.getPosition();
        double changeInPosition = Range.clip(power, -3.0, 3.0);
        carouselPosition += changeInPosition;

        carouselMotor.setPosition(carouselPosition);
    }
}
