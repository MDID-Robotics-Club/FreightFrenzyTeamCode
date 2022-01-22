package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    Servo carouselServo;

    boolean allowIntake;
    private boolean extensionIsBusy = false;

    private int armTargetPosition = 0;
    private double cargoLoaderPosition = 0;
    private int minExtensionPosition = 0;
    private int maxExtensionPosition = 5000;

    private double carouselPosition = 0;
    private int extensionPosition;
    private int swivelPosition = 0;

    int swivelLeftLock = 12500;
    int swivelRightLock = -12500;

    PIDController liftPID = new PIDController(0.0007, 0.0001, 0.0001);
    PIDController extensionPID = new PIDController(0.0004, 0, 0);
    PIDController swivelPID = new PIDController(0.003, 0, 0);

    public MotorControl(Robot robot) {
        drivenRobot = robot;

        swivelMotor = robot.swivel;
        intakeMotor = robot.intake;
        liftMotor = robot.lift;
        extensionMotor = robot.extension;

        cargoMotor = robot.cargo;
        carouselServo = robot.carousel;
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        cargoLoaderPosition = 1.0;
        cargoMotor.setPosition(cargoLoaderPosition);
    }

    public void swivelDrive(double Power) {
        // Method defined in Tested Operator Mode
        allowIntake = false;
    }

    public void intakeDrive() {
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
    }

    public void intakePause() {
        intakeMotor.setPower(0.0);
    }

    // SWIVEL PLATE CONTROL

    public int autoSwivel(Gamepad operatorGamepad) {
        // If the Operator intends to turn the swivel
        if (operatorGamepad.left_stick_x > 0.2 || operatorGamepad.left_stick_x < -0.2) {
            // Conditioning Left or Right Turn

            // Right Condition
            if (operatorGamepad.left_stick_x > 0) {
                // If the current position is more than the locked position, the lock it at the bottom-line position

//                if (Swivel.getCurrentPosition() < rightLock) {
//                    swivelPosition = rightLock;
//                    swivelLock(swivelPosition, Swivel.getCurrentPosition());
//                } else {
//                    // Otherwise give it power as intended
//                    Swivel.setPower(1.0 * swivelPowerScale);
//                }

                swivelMotor.setPower(-0.6);

                swivelPosition = swivelMotor.getCurrentPosition();

                // Left Condition
            } else if (operatorGamepad.left_stick_x < 0) {
                // If the current position is more than the locked position, the lock it at the bottom-line position

//                if (Swivel.getCurrentPosition() > leftLock) {
//                    swivelPosition = leftLock;
//                    swivelLock(swivelPosition, Swivel.getCurrentPosition());
//                } else {
//                    // Otherwise give it power as intended
//                    Swivel.setPower(-1.0 * swivelPowerScale);
//                }

                swivelMotor.setPower(0.6);

                swivelPosition = swivelMotor.getCurrentPosition();
            }

            // Press X Button to reset, controlled using swivelLock Controller
        } else if (operatorGamepad.x) {
            swivelPosition = 0;
            swivelMotor.setPower(swivelPID.update(swivelPosition, swivelMotor.getCurrentPosition()));
        } else {
            swivelMotor.setPower(swivelPID.update(swivelPosition, swivelMotor.getCurrentPosition()));
        }
        return swivelPosition;
    }

    // ARM RAISE/LOWER CONTROL
    public void holdArm() {
        liftMotor.setPower(-liftPID.update(armTargetPosition, liftMotor.getCurrentPosition()));
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
        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
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
        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
        // Optimization Room: adjusting for cos() because of gravity.
    }


    // ARM EXTENSION CONTROL
    public int autoExtension(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            extensionMotor.setPower(0.3);
            extensionPosition = extensionMotor.getCurrentPosition();
        } else if (gamepad.left_trigger > 0) {
            extensionMotor.setPower(-0.3);
            extensionPosition = extensionMotor.getCurrentPosition();
        } else {
            extensionMotor.setPower(extensionPID.update(extensionPosition, extensionMotor.getCurrentPosition()));
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
            extensionMotor.setTargetPosition(Position);
            extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extensionMotor.setPower(0.3);
            while (extensionMotor.getCurrentPosition() < Position || extensionMotor.isBusy()) {

            }
            extensionMotor.setPower(0);
            extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extensionPosition = extensionMotor.getCurrentPosition();
        }
    }

    public void retractArm(int Position) {
        if (Position > extensionPosition) {
            Position = extensionPosition;
        }
        extensionMotor.setTargetPosition(Position);
        extensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extensionMotor.setPower(0.3);
        while (extensionMotor.getCurrentPosition() < Position || extensionMotor.isBusy()) {

        }
        extensionMotor.setPower(0);
        extensionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionPosition = extensionMotor.getCurrentPosition();
    }

    public void resetExtensionArm() {

    }

    // CARGO SERVO RELATED OPERATIONS

    public void autoCargoControl(Gamepad gamepad) {
        extensionIsBusy = extensionMotor.isBusy() || extensionMotor.getPower() > 0.05 || extensionPosition > minExtensionPosition + 10;
        if (!(gamepad.right_bumper && gamepad.left_bumper)) {
            if (extensionIsBusy) {
                if (gamepad.b) {
                    dropCargo();
                } else if (gamepad.a) {
                    defaultCargo();
                }
            } else {
                loadCargo();
            }
        }
    }

    public void dropCargo() {
        // Maybe consider adjusting logic
        cargoLoaderPosition = -1.0;
        cargoMotor.setPosition(cargoLoaderPosition);
    }

    public void defaultCargo() {
        cargoLoaderPosition = 0.9;
        cargoMotor.setPosition(cargoLoaderPosition);
    }

    public void loadCargo() {
        // Could insert color sensor logic in here.
        cargoLoaderPosition = 0.7;
        cargoMotor.setPosition(cargoLoaderPosition);
    }

    public void specifyCargo(double Power) {
        cargoLoaderPosition += Power;
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

    // CARAOUSEL CONTROL

    public void spinCarousel(double power) {
        carouselPosition = carouselServo.getPosition();
        double changeInPosition = Range.clip(power, -3.0, 3.0);
        carouselPosition += changeInPosition;

        carouselServo.setPosition(carouselPosition);
    }
}
