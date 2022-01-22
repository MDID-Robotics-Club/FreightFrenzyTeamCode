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


    PIDController liftPID = new PIDController(0.03, 0, 0);
    PIDController extensionPID = new PIDController(0.0004, 0, 0);
    PIDController swivelPID = new PIDController(0.003, 0, 0);

    public MotorControl(Robot robot) {
        drivenRobot = robot;

        robot.intake.setDirection(DcMotor.Direction.REVERSE);

        cargoLoaderPosition = 1.0;
        robot.cargo.setPosition(cargoLoaderPosition);
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
//        ]
        drivenRobot.intake.setPower(1.0);
    }

    public void intakePause() { drivenRobot.intake.setPower(0.0); }

    // SWIVEL PLATE CONTROL

    public int autoSwivel(Gamepad operatorGamepad) {
        // If the Operator intends to turn the swivel
        if (operatorGamepad.left_stick_x > 0.2 || operatorGamepad.left_stick_x < -0.2) {
            // Conditioning Left or Right Turn

            // Right Condition
            if (operatorGamepad.left_stick_x > 0) {
                // If the current position is more than the locked position, the lock it at the bottom-line position
               if (drivenRobot.swivel.getCurrentPosition() > minSwivelPosition) {
                    drivenRobot.swivel.setPower(0);
                } else {
                    // Otherwise give it power as intended
                    drivenRobot.swivel.setPower(0.8);
                }
                swivelPosition = drivenRobot.swivel.getCurrentPosition();

                // Left Condition
            } else if (operatorGamepad.left_stick_x < 0) {
                // If the current position is more than the locked position, the lock it at the bottom-line position

                if (drivenRobot.swivel.getCurrentPosition() < minSwivelPosition) {
                    drivenRobot.swivel.setPower(0);
                } else {
                    // Otherwise give it power as intended
                    drivenRobot.swivel.setPower(-0.8);
                }

                swivelPosition = drivenRobot.swivel.getCurrentPosition();
            }

            // Press X Button to reset, controlled using swivelLock Controller
        } else if (operatorGamepad.x) {
            swivelPosition = 0;
            drivenRobot.swivel.setPower(swivelPID.update(swivelPosition, drivenRobot.swivel.getCurrentPosition()));
        } else {
            drivenRobot.swivel.setPower(swivelPID.update(swivelPosition, drivenRobot.swivel.getCurrentPosition()));
        }
        return swivelPosition;
    }

    // ARM RAISE/LOWER CONTROL
    public double autoArm(Gamepad gamepad) {
        if (gamepad.dpad_down){
            liftPosition = drivenRobot.lift.getCurrentPosition();
            if(drivenRobot.lift.getCurrentPosition() < -2500){
                drivenRobot.lift.setPower(0);
            }else{
                drivenRobot.lift.setPower(0.8);
            }
        } else if (gamepad.dpad_up){
            liftPosition = drivenRobot.lift.getCurrentPosition();
            if(drivenRobot.lift.getCurrentPosition() > 2500){
                drivenRobot.lift.setPower(0);
            }else{
                drivenRobot.lift.setPower(-0.8);
            }
        } else {
            drivenRobot.lift.setPower(-(liftPosition - drivenRobot.lift.getCurrentPosition()) * 0.004);
        }
        return drivenRobot.lift.getPower();
    }

    public double holdArm() {
        drivenRobot.lift.setPower(-liftPID.pUpdate(liftPosition, drivenRobot.lift.getCurrentPosition()));
        return drivenRobot.lift.getPower();
    }

//    public void raiseArm(boolean dPad) {
//        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        if (dPad) {
//            armTargetPosition += 3;
//            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            liftMotor.setTargetPosition(armTargetPosition);
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftMotor.setPower(1);
//        }
//        liftMotor.setPower(0);
//        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
//    }

//    public void lowerArm(boolean dPad) {
//        if (dPad) {
//            armTargetPosition -= 3;
//            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            liftMotor.setTargetPosition(armTargetPosition);
//            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            liftMotor.setPower(-1);
//        }
//        liftMotor.setPower(0);
//        // Adjusts the motor to the target position, which eliminates potential errors such as gravity.
//        // Optimization Room: adjusting for cos() because of gravity.
//    }


    // ARM EXTENSION CONTROL
    public int autoExtension(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            drivenRobot.extension.setPower(0.3);
            extensionPosition = drivenRobot.extension.getCurrentPosition();
        } else if (gamepad.left_trigger > 0) {
            drivenRobot.extension.setPower(-0.3);
            extensionPosition = drivenRobot.extension.getCurrentPosition();
        } else {
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

            }
            drivenRobot.extension.setPower(0);
            drivenRobot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            extensionPosition = drivenRobot.extension.getCurrentPosition();
        }
    }

    public void retractArm(int Position) {
        if (Position > extensionPosition) {
            Position = extensionPosition;
        }
        drivenRobot.extension.setTargetPosition(Position);
        drivenRobot.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drivenRobot.extension.setPower(0.3);
        while (drivenRobot.extension.getCurrentPosition() < Position || drivenRobot.extension.isBusy()) {

        }
        drivenRobot.extension.setPower(0);
        drivenRobot.extension.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extensionPosition = drivenRobot.extension.getCurrentPosition();
    }

    public void resetExtensionArm() {

    }

    // CARGO SERVO RELATED OPERATIONS

    public void autoCargoControl(Gamepad gamepad) {
        extensionIsBusy = drivenRobot.extension.isBusy() || drivenRobot.extension.getPower() > 0.05 || extensionPosition > minExtensionPosition + 10;
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

    public void specifyCargo(double Power) {
        cargoLoaderPosition += Power;
        drivenRobot.cargo.setPosition(cargoLoaderPosition);
    }

    public void manualCargoControl(Gamepad gamepad) {
        if (gamepad.a) {
            cargoLoaderPosition -= 0.2;
        }
        if (gamepad.b) {
            cargoLoaderPosition += 0.2;
        }
        drivenRobot.cargo.setPosition(Range.clip(cargoLoaderPosition, -1.0, 1.0));
    }

    // CARAOUSEL CONTROL

    public void spinCarousel(double power) {
        carouselPosition = drivenRobot.carousel.getPosition();
        double changeInPosition = Range.clip(power, -3.0, 3.0);
        carouselPosition += changeInPosition;

        drivenRobot.carousel.setPosition(carouselPosition);
    }
}
