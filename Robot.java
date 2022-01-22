package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Robot-Class System that simulates the mapping and configuration of the MDID2 Robot based on OOP declarative paradigm
 * To use, initialize with which RunMode, DriveMode, and hardwareMap the robot is in, or is using
 */
public class Robot {
    public static String TEAM_NAME = "Mingdao";

    RobotDrive robotDrive;
    MotorControl motorControl;

    DcMotor LF = null;
    DcMotor LB = null;
    DcMotor RF = null;
    DcMotor RB = null;

    DcMotor swivel = null;
    DcMotor intake = null;
    DcMotor lift = null;
    DcMotor extension = null;
    Servo cargo = null;
    Servo carousel = null;

    boolean cargoLoad = false;

    /**
     * Constructor: Creating the instance of Robot Object
     * @param runMode specified which mode the robot is supposed to be running in
     * @param driveMode specifies which drive the robot is running in
     * @param hardwareMap the hardwareMap from the FTCController Library, used for the robot to interact with motors
     */

    public Robot(Parameters.RunMode runMode, Parameters.DriveMode driveMode, HardwareMap hardwareMap) {
        LF = hardwareMap.tryGet(DcMotor.class, "leftFront");
        LB = hardwareMap.tryGet(DcMotor.class, "leftBack");
        RF = hardwareMap.tryGet(DcMotor.class, "rightFront");
        RB = hardwareMap.tryGet(DcMotor.class, "rightBack");

        // Have not yet initiated reverse MOTOR PARAMETER, so using as a temporary placeholder.
        LF.setDirection(DcMotor.Direction.REVERSE);
        LB.setDirection(DcMotor.Direction.REVERSE);
//        RF.setDirection(DcMotor.Direction.REVERSE);
//        RB.setDirection(DcMotor.Direction.REVERSE);

        swivel = hardwareMap.get(DcMotor.class, "swivelMotor");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        lift = hardwareMap.get(DcMotor.class, "liftMotor");
        extension = hardwareMap.get(DcMotor.class, "extensionMotor");

        carousel = hardwareMap.tryGet(Servo.class, "carouselServo");
        cargo = hardwareMap.get(Servo.class, "cargoServo");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extension.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(Parameters.LIFT_MOTOR_MODE);
        extension.setMode(Parameters.EXTENSION_MOTOR_MODE);
        swivel.setMode(Parameters.SWIVEL_MOTOR_MODE);


//        swivel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        Call the Robot's common initialization protocol and process
        robotDrive = new RobotDrive(this);
        motorControl = new MotorControl(this);
    }

    public void startMode(Parameters.RunMode runMode) {
        String funcMessage = "startMode";
        // Beginning All Operations
    }

    public void stopMode(Parameters.RunMode runMode) {
        String funcMessage = "stopMode";
        // Halting All Operations
    }

    public double getCargoLoaderStatus() {
        if (cargo.getPosition() > 0.7) {
            return 1.0;
        } else if (cargo.getPosition() < -0.7) {
            return -1.0;
        }
        return 0.0;
    }

    /**
     * Setup Vision Programming and Odoemtry Programming Below
     */

    public void initVision(Parameters.RunMode runMode) {
        // Setting Up Vision for Robot

    }

    public boolean getCargoStatus() {
        return cargoLoad;
    }
}