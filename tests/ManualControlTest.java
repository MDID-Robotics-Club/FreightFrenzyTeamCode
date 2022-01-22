package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Manual Control", group="Linear Opmode")
public class ManualControlTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor LF = null;
    public DcMotor RF = null;
    public DcMotor LB = null;
    public DcMotor RB = null;
    public DcMotor intake = null;
    public DcMotor arm_angle = null;
    public DcMotor arm_extend = null;
    public DcMotor turn = null;
    public Servo box = null;
    public CRServo duck = null;

    double LF_P = 0;
    double RF_P = 0;
    double RB_P = 0;
    double LB_P = 0;
    double box_position = 0;
    double box_default_position = 0;
    double chassis_power = 0;
    double arm_angle_position = 0;
    int turnPosition = 0;
    double arm_angle_P = 0.004;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LF = hardwareMap.get(DcMotor.class, "leftFront");
        RF = hardwareMap.get(DcMotor.class, "rightFront");
        LB = hardwareMap.get(DcMotor.class, "leftBack");
        RB = hardwareMap.get(DcMotor.class, "rightBack");
        duck = hardwareMap.get(CRServo.class, "carouselServo");
        intake = hardwareMap.get(DcMotor.class, "intakeMotor");
        arm_angle = hardwareMap.get(DcMotor.class, "liftMotor");
        arm_extend= hardwareMap.get(DcMotor.class, "extensionMotor");
        turn = hardwareMap.get(DcMotor.class, "swivelMotor");
        box = hardwareMap.get(Servo.class, "cargoServo");

        LF.setDirection(DcMotor.Direction.FORWARD);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        RB.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        arm_extend.setDirection(DcMotor.Direction.FORWARD);
        arm_angle.setDirection(DcMotor.Direction.FORWARD);
        turn.setDirection(DcMotor.Direction.FORWARD);
        box.setDirection(Servo.Direction.FORWARD);
        duck.setDirection(CRServo.Direction.FORWARD);

        arm_angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm_extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_extend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_angle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turn.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm_extend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        box.setPosition(box_default_position);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            drive();
            intake();
            arm_angle();
            arm_extend();
            box();
            duck();
            turn();
            telemetry();
        }
    }

    public void drive(){
        if(gamepad1.right_bumper || gamepad1.left_bumper){
            chassis_power = 0.5;
        }else{
            chassis_power = 1.0;
        }

        LF_P = Range.clip(gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -chassis_power, chassis_power);
        RF_P = Range.clip(-gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x, -chassis_power, chassis_power);
        RB_P = Range.clip(-gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -chassis_power, chassis_power);
        LB_P = Range.clip(gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x, -chassis_power, chassis_power);

        LF.setPower(LF_P);
        RF.setPower(RF_P);
        RB.setPower(RB_P);
        LB.setPower(LB_P);
    }

    public void intake(){
        if(gamepad1.left_trigger > 0){
            intake.setPower(1);
        }else if(gamepad1.right_trigger > 0){
            intake.setPower(-1);
        }else{
            intake.setPower(0);
        }
    }

    public void arm_angle(){
        if(gamepad2.dpad_down){
            arm_angle_position = arm_angle.getCurrentPosition();
            if(arm_angle.getCurrentPosition() < -2500){
                arm_angle.setPower(0);
            }else{
                arm_angle.setPower(0.8);
            }
        }else if(gamepad2.dpad_up){
            arm_angle_position = arm_angle.getCurrentPosition();
            if(arm_angle.getCurrentPosition() > 2500){
                arm_angle.setPower(0);
            }else{
                arm_angle.setPower(-0.8);
            }
        }else{
            arm_angle.setPower(-(arm_angle_position - arm_angle.getCurrentPosition()) * arm_angle_P);
        }
    }

    public void arm_extend(){
        if(gamepad2.y){
            if(arm_extend.getCurrentPosition() > 2000){
                arm_extend.setPower(0);
            }else{
                arm_extend.setPower(1);
            }
        }else if(gamepad2.a){
            if(arm_extend.getCurrentPosition() < -2000){
                arm_extend.setPower(0);
            }else{
                arm_extend.setPower(-1);
            }
        }else{
            arm_extend.setPower(0);
        }
    }

    public void box(){
        if(gamepad2.right_bumper){
            box_position += 0.007;
            box_position =Range.clip(box_position, -1, 1);
            box.setPosition(box_position);
        }else if(gamepad2.left_bumper){
            box_position -= 0.007;
            box_position =Range.clip(box_position, -1, 1);
            box.setPosition(box_position);
        }else if(gamepad2.x){
            box_position = box_default_position;
            box.setPosition(box_position);
        }
    }

    public void duck(){
        duck.setPower(gamepad2.left_stick_x);
    }

    public void turn(){
        if(gamepad2.right_trigger > 0){
            if(arm_angle.getCurrentPosition() < -25000){
                turn.setPower(0);
            }else{
                turn.setPower(-0.65);
            }
        }else if(gamepad2.left_trigger > 0){
            if(arm_angle.getCurrentPosition() > 25000){
                turn.setPower(0);
            }else{
                turn.setPower(0.65);
            }
        }else if(gamepad2.x){
            if(turn.getCurrentPosition() < 0){
                turn.setPower(-0.35);
            }else if(turn.getCurrentPosition() > 0){
                turn.setPower(-0.35);
            }
        }else{
            turn.setPower(0);
        }
    }

    public void telemetry(){
        telemetry.addData("LF_P", LF.getPower());
        telemetry.addData("LB_P", LB.getPower());
        telemetry.addData("RF_P", RF.getPower());
        telemetry.addData("RB_P", RB.getPower());
        telemetry.addData("arm extend position", arm_extend.getCurrentPosition());
        telemetry.addData("arm angle position", arm_angle.getCurrentPosition());
        telemetry.addData("turn position", turn.getCurrentPosition());
        telemetry.addData("box position", box.getPosition());
        telemetry.addData("arm extend power", arm_extend.getPower());
        telemetry.addData("arm angle power", arm_angle.getPower());
        telemetry.addData("turn power", turn.getPower());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}