package org.firstinspires.ftc.teamcode.mdidlib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    DcMotor controlledMotor = null;

    double kP;
    double kI;
    double kD;

    double integralSum = 0;
    double maxIntegral;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    public PIDController(double p, double i, double d, DcMotor motor, double maxIntegralSum) {
        controlledMotor = motor;

        kP = p;
        kI = i;
        kD = d;

        maxIntegral = maxIntegralSum;

        timer.reset();
    }

    public double[] getPIDCoefficients() {
        double[] coefficients = {kP, kI, kD};
        return coefficients;
    }

    public double control(double reference, double state) {
        double error = reference - state;

        double P = kP * error;

        double I = kI * (error * timer.time());
        integralSum += I;
        if (integralSum > maxIntegral) {
            integralSum = maxIntegral;
        } else if (integralSum < -maxIntegral) {
            integralSum = -maxIntegral;
        }

        double D = kD * ( (error - lastError) / (timer.time()) );

        double output = integralSum + P + D;

        lastError = error;
        timer.reset();

        return output;
    }

    public double getCurrentIntegral() {
        return integralSum;
    }
}
