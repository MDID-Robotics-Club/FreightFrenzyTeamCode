package org.firstinspires.ftc.teamcode.mdidlib;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {

    double kP;
    double kI;
    double kD;

    double integralSum = 0;
    double maxIntegral = 0;

    ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    private double lastError = 0;

    public PIDController(double p, double i, double d) {
        kP = p;
        kI = i;
        kD = d;

        timer.reset();
    }

    public PIDController(double p, double i, double d, double maxIntegralSum) {
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

    public double update(double reference, double state) {
        double error = reference - state;

        double P = kP * error;

        double I = kI * (error * timer.seconds());
        integralSum += I;

        if (maxIntegral > 0) {
            if (integralSum > maxIntegral) {
                integralSum = maxIntegral;
            } else if (integralSum < -maxIntegral) {
                integralSum = -maxIntegral;
            }
        }

        double D = kD * ( (error - lastError) / (timer.seconds()) );

        double output = integralSum + P + D;

        lastError = error;
        timer.reset();

        return output;
    }

    public double getCurrentIntegral() {
        return integralSum;
    }
}
