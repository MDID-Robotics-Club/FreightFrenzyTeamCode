package org.firstinspires.ftc.teamcode.mdidlib;

public class FeedForwardController {
    double kP;
    public FeedForwardController(double p) {
        kP = p;
    }

    public double feedforwardUpdate(double reference) {
        double output = reference * kP;
        return output;
    }
}
