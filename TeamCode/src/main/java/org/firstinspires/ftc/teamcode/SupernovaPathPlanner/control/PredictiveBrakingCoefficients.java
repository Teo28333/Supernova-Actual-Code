package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control;

public class PredictiveBrakingCoefficients {
    public double kP;
    public double kLinear;
    public double kQuadratic;

    public PredictiveBrakingCoefficients(double kP, double kLinear, double kQuadratic) {
        this.kP = kP;
        this.kLinear = kLinear;
        this.kQuadratic = kQuadratic;
    }
}
