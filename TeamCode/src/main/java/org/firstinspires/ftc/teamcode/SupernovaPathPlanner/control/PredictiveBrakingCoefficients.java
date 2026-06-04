package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control;

/**
 * Documentation ajoutee: Coefficients utilises pour estimer une distance de freinage predictive.
 */
public class PredictiveBrakingCoefficients {
    public double kP;
    public double kLinear;
    public double kQuadratic;

    // Construit cette classe avec les dependances et reglages necessaires.
    public PredictiveBrakingCoefficients(double kP, double kLinear, double kQuadratic) {
        this.kP = kP;
        this.kLinear = kLinear;
        this.kQuadratic = kQuadratic;
    }
}
