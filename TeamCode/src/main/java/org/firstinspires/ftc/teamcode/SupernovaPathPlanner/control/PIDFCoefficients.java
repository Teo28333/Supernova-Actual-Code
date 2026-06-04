package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control;

/**
 * Documentation ajoutee: Simple conteneur des coefficients PIDF.
 */
public class PIDFCoefficients {
    public double kP;
    public double kI;
    public double kD;
    public double kF;

    // Construit cette classe avec les dependances et reglages necessaires.
    public PIDFCoefficients(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
}
