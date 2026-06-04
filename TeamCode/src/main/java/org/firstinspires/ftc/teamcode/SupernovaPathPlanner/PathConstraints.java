package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

/**
 * Documentation ajoutee: Limites de vitesse/puissance et tolerances applicables a un chemin ou segment.
 */
public class PathConstraints {
    public double maxTranslationPower;
    public double maxFinalTranslationPower;
    public double maxHeadingPower;
    public double positionTolerance;
    public double headingTolerance;

    // Construit cette classe avec les dependances et reglages necessaires.
    public PathConstraints(FollowerConfig config) {
        maxTranslationPower = config.maxTranslationPower;
        maxFinalTranslationPower = config.maxFinalTranslationPower;
        maxHeadingPower = config.maxHeadingPower;
        positionTolerance = config.positionTolerance;
        headingTolerance = config.headingTolerance;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public PathConstraints withMaxTranslationPower(double maxTranslationPower) {
        this.maxTranslationPower = maxTranslationPower;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public PathConstraints withMaxFinalTranslationPower(double maxFinalTranslationPower) {
        this.maxFinalTranslationPower = maxFinalTranslationPower;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public PathConstraints withMaxHeadingPower(double maxHeadingPower) {
        this.maxHeadingPower = maxHeadingPower;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public PathConstraints withPositionTolerance(double positionTolerance) {
        this.positionTolerance = positionTolerance;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public PathConstraints withHeadingTolerance(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        return this;
    }
}
