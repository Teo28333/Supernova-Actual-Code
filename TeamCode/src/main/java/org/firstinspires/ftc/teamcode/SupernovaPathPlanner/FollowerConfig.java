package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control.PredictiveBrakingCoefficients;

public class FollowerConfig {
    public PIDFCoefficients drivePidf = new PIDFCoefficients(0.045, 0.0, 0.0, 0.0);
    public PIDFCoefficients translationalPidf = new PIDFCoefficients(0.045, 0.0, 0.0, 0.0);
    public PIDFCoefficients turnPidf = new PIDFCoefficients(1.6, 0.0, 0.0, 0.0);
    public PIDFCoefficients secondaryDrivePidf = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
    public PIDFCoefficients secondaryTranslationalPidf = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
    public PIDFCoefficients secondaryTurnPidf = new PIDFCoefficients(0.0, 0.0, 0.0, 0.0);
    public PredictiveBrakingCoefficients predictiveBrakingCoefficients =
            new PredictiveBrakingCoefficients(0.1, 0.04, 0.0016);

    public double translationKp = 0.045;
    public double translationKi = 0.0;
    public double translationKd = 0.0;
    public double translationKf = 0.0;
    public double translationIntegralLimit = 4.0;
    public double headingKp = 1.6;
    public double headingKi = 0.0;
    public double headingKd = 0.0;
    public double headingKf = 0.0;
    public double headingIntegralLimit = 0.5;
    public double maxTranslationPower = 0.85;
    public double maxFinalTranslationPower = 0.35;
    public double maxHeadingPower = 0.75;
    public double lookaheadDistance = 8.0;
    public double finalApproachDistance = 12.0;
    public double positionTolerance = 1.0;
    public double headingTolerance = Math.toRadians(3.0);
    public double segmentAdvanceTolerance = 0.02;
    public boolean centripetalCorrectionEnabled = true;
    public double centripetalCorrectionScale = 0.0;
    public double curvatureSampleStep = 0.01;
    public boolean predictiveBrakingEnabled = false;

    public FollowerConfig withTranslationKp(double translationKp) {
        this.translationKp = translationKp;
        translationalPidf.kP = translationKp;
        return this;
    }

    public FollowerConfig withTranslationPidf(double kP, double kI, double kD, double kF) {
        return withTranslationalPidf(kP, kI, kD, kF);
    }

    public FollowerConfig withDrivePidf(double kP, double kI, double kD, double kF) {
        drivePidf = new PIDFCoefficients(kP, kI, kD, kF);
        return this;
    }

    public FollowerConfig withTranslationalPidf(double kP, double kI, double kD, double kF) {
        translationalPidf = new PIDFCoefficients(kP, kI, kD, kF);
        translationKp = kP;
        translationKi = kI;
        translationKd = kD;
        translationKf = kF;
        return this;
    }

    public FollowerConfig withTurnPidf(double kP, double kI, double kD, double kF) {
        turnPidf = new PIDFCoefficients(kP, kI, kD, kF);
        headingKp = kP;
        headingKi = kI;
        headingKd = kD;
        headingKf = kF;
        return this;
    }

    public FollowerConfig withSecondaryDrivePidf(double kP, double kI, double kD, double kF) {
        secondaryDrivePidf = new PIDFCoefficients(kP, kI, kD, kF);
        return this;
    }

    public FollowerConfig withSecondaryTranslationalPidf(double kP, double kI, double kD, double kF) {
        secondaryTranslationalPidf = new PIDFCoefficients(kP, kI, kD, kF);
        return this;
    }

    public FollowerConfig withSecondaryTurnPidf(double kP, double kI, double kD, double kF) {
        secondaryTurnPidf = new PIDFCoefficients(kP, kI, kD, kF);
        return this;
    }

    public FollowerConfig withPredictiveBraking(boolean enabled) {
        predictiveBrakingEnabled = enabled;
        return this;
    }

    public FollowerConfig withPredictiveBrakingCoefficients(double kP, double kLinear, double kQuadratic) {
        predictiveBrakingCoefficients = new PredictiveBrakingCoefficients(kP, kLinear, kQuadratic);
        return this;
    }

    public FollowerConfig withPredictiveBrakingCoefficients(PredictiveBrakingCoefficients coefficients) {
        predictiveBrakingCoefficients = coefficients;
        return this;
    }

    public FollowerConfig withPathPidf(double kP, double kI, double kD, double kF) {
        return withDrivePidf(kP, kI, kD, kF);
    }

    public FollowerConfig withPrimaryPidf(double kP, double kI, double kD, double kF) {
        return withDrivePidf(kP, kI, kD, kF);
    }

    public FollowerConfig withTranslationIntegralLimit(double translationIntegralLimit) {
        this.translationIntegralLimit = translationIntegralLimit;
        return this;
    }

    public FollowerConfig withHeadingKp(double headingKp) {
        this.headingKp = headingKp;
        turnPidf.kP = headingKp;
        return this;
    }

    public FollowerConfig withHeadingPidf(double kP, double kI, double kD, double kF) {
        return withTurnPidf(kP, kI, kD, kF);
    }

    public FollowerConfig withSecondaryPidf(double kP, double kI, double kD, double kF) {
        return withSecondaryDrivePidf(kP, kI, kD, kF);
    }

    public FollowerConfig withHeadingIntegralLimit(double headingIntegralLimit) {
        this.headingIntegralLimit = headingIntegralLimit;
        return this;
    }

    public FollowerConfig withMaxTranslationPower(double maxTranslationPower) {
        this.maxTranslationPower = maxTranslationPower;
        return this;
    }

    public FollowerConfig withMaxFinalTranslationPower(double maxFinalTranslationPower) {
        this.maxFinalTranslationPower = maxFinalTranslationPower;
        return this;
    }

    public FollowerConfig withMaxHeadingPower(double maxHeadingPower) {
        this.maxHeadingPower = maxHeadingPower;
        return this;
    }

    public FollowerConfig withLookaheadDistance(double lookaheadDistance) {
        this.lookaheadDistance = lookaheadDistance;
        return this;
    }

    public FollowerConfig withFinalApproachDistance(double finalApproachDistance) {
        this.finalApproachDistance = finalApproachDistance;
        return this;
    }

    public FollowerConfig withPositionTolerance(double positionTolerance) {
        this.positionTolerance = positionTolerance;
        return this;
    }

    public FollowerConfig withHeadingTolerance(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        return this;
    }

    public FollowerConfig withSegmentAdvanceTolerance(double segmentAdvanceTolerance) {
        this.segmentAdvanceTolerance = segmentAdvanceTolerance;
        return this;
    }

    public FollowerConfig withCentripetalCorrection(boolean enabled) {
        centripetalCorrectionEnabled = enabled;
        return this;
    }

    public FollowerConfig withCentripetalCorrectionScale(double centripetalCorrectionScale) {
        this.centripetalCorrectionScale = centripetalCorrectionScale;
        return this;
    }

    public FollowerConfig withCurvatureSampleStep(double curvatureSampleStep) {
        this.curvatureSampleStep = Math.abs(curvatureSampleStep);
        return this;
    }

    public PIDFCoefficients getDrivePidf() {
        return drivePidf;
    }

    public PIDFCoefficients getTranslationalPidf() {
        return translationalPidf;
    }

    public PIDFCoefficients getTurnPidf() {
        return turnPidf;
    }
}
