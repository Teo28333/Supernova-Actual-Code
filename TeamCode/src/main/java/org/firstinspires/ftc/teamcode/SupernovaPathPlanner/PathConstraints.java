package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

public class PathConstraints {
    public double maxTranslationPower;
    public double maxFinalTranslationPower;
    public double maxHeadingPower;
    public double positionTolerance;
    public double headingTolerance;

    public PathConstraints(FollowerConfig config) {
        maxTranslationPower = config.maxTranslationPower;
        maxFinalTranslationPower = config.maxFinalTranslationPower;
        maxHeadingPower = config.maxHeadingPower;
        positionTolerance = config.positionTolerance;
        headingTolerance = config.headingTolerance;
    }

    public PathConstraints withMaxTranslationPower(double maxTranslationPower) {
        this.maxTranslationPower = maxTranslationPower;
        return this;
    }

    public PathConstraints withMaxFinalTranslationPower(double maxFinalTranslationPower) {
        this.maxFinalTranslationPower = maxFinalTranslationPower;
        return this;
    }

    public PathConstraints withMaxHeadingPower(double maxHeadingPower) {
        this.maxHeadingPower = maxHeadingPower;
        return this;
    }

    public PathConstraints withPositionTolerance(double positionTolerance) {
        this.positionTolerance = positionTolerance;
        return this;
    }

    public PathConstraints withHeadingTolerance(double headingTolerance) {
        this.headingTolerance = headingTolerance;
        return this;
    }
}
