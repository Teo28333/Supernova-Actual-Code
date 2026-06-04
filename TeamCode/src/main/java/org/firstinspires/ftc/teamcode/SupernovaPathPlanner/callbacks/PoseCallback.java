package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Callback qui se declenche lorsque le robot arrive pres d'une pose cible.
 */
public class PoseCallback extends PathCallback {
    private final Pose targetPose;
    private final double positionTolerance;
    private final double headingTolerance;
    private final Runnable action;

    // Construit cette classe avec les dependances et reglages necessaires.
    public PoseCallback(Pose targetPose, double positionTolerance,
                        double headingTolerance, Runnable action) {
        this(targetPose, positionTolerance, headingTolerance, action, true);
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public PoseCallback(Pose targetPose, double positionTolerance,
                        double headingTolerance, Runnable action, boolean runOnce) {
        super(runOnce);
        this.targetPose = targetPose.copy();
        this.positionTolerance = positionTolerance;
        this.headingTolerance = headingTolerance;
        this.action = action;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    @Override
    protected boolean shouldRun(Pose pose, int pathIndex, int segmentIndex, double segmentT, double elapsedSeconds) {
        return pose.distanceTo(targetPose) <= positionTolerance
                && Math.abs(pose.headingErrorTo(targetPose)) <= headingTolerance;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    @Override
    protected void run() {
        action.run();
    }
}
