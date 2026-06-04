package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Callback qui se declenche apres un temps ecoule donne.
 */
public class TemporalCallback extends PathCallback {
    private final double triggerTimeSeconds;
    private final Runnable action;

    // Construit cette classe avec les dependances et reglages necessaires.
    public TemporalCallback(double triggerTimeSeconds, Runnable action) {
        this(triggerTimeSeconds, action, true);
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public TemporalCallback(double triggerTimeSeconds, Runnable action, boolean runOnce) {
        super(runOnce);
        this.triggerTimeSeconds = triggerTimeSeconds;
        this.action = action;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    @Override
    protected boolean shouldRun(Pose pose, int pathIndex, int segmentIndex, double segmentT, double elapsedSeconds) {
        return elapsedSeconds >= triggerTimeSeconds;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    @Override
    protected void run() {
        action.run();
    }
}
