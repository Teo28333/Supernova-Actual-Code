package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Callback qui se declenche quand le parametre t d'un segment atteint un seuil.
 */
public class ParametricCallback extends PathCallback {
    private final int triggerPathIndex;
    private final int triggerSegmentIndex;
    private final double triggerT;
    private final Runnable action;

    // Construit cette classe avec les dependances et reglages necessaires.
    public ParametricCallback(double triggerT, Runnable action) {
        this(-1, -1, triggerT, action, true);
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public ParametricCallback(int triggerPathIndex, int triggerSegmentIndex,
                              double triggerT, Runnable action, boolean runOnce) {
        super(runOnce);
        this.triggerPathIndex = triggerPathIndex;
        this.triggerSegmentIndex = triggerSegmentIndex;
        this.triggerT = triggerT;
        this.action = action;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    @Override
    protected boolean shouldRun(Pose pose, int pathIndex, int segmentIndex, double segmentT, double elapsedSeconds) {
        boolean pathMatches = triggerPathIndex < 0 || triggerPathIndex == pathIndex;
        boolean segmentMatches = triggerSegmentIndex < 0 || triggerSegmentIndex == segmentIndex;
        return pathMatches && segmentMatches && segmentT >= triggerT;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    @Override
    protected void run() {
        action.run();
    }
}
