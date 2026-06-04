package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Base abstraite pour declencher une action pendant le suivi d'un chemin.
 */
public abstract class PathCallback {
    private final boolean runOnce;
    private boolean hasRun = false;

    // Construit cette classe avec les dependances et reglages necessaires.
    public PathCallback(boolean runOnce) {
        this.runOnce = runOnce;
    }

    // Rafraichit les lectures, calcule les erreurs et met a jour l'etat interne.
    public void update(Pose pose, int pathIndex, int segmentIndex, double segmentT, double elapsedSeconds) {
        if (runOnce && hasRun) {
            return;
        }

        if (shouldRun(pose, pathIndex, segmentIndex, segmentT, elapsedSeconds)) {
            run();
            hasRun = true;
        }
    }

    // Remet l'etat interne a une base propre pour eviter les anciennes erreurs accumulees.
    public void reset() {
        hasRun = false;
    }

    protected abstract boolean shouldRun(Pose pose, int pathIndex, int segmentIndex,
                                         double segmentT, double elapsedSeconds);

    protected abstract void run();
}
