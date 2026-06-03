package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

public abstract class PathCallback {
    private final boolean runOnce;
    private boolean hasRun = false;

    public PathCallback(boolean runOnce) {
        this.runOnce = runOnce;
    }

    public void update(Pose pose, int pathIndex, int segmentIndex, double segmentT, double elapsedSeconds) {
        if (runOnce && hasRun) {
            return;
        }

        if (shouldRun(pose, pathIndex, segmentIndex, segmentT, elapsedSeconds)) {
            run();
            hasRun = true;
        }
    }

    public void reset() {
        hasRun = false;
    }

    protected abstract boolean shouldRun(Pose pose, int pathIndex, int segmentIndex,
                                         double segmentT, double elapsedSeconds);

    protected abstract void run();
}
