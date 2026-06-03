package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

public class ParametricCallback extends PathCallback {
    private final int triggerPathIndex;
    private final int triggerSegmentIndex;
    private final double triggerT;
    private final Runnable action;

    public ParametricCallback(double triggerT, Runnable action) {
        this(-1, -1, triggerT, action, true);
    }

    public ParametricCallback(int triggerPathIndex, int triggerSegmentIndex,
                              double triggerT, Runnable action, boolean runOnce) {
        super(runOnce);
        this.triggerPathIndex = triggerPathIndex;
        this.triggerSegmentIndex = triggerSegmentIndex;
        this.triggerT = triggerT;
        this.action = action;
    }

    @Override
    protected boolean shouldRun(Pose pose, int pathIndex, int segmentIndex, double segmentT, double elapsedSeconds) {
        boolean pathMatches = triggerPathIndex < 0 || triggerPathIndex == pathIndex;
        boolean segmentMatches = triggerSegmentIndex < 0 || triggerSegmentIndex == segmentIndex;
        return pathMatches && segmentMatches && segmentT >= triggerT;
    }

    @Override
    protected void run() {
        action.run();
    }
}
