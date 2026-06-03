package org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

public class TemporalCallback extends PathCallback {
    private final double triggerTimeSeconds;
    private final Runnable action;

    public TemporalCallback(double triggerTimeSeconds, Runnable action) {
        this(triggerTimeSeconds, action, true);
    }

    public TemporalCallback(double triggerTimeSeconds, Runnable action, boolean runOnce) {
        super(runOnce);
        this.triggerTimeSeconds = triggerTimeSeconds;
        this.action = action;
    }

    @Override
    protected boolean shouldRun(Pose pose, int pathIndex, int segmentIndex, double segmentT, double elapsedSeconds) {
        return elapsedSeconds >= triggerTimeSeconds;
    }

    @Override
    protected void run() {
        action.run();
    }
}
