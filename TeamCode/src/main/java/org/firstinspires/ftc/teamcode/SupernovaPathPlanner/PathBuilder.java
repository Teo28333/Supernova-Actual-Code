package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.ParametricCallback;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PathCallback;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PoseCallback;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.TemporalCallback;

public class PathBuilder {
    private final PathChain chain = new PathChain();
    private SupernovaPath currentPath = new SupernovaPath();

    public PathBuilder startAt(Pose pose) {
        currentPath.startAt(pose);
        return this;
    }

    public PathBuilder addPath(SupernovaPath path) {
        finishCurrentPathIfNeeded();
        chain.addPath(path);
        currentPath = new SupernovaPath().startAt(path.getEndPose());
        return this;
    }

    public PathBuilder lineTo(Pose pose) {
        currentPath.lineTo(pose);
        return this;
    }

    public PathBuilder lineTo(double x, double y, double heading) {
        return lineTo(new Pose(x, y, heading));
    }

    public PathBuilder curveTo(Pose controlOne, Pose controlTwo, Pose end) {
        currentPath.curveTo(controlOne, controlTwo, end);
        return this;
    }

    public PathBuilder curveTo(Pose start, Pose controlOne, Pose controlTwo, Pose end) {
        currentPath.curveTo(start, controlOne, controlTwo, end);
        return this;
    }

    public PathBuilder setConstraints(PathConstraints constraints) {
        currentPath.setConstraints(constraints);
        return this;
    }

    public PathBuilder addCallback(PathCallback callback) {
        currentPath.addCallback(callback);
        return this;
    }

    public PathBuilder addTemporalCallback(double seconds, Runnable action) {
        return addCallback(new TemporalCallback(seconds, action));
    }

    public PathBuilder addParametricCallback(double t, Runnable action) {
        return addCallback(new ParametricCallback(t, action));
    }

    public PathBuilder addPoseCallback(Pose pose, double positionTolerance,
                                       double headingTolerance, Runnable action) {
        return addCallback(new PoseCallback(pose, positionTolerance, headingTolerance, action));
    }

    public PathBuilder nextPath() {
        finishCurrentPathIfNeeded();
        currentPath = new SupernovaPath().startAt(chain.get(chain.size() - 1).getEndPose());
        return this;
    }

    public SupernovaPath buildPath() {
        return currentPath;
    }

    public PathChain build() {
        finishCurrentPathIfNeeded();
        return chain;
    }

    private void finishCurrentPathIfNeeded() {
        if (currentPath.size() > 0) {
            chain.addPath(currentPath);
            currentPath = new SupernovaPath();
        }
    }
}
