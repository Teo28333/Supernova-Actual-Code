package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PathCallback;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SupernovaPath {
    private final List<PathSegment> segments = new ArrayList<>();
    private final List<PathCallback> callbacks = new ArrayList<>();
    private Pose currentPose = null;
    private PathConstraints constraints = null;

    public SupernovaPath startAt(Pose pose) {
        currentPose = pose.copy();
        return this;
    }

    public SupernovaPath add(PathSegment segment) {
        segments.add(segment);
        currentPose = segment.get(1.0);
        return this;
    }

    public SupernovaPath addCallback(PathCallback callback) {
        callbacks.add(callback);
        return this;
    }

    public SupernovaPath setConstraints(PathConstraints constraints) {
        this.constraints = constraints;
        return this;
    }

    public SupernovaPath lineTo(Pose start, Pose end) {
        return add(new SupernovaLine(start, end));
    }

    public SupernovaPath lineTo(Pose end) {
        requireStartPose();
        return add(new SupernovaLine(currentPose, end));
    }

    public SupernovaPath curveTo(Pose start, Pose controlOne, Pose controlTwo, Pose end) {
        return add(new SupernovaCurve(start, controlOne, controlTwo, end));
    }

    public SupernovaPath curveTo(Pose controlOne, Pose controlTwo, Pose end) {
        requireStartPose();
        return add(new SupernovaCurve(currentPose, controlOne, controlTwo, end));
    }

    public PathSegment get(int index) {
        return segments.get(index);
    }

    public int size() {
        return segments.size();
    }

    public List<PathSegment> getSegments() {
        return Collections.unmodifiableList(segments);
    }

    public List<PathCallback> getCallbacks() {
        return Collections.unmodifiableList(callbacks);
    }

    public PathConstraints getConstraints() {
        return constraints;
    }

    public Pose getStartPose() {
        if (segments.isEmpty()) {
            return new Pose();

        }
        return segments.get(0).get(0.0);
    }

    public Pose getEndPose() {
        if (segments.isEmpty()) {
            return currentPose == null ? new Pose() : currentPose.copy();
        }

        return segments.get(segments.size() - 1).get(1.0);
    }

    public double length() {
        double length = 0.0;

        for (PathSegment segment : segments) {
            length += segment.length();
        }

        return length;
    }

    private void requireStartPose() {
        if (currentPose == null) {
            throw new IllegalStateException("Call startAt() before appending a segment without an explicit start pose.");
        }
    }
}
