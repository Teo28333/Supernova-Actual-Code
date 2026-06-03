package org.firstinspires.ftc.teamcode.pathplanner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class SupernovaPath {
    private final List<PathSegment> segments = new ArrayList<>();
    private Pose currentPose = null;

    public SupernovaPath startAt(Pose pose) {
        currentPose = pose.copy();
        return this;
    }

    public SupernovaPath add(PathSegment segment) {
        segments.add(segment);
        currentPose = segment.get(1.0);
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

    private void requireStartPose() {
        if (currentPose == null) {
            throw new IllegalStateException("Call startAt() before appending a segment without an explicit start pose.");
        }
    }
}
