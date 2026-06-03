package org.firstinspires.ftc.teamcode.pathplanner;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PedroPath {
    private final List<PathSegment> segments = new ArrayList<>();
    private Pose currentPose = null;

    public PedroPath startAt(Pose pose) {
        currentPose = pose.copy();
        return this;
    }

    public PedroPath add(PathSegment segment) {
        segments.add(segment);
        currentPose = segment.get(1.0);
        return this;
    }

    public PedroPath lineTo(Pose start, Pose end) {
        return add(new PedroLine(start, end));
    }

    public PedroPath lineTo(Pose end) {
        requireStartPose();
        return add(new PedroLine(currentPose, end));
    }

    public PedroPath curveTo(Pose start, Pose controlOne, Pose controlTwo, Pose end) {
        return add(new PedroCurve(start, controlOne, controlTwo, end));
    }

    public PedroPath curveTo(Pose controlOne, Pose controlTwo, Pose end) {
        requireStartPose();
        return add(new PedroCurve(currentPose, controlOne, controlTwo, end));
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
