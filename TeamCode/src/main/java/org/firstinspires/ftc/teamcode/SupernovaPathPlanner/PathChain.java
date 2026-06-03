package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PathCallback;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class PathChain {
    private final List<SupernovaPath> paths = new ArrayList<>();
    private final List<PathCallback> callbacks = new ArrayList<>();

    public PathChain addPath(SupernovaPath path) {
        if (path == null || path.size() == 0) {
            throw new IllegalArgumentException("Path must contain at least one segment.");
        }

        paths.add(path);
        return this;
    }

    public PathChain addCallback(PathCallback callback) {
        callbacks.add(callback);
        return this;
    }

    public SupernovaPath get(int index) {
        return paths.get(index);
    }

    public int size() {
        return paths.size();
    }

    public List<SupernovaPath> getPaths() {
        return Collections.unmodifiableList(paths);
    }

    public List<PathCallback> getCallbacks() {
        return Collections.unmodifiableList(callbacks);
    }

    public Pose getStartPose() {
        if (paths.isEmpty()) {
            return new Pose();
        }

        return paths.get(0).getStartPose();
    }

    public Pose getEndPose() {
        if (paths.isEmpty()) {
            return new Pose();
        }

        return paths.get(paths.size() - 1).getEndPose();
    }

    public void resetCallbacks() {
        for (PathCallback callback : callbacks) {
            callback.reset();
        }

        for (SupernovaPath path : paths) {
            for (PathCallback callback : path.getCallbacks()) {
                callback.reset();
            }
        }
    }
}
