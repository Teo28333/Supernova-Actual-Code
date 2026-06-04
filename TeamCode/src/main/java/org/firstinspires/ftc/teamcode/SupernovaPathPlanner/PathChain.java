package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PathCallback;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Documentation ajoutee: Sequence de chemins Supernova avec callbacks globaux et acces au depart/arrivee.
 */
public class PathChain {
    private final List<SupernovaPath> paths = new ArrayList<>();
    private final List<PathCallback> callbacks = new ArrayList<>();

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathChain addPath(SupernovaPath path) {
        if (path == null || path.size() == 0) {
            throw new IllegalArgumentException("Path must contain at least one segment.");
        }

        paths.add(path);
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathChain addCallback(PathCallback callback) {
        callbacks.add(callback);
        return this;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public SupernovaPath get(int index) {
        return paths.get(index);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public int size() {
        return paths.size();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public List<SupernovaPath> getPaths() {
        return Collections.unmodifiableList(paths);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public List<PathCallback> getCallbacks() {
        return Collections.unmodifiableList(callbacks);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getStartPose() {
        if (paths.isEmpty()) {
            return new Pose();
        }

        return paths.get(0).getStartPose();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getEndPose() {
        if (paths.isEmpty()) {
            return new Pose();
        }

        return paths.get(paths.size() - 1).getEndPose();
    }

    // Remet l'etat interne a une base propre pour eviter les anciennes erreurs accumulees.
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
