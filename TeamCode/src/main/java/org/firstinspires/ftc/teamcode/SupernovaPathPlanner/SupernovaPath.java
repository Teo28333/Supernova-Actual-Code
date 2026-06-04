package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PathCallback;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Documentation ajoutee: Liste de segments formant un chemin, avec contraintes et callbacks associes.
 */
public class SupernovaPath {
    private final List<PathSegment> segments = new ArrayList<>();
    private final List<PathCallback> callbacks = new ArrayList<>();
    private Pose currentPose = null;
    private PathConstraints constraints = null;

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public SupernovaPath startAt(Pose pose) {
        currentPose = pose.copy();
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public SupernovaPath add(PathSegment segment) {
        segments.add(segment);
        currentPose = segment.get(1.0);
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public SupernovaPath addCallback(PathCallback callback) {
        callbacks.add(callback);
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SupernovaPath setConstraints(PathConstraints constraints) {
        this.constraints = constraints;
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public SupernovaPath lineTo(Pose start, Pose end) {
        return add(new SupernovaLine(start, end));
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public SupernovaPath lineTo(Pose end) {
        requireStartPose();
        return add(new SupernovaLine(currentPose, end));
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public SupernovaPath curveTo(Pose start, Pose controlOne, Pose controlTwo, Pose end) {
        return add(new SupernovaCurve(start, controlOne, controlTwo, end));
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public SupernovaPath curveTo(Pose controlOne, Pose controlTwo, Pose end) {
        requireStartPose();
        return add(new SupernovaCurve(currentPose, controlOne, controlTwo, end));
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public PathSegment get(int index) {
        return segments.get(index);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public int size() {
        return segments.size();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public List<PathSegment> getSegments() {
        return Collections.unmodifiableList(segments);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public List<PathCallback> getCallbacks() {
        return Collections.unmodifiableList(callbacks);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public PathConstraints getConstraints() {
        return constraints;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getStartPose() {
        if (segments.isEmpty()) {
            return new Pose();

        }
        return segments.get(0).get(0.0);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getEndPose() {
        if (segments.isEmpty()) {
            return currentPose == null ? new Pose() : currentPose.copy();
        }

        return segments.get(segments.size() - 1).get(1.0);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double length() {
        double length = 0.0;

        for (PathSegment segment : segments) {
            length += segment.length();
        }

        return length;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    private void requireStartPose() {
        if (currentPose == null) {
            throw new IllegalStateException("Call startAt() before appending a segment without an explicit start pose.");
        }
    }
}
