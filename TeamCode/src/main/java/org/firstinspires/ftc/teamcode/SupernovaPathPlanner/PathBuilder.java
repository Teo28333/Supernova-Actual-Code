package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.ParametricCallback;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PathCallback;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PoseCallback;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.TemporalCallback;

/**
 * Documentation ajoutee: Constructeur fluide pour assembler des lignes, courbes, contraintes et callbacks de trajectoire.
 */
public class PathBuilder {
    private final PathChain chain = new PathChain();
    private SupernovaPath currentPath = new SupernovaPath();

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathBuilder startAt(Pose pose) {
        currentPath.startAt(pose);
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathBuilder addPath(SupernovaPath path) {
        finishCurrentPathIfNeeded();
        chain.addPath(path);
        currentPath = new SupernovaPath().startAt(path.getEndPose());
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathBuilder lineTo(Pose pose) {
        currentPath.lineTo(pose);
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathBuilder lineTo(double x, double y, double heading) {
        return lineTo(new Pose(x, y, heading));
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathBuilder curveTo(Pose controlOne, Pose controlTwo, Pose end) {
        currentPath.curveTo(controlOne, controlTwo, end);
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathBuilder curveTo(Pose start, Pose controlOne, Pose controlTwo, Pose end) {
        currentPath.curveTo(start, controlOne, controlTwo, end);
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public PathBuilder setConstraints(PathConstraints constraints) {
        currentPath.setConstraints(constraints);
        return this;
    }

    // Ajoute ou lance une etape de trajectoire dans l'API de path planning.
    public PathBuilder addCallback(PathCallback callback) {
        currentPath.addCallback(callback);
        return this;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public PathBuilder addTemporalCallback(double seconds, Runnable action) {
        return addCallback(new TemporalCallback(seconds, action));
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public PathBuilder addParametricCallback(double t, Runnable action) {
        return addCallback(new ParametricCallback(t, action));
    }

    public PathBuilder addPoseCallback(Pose pose, double positionTolerance,
                                       double headingTolerance, Runnable action) {
        return addCallback(new PoseCallback(pose, positionTolerance, headingTolerance, action));
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public PathBuilder nextPath() {
        finishCurrentPathIfNeeded();
        currentPath = new SupernovaPath().startAt(chain.get(chain.size() - 1).getEndPose());
        return this;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public SupernovaPath buildPath() {
        return currentPath;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public PathChain build() {
        finishCurrentPathIfNeeded();
        return chain;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    private void finishCurrentPathIfNeeded() {
        if (currentPath.size() > 0) {
            chain.addPath(currentPath);
            currentPath = new SupernovaPath();
        }
    }
}
