package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import com.qualcomm.robotcore.util.Range;

/**
 * Documentation ajoutee: Segment de trajectoire en courbe de Bezier cubique avec estimation de longueur.
 */
public class SupernovaCurve implements PathSegment {
    private static final int LENGTH_SAMPLES = 40;

    private final Pose start;
    private final Pose controlOne;
    private final Pose controlTwo;
    private final Pose end;
    private final double length;

    // Construit cette classe avec les dependances et reglages necessaires.
    public SupernovaCurve(Pose start, Pose controlOne, Pose controlTwo, Pose end) {
        this.start = start.copy();
        this.controlOne = controlOne.copy();
        this.controlTwo = controlTwo.copy();
        this.end = end.copy();
        this.length = estimateLength();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    @Override
    public Pose get(double t) {
        t = Range.clip(t, 0.0, 1.0);
        double inverseT = 1.0 - t;
        double startWeight = inverseT * inverseT * inverseT;
        double controlOneWeight = 3.0 * inverseT * inverseT * t;
        double controlTwoWeight = 3.0 * inverseT * t * t;
        double endWeight = t * t * t;
        double headingDelta = Pose.normalizeRadians(end.getHeading() - start.getHeading());

        return new Pose(
                (start.getX() * startWeight)
                        + (controlOne.getX() * controlOneWeight)
                        + (controlTwo.getX() * controlTwoWeight)
                        + (end.getX() * endWeight),
                (start.getY() * startWeight)
                        + (controlOne.getY() * controlOneWeight)
                        + (controlTwo.getY() * controlTwoWeight)
                        + (end.getY() * endWeight),
                Pose.normalizeRadians(start.getHeading() + (headingDelta * t))
        );
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    @Override
    public Pose derivative(double t) {
        t = Range.clip(t, 0.0, 1.0);
        double inverseT = 1.0 - t;

        double dx = (3.0 * inverseT * inverseT * (controlOne.getX() - start.getX()))
                + (6.0 * inverseT * t * (controlTwo.getX() - controlOne.getX()))
                + (3.0 * t * t * (end.getX() - controlTwo.getX()));
        double dy = (3.0 * inverseT * inverseT * (controlOne.getY() - start.getY()))
                + (6.0 * inverseT * t * (controlTwo.getY() - controlOne.getY()))
                + (3.0 * t * t * (end.getY() - controlTwo.getY()));

        return new Pose(dx, dy, Pose.normalizeRadians(end.getHeading() - start.getHeading()));
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    @Override
    public double length() {
        return length;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    private double estimateLength() {
        double total = 0.0;
        Pose previous = get(0.0);

        for (int i = 1; i <= LENGTH_SAMPLES; i++) {
            Pose current = get((double) i / LENGTH_SAMPLES);
            total += previous.distanceTo(current);
            previous = current;
        }

        return total;
    }
}
