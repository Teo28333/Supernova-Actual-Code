package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import com.qualcomm.robotcore.util.Range;

public class SupernovaLine implements PathSegment {
    private final Pose start;
    private final Pose end;
    private final double length;

    public SupernovaLine(Pose start, Pose end) {
        this.start = start.copy();
        this.end = end.copy();
        this.length = start.distanceTo(end);
    }

    @Override
    public Pose get(double t) {
        t = Range.clip(t, 0.0, 1.0);
        double headingDelta = Pose.normalizeRadians(end.getHeading() - start.getHeading());

        return new Pose(
                lerp(start.getX(), end.getX(), t),
                lerp(start.getY(), end.getY(), t),
                Pose.normalizeRadians(start.getHeading() + (headingDelta * t))
        );
    }

    @Override
    public Pose derivative(double t) {
        return new Pose(
                end.getX() - start.getX(),
                end.getY() - start.getY(),
                Pose.normalizeRadians(end.getHeading() - start.getHeading())
        );
    }

    @Override
    public double length() {
        return length;
    }

    private static double lerp(double start, double end, double t) {
        return start + ((end - start) * t);
    }
}
