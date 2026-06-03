package org.firstinspires.ftc.teamcode.pathplanner;

import java.util.Objects;

public class Pose {
    private double x;
    private double y;
    private double heading;

    public Pose() {
        this(0.0, 0.0, 0.0);
    }

    /**
     * Create a pose in field coordinates (Pedro coordinate system).
     *
     * @param x x coordinate in inches
     * @param y y coordinate in inches
     * @param heading heading in radians
     *
     */
    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(Pose pose) {
        this(pose.x, pose.y, pose.heading);
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }

    public void setHeadingDegrees(double headingDegrees) {
        this.heading = Math.toRadians(headingDegrees);
    }

    public void set(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public void set(Pose pose) {
        set(pose.x, pose.y, pose.heading);
    }

    public Pose copy() {
        return new Pose(this);
    }

    public Pose withX(double x) {
        return new Pose(x, y, heading);
    }

    public Pose withY(double y) {
        return new Pose(x, y, heading);
    }

    public Pose withHeading(double heading) {
        return new Pose(x, y, heading);
    }

    public Pose plus(Pose pose) {
        return new Pose(x + pose.x, y + pose.y, heading + pose.heading);
    }

    public Pose minus(Pose pose) {
        return new Pose(x - pose.x, y - pose.y, heading - pose.heading);
    }

    public Pose times(double scalar) {
        return new Pose(x * scalar, y * scalar, heading * scalar);
    }

    public Pose div(double scalar) {
        if (scalar == 0.0) {
            throw new IllegalArgumentException("Cannot divide a pose by zero.");
        }

        return new Pose(x / scalar, y / scalar, heading / scalar);
    }

    public double distanceTo(Pose pose) {
        return Math.hypot(pose.x - x, pose.y - y);
    }

    public double headingTo(Pose pose) {
        return Math.atan2(pose.y - y, pose.x - x);
    }

    public double headingErrorTo(Pose pose) {
        return normalizeRadians(pose.heading - heading);
    }

    public Pose rotated(double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);

        return new Pose(
                (x * cos) - (y * sin),
                (x * sin) + (y * cos),
                heading + angleRadians
        );
    }

    public static Pose fromDegrees(double x, double y, double headingDegrees) {
        return new Pose(x, y, Math.toRadians(headingDegrees));
    }

    public static Pose zero() {
        return new Pose();
    }

    public static double normalizeRadians(double angleRadians) {
        double normalized = Math.IEEEremainder(angleRadians, 2.0 * Math.PI);

        if (normalized <= -Math.PI) {
            normalized += 2.0 * Math.PI;
        }

        if (normalized > Math.PI) {
            normalized -= 2.0 * Math.PI;
        }

        return normalized;
    }
}
