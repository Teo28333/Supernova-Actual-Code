package org.firstinspires.ftc.teamcode.pathplanner;

public interface PathSegment {
    Pose get(double t);

    Pose derivative(double t);

    double length();
}
