package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

public interface PathSegment {
    Pose get(double t);

    Pose derivative(double t);

    double length();
}
