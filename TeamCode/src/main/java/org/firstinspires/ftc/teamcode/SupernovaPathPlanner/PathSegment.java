package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

/**
 * Documentation ajoutee: Interface de segment de trajectoire, capable de donner une pose, une derivee et une longueur.
 */
public interface PathSegment {
    Pose get(double t);

    Pose derivative(double t);

    double length();
}
