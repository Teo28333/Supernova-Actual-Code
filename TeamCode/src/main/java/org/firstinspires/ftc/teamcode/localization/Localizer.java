package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Contrat commun pour tous les systemes qui estiment la position et la vitesse du robot.
 */
public interface Localizer {
    void update();

    Pose getPose();

    default Pose getPoseEstimate() {
        return getPose();
    }

    Pose getVelocity();

    void setPose(Pose pose);

    default void setStartingPose(Pose pose) {
        setPose(pose);
    }
}
