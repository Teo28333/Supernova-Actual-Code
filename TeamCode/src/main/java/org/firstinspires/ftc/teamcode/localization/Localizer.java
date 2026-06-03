package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

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
