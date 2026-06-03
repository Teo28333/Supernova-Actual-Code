package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

public interface Localizer {
    void update();

    Pose getPose();

    Pose getVelocity();

    void setPose(Pose pose);
}
