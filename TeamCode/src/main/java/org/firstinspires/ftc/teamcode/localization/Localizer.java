package org.firstinspires.ftc.teamcode.localization;

import org.firstinspires.ftc.teamcode.pathplanner.Pose;

public interface Localizer {
    void update();

    Pose getPose();

    Pose getVelocity();

    void setPose(Pose pose);
}
