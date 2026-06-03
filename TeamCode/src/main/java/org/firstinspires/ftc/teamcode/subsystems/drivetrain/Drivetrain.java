package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

public interface Drivetrain {
    void driveFieldCentric(Pose driveCommand, Pose currentPose, Pose velocity, Pose targetPose);

    void stop();

    default void update() {
    }
}
