package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Interface commune utilisee par le follower pour commander n'importe quel type de drivetrain.
 */
public interface Drivetrain {
    void driveFieldCentric(Pose driveCommand, Pose currentPose, Pose velocity, Pose targetPose);

    void stop();

    default void update() {
    }
}
