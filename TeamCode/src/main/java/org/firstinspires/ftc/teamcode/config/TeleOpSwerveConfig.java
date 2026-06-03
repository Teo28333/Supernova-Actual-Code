package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveDriveConfig;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveModuleConfig;

public class TeleOpSwerveConfig {
    public static final String PINPOINT_NAME = "pinpoint";

    public static final double TRACK_WIDTH = 13.0;
    public static final double WHEEL_BASE = 13.0;

    public static PinpointLocalizer.Config pinpoint() {
        return SwerveAutonTuningConfig.pinpoint();
    }

    public static SwerveDriveConfig swerveDrive() {
        return SwerveAutonTuningConfig.swerveDrive();
    }

    private static SwerveModuleConfig module(String driveMotorName, String turnServoName,
                                             String absoluteEncoderName, Pose modulePose) {
        return new SwerveModuleConfig(driveMotorName, turnServoName, absoluteEncoderName, modulePose)
                .withVoltageRange(0.0, 3.3)
                .withAngleOffset(0.0)
                .withTurnPidf(1.8, 0.0, 0.05, 0.0)
                .withTurnToleranceRadians(Math.toRadians(2.0))
                .withTurnIntegralLimit(0.5);
    }
}
