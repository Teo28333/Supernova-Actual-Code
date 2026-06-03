package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

public class SwerveDriveConfig {
    public final SwerveModuleConfig frontLeft;
    public final SwerveModuleConfig frontRight;
    public final SwerveModuleConfig backLeft;
    public final SwerveModuleConfig backRight;

    public SwerveDriveConfig(SwerveModuleConfig frontLeft, SwerveModuleConfig frontRight,
                             SwerveModuleConfig backLeft, SwerveModuleConfig backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }
}
