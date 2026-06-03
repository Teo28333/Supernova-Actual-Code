package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveDriveConfig {
    public final SwerveModuleConfig frontLeft;
    public final SwerveModuleConfig frontRight;
    public final SwerveModuleConfig backLeft;
    public final SwerveModuleConfig backRight;
    public boolean centripetalCorrectionEnabled = true;
    public double centripetalCorrectionScale = 0.0;
    public boolean xLockEnabled = true;
    public double xLockDeadband = 0.03;
    public boolean brakeModeEnabled = true;

    public SwerveDriveConfig(SwerveModuleConfig frontLeft, SwerveModuleConfig frontRight,
                             SwerveModuleConfig backLeft, SwerveModuleConfig backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public SwerveDriveConfig withCentripetalCorrection(boolean enabled) {
        centripetalCorrectionEnabled = enabled;
        return this;
    }

    public SwerveDriveConfig withCentripetalCorrectionScale(double centripetalCorrectionScale) {
        this.centripetalCorrectionScale = centripetalCorrectionScale;
        return this;
    }

    public SwerveDriveConfig withXLock(boolean enabled) {
        xLockEnabled = enabled;
        return this;
    }

    public SwerveDriveConfig withXLockDeadband(double xLockDeadband) {
        this.xLockDeadband = xLockDeadband;
        return this;
    }

    public SwerveDriveConfig withBrakeMode(boolean brakeModeEnabled) {
        this.brakeModeEnabled = brakeModeEnabled;
        return this;
    }

    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return brakeModeEnabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
    }
}
