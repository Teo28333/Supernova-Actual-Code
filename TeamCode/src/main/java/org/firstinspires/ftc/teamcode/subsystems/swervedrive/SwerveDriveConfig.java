package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveDriveConfig {
    public final SwerveModuleConfig frontLeft;
    public final SwerveModuleConfig frontRight;
    public final SwerveModuleConfig backLeft;
    public final SwerveModuleConfig backRight;
    public boolean predictiveBrakingEnabled = true;
    public double maxLinearDeceleration = 40.0;
    public double maxAngularDeceleration = Math.toRadians(360.0);
    public double brakingDistanceBuffer = 2.0;
    public double minimumBrakeScale = 0.12;
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

    public SwerveDriveConfig withPredictiveBraking(boolean enabled) {
        predictiveBrakingEnabled = enabled;
        return this;
    }

    public SwerveDriveConfig withMaxLinearDeceleration(double maxLinearDeceleration) {
        this.maxLinearDeceleration = maxLinearDeceleration;
        return this;
    }

    public SwerveDriveConfig withMaxAngularDeceleration(double maxAngularDeceleration) {
        this.maxAngularDeceleration = maxAngularDeceleration;
        return this;
    }

    public SwerveDriveConfig withBrakingDistanceBuffer(double brakingDistanceBuffer) {
        this.brakingDistanceBuffer = brakingDistanceBuffer;
        return this;
    }

    public SwerveDriveConfig withMinimumBrakeScale(double minimumBrakeScale) {
        this.minimumBrakeScale = minimumBrakeScale;
        return this;
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
