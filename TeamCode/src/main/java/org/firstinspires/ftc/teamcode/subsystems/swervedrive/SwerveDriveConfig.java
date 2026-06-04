package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Documentation ajoutee: Regroupe les quatre configurations de modules swerve et les options globales du drivetrain.
 */
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

    // Construit cette classe avec les dependances et reglages necessaires.
    public SwerveDriveConfig(SwerveModuleConfig frontLeft, SwerveModuleConfig frontRight,
                             SwerveModuleConfig backLeft, SwerveModuleConfig backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveDriveConfig withCentripetalCorrection(boolean enabled) {
        centripetalCorrectionEnabled = enabled;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveDriveConfig withCentripetalCorrectionScale(double centripetalCorrectionScale) {
        this.centripetalCorrectionScale = centripetalCorrectionScale;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveDriveConfig withXLock(boolean enabled) {
        xLockEnabled = enabled;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveDriveConfig withXLockDeadband(double xLockDeadband) {
        this.xLockDeadband = xLockDeadband;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveDriveConfig withBrakeMode(boolean brakeModeEnabled) {
        this.brakeModeEnabled = brakeModeEnabled;
        return this;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public DcMotor.ZeroPowerBehavior getZeroPowerBehavior() {
        return brakeModeEnabled ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
    }
}
