package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Decrit le materiel, la position et les reglages PID d'un module swerve.
 */
public class SwerveModuleConfig {
    public final String driveMotorName;
    public final String turnServoName;
    public final String absoluteEncoderName;
    public final Pose modulePose;
    public double angleOffset = 0.0;
    public double minVoltage = 0.0;
    public double maxVoltage = 3.3;
    public double turnKp = 1.8;
    public double turnKi = 0.0;
    public double turnKd = 0.0;
    public double turnKf = 0.0;
    public double turnToleranceRadians = Math.toRadians(2.0);
    public double turnIntegralLimit = 0.5;
    public boolean driveReversed = false;
    public boolean turnReversed = false;

    // Construit cette classe avec les dependances et reglages necessaires.
    public SwerveModuleConfig(String driveMotorName, String turnServoName,
                              String absoluteEncoderName, Pose modulePose) {
        this.driveMotorName = driveMotorName;
        this.turnServoName = turnServoName;
        this.absoluteEncoderName = absoluteEncoderName;
        this.modulePose = modulePose;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withVoltageRange(double minVoltage, double maxVoltage) {
        this.minVoltage = minVoltage;
        this.maxVoltage = maxVoltage;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withTurnKp(double turnKp) {
        this.turnKp = turnKp;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withTurnPid(double turnKp, double turnKi, double turnKd) {
        this.turnKp = turnKp;
        this.turnKi = turnKi;
        this.turnKd = turnKd;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withTurnPidf(double turnKp, double turnKi, double turnKd, double turnKf) {
        this.turnKp = turnKp;
        this.turnKi = turnKi;
        this.turnKd = turnKd;
        this.turnKf = turnKf;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withTurnToleranceRadians(double turnToleranceRadians) {
        this.turnToleranceRadians = turnToleranceRadians;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withTurnIntegralLimit(double turnIntegralLimit) {
        this.turnIntegralLimit = turnIntegralLimit;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withDriveReversed(boolean driveReversed) {
        this.driveReversed = driveReversed;
        return this;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public SwerveModuleConfig withTurnReversed(boolean turnReversed) {
        this.turnReversed = turnReversed;
        return this;
    }
}
