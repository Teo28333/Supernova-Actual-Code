package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

/**
 * Documentation ajoutee: Drivetrain swerve complet qui convertit les commandes champ/robot en angles et puissances pour chaque pod.
 */
public class SwerveDrive implements Drivetrain {
    private final SwervePods[] pods;
    private final Localizer localizer;
    private final double driveRadius;
    private SwerveDriveConfig config;

    // Construit cette classe avec les dependances et reglages necessaires.
    public SwerveDrive(SwervePods frontLeft, SwervePods frontRight,
                       SwervePods backLeft, SwervePods backRight,
                       Localizer localizer) {
        this.localizer = localizer;
        this.pods = new SwervePods[] {frontLeft, frontRight, backLeft, backRight};
        this.driveRadius = calculateDriveRadius(pods);
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public SwerveDrive(HardwareMap hardwareMap, Localizer localizer, SwerveDriveConfig config) {
        this(
                createPod(hardwareMap, config.frontLeft),
                createPod(hardwareMap, config.frontRight),
                createPod(hardwareMap, config.backLeft),
                createPod(hardwareMap, config.backRight),
                localizer
        );
        setConfig(config);
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    public void driveFieldCentric(double forward, double strafe, double rotate) {
        double heading = localizer.getPose().getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotForward = (forward * cos) + (strafe * sin);
        double robotStrafe = (-forward * sin) + (strafe * cos);

        driveRobotCentric(robotForward, robotStrafe, rotate);
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    public void driveFieldCentric(Pose driveCommand) {
        driveFieldCentric(driveCommand.getY(), driveCommand.getX(), driveCommand.getHeading());
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    public void driveFieldCentric(Pose driveCommand, double robotHeading) {
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double robotForward = (driveCommand.getY() * cos) + (driveCommand.getX() * sin);
        double robotStrafe = (-driveCommand.getY() * sin) + (driveCommand.getX() * cos);

        driveRobotCentric(robotForward, robotStrafe, driveCommand.getHeading());
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    public void driveFieldCentricWithBraking(Pose driveCommand, Pose targetPose) {
        driveFieldCentric(driveCommand);
    }

    public void driveFieldCentricWithBraking(Pose driveCommand, Pose currentPose,
                                             Pose velocity, Pose targetPose) {
        driveFieldCentric(driveCommand, currentPose.getHeading());
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    @Override
    public void driveFieldCentric(Pose driveCommand, Pose currentPose, Pose velocity, Pose targetPose) {
        driveFieldCentric(driveCommand, currentPose.getHeading());
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    public void driveRobotCentric(double forward, double strafe, double rotate) {
        forward = Range.clip(forward, -1.0, 1.0);
        strafe = Range.clip(strafe, -1.0, 1.0);
        rotate = Range.clip(rotate, -1.0, 1.0);

        if (shouldXLock(forward, strafe, rotate)) {
            xLock();
            return;
        }

        double[] speeds = new double[pods.length];
        double[] angles = new double[pods.length];
        double maxSpeed = 1.0;

        for (int i = 0; i < pods.length; i++) {
            SwervePods pod = pods[i];
            double wheelForward = forward - (rotate * pod.getPodPoseY() / driveRadius);
            double wheelStrafe = strafe + (rotate * pod.getPodPoseX() / driveRadius);

            speeds[i] = Math.hypot(wheelForward, wheelStrafe);
            angles[i] = Math.atan2(wheelStrafe, wheelForward);
            maxSpeed = Math.max(maxSpeed, speeds[i]);
        }

        for (int i = 0; i < pods.length; i++) {
            pods[i].setTargetState(speeds[i] / maxSpeed, angles[i]);
        }
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getPose() {
        return localizer.getPose();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Localizer getLocalizer() {
        return localizer;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public SwervePods[] getPods() {
        return pods.clone();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public SwerveDriveConfig getConfig() {
        return config;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setConfig(SwerveDriveConfig config) {
        this.config = config;

        if (config == null) {
            return;
        }

        for (SwervePods pod : pods) {
            pod.setDriveZeroPowerBehavior(config.getZeroPowerBehavior());
        }
    }

    // Remet l'etat interne a une base propre pour eviter les anciennes erreurs accumulees.
    public void resetPose(Pose pose) {
        localizer.setPose(pose);
    }

    // Arrete proprement les moteurs et remet les commandes a zero.
    @Override
    public void stop() {
        for (SwervePods pod : pods) {
            pod.stop();
        }
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    public void xLock() {
        for (SwervePods pod : pods) {
            pod.pointAt(Math.atan2(pod.getPodPoseX(), pod.getPodPoseY()));
        }
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setAllTurnPidf(double kP, double kI, double kD, double kF) {
        for (SwervePods pod : pods) {
            pod.setTurnPidf(kP, kI, kD, kF);
        }
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setAllTurnToleranceRadians(double toleranceRadians) {
        for (SwervePods pod : pods) {
            pod.setTurnToleranceRadians(toleranceRadians);
        }
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setAllTurnIntegralLimit(double integralLimit) {
        for (SwervePods pod : pods) {
            pod.setIntegralLimit(integralLimit);
        }
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    private static SwervePods createPod(HardwareMap hardwareMap, SwerveModuleConfig config) {
        SwervePods pod = new SwervePods(
                hardwareMap,
                config.driveMotorName,
                config.turnServoName,
                config.absoluteEncoderName,
                config.angleOffset,
                config.minVoltage,
                config.maxVoltage
        );

        pod.setPodPose(config.modulePose);
        pod.setTurnPidf(config.turnKp, config.turnKi, config.turnKd, config.turnKf);
        pod.setTurnToleranceRadians(config.turnToleranceRadians);
        pod.setIntegralLimit(config.turnIntegralLimit);
        pod.setDriveReversed(config.driveReversed);
        pod.setTurnReversed(config.turnReversed);

        return pod;
    }

    // Calcule une valeur intermediaire utilisee pour limiter ou corriger la commande.
    private static double calculateDriveRadius(SwervePods[] pods) {
        double radius = 0.0;

        for (SwervePods pod : pods) {
            radius = Math.max(radius, Math.hypot(pod.getPodPoseX(), pod.getPodPoseY()));
        }

        return radius == 0.0 ? 1.0 : radius;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    private boolean shouldXLock(double forward, double strafe, double rotate) {
        if (config == null || !config.xLockEnabled) {
            return false;
        }

        double deadband = Math.abs(config.xLockDeadband);
        return Math.abs(forward) <= deadband
                && Math.abs(strafe) <= deadband
                && Math.abs(rotate) <= deadband;
    }

}
