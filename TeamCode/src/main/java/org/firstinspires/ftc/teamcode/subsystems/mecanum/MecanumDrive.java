package org.firstinspires.ftc.teamcode.subsystems.mecanum;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

/**
 * Documentation ajoutee: Drivetrain mecanum qui transforme les commandes de translation/rotation en puissances moteur.
 */
public class MecanumDrive implements Drivetrain {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private MecanumDriveConfig config;

    // Construit cette classe avec les dependances et reglages necessaires.
    public MecanumDrive(HardwareMap hardwareMap, MecanumDriveConfig config) {
        this(
                hardwareMap.get(DcMotorEx.class, config.frontLeftMotorName),
                hardwareMap.get(DcMotorEx.class, config.frontRightMotorName),
                hardwareMap.get(DcMotorEx.class, config.backLeftMotorName),
                hardwareMap.get(DcMotorEx.class, config.backRightMotorName),
                config
        );
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight,
                        DcMotorEx backLeft, DcMotorEx backRight,
                        MecanumDriveConfig config) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        setConfig(config);
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    @Override
    public void driveFieldCentric(Pose driveCommand, Pose currentPose, Pose velocity, Pose targetPose) {
        Pose scaledCommand = applyPredictiveBraking(driveCommand, currentPose, velocity, targetPose);
        driveFieldCentric(scaledCommand, currentPose.getHeading());
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
    public void driveRobotCentric(double forward, double strafe, double turn) {
        forward = Range.clip(forward, -1.0, 1.0);
        strafe = Range.clip(strafe, -1.0, 1.0);
        turn = Range.clip(turn, -1.0, 1.0);

        if (shouldXLock(forward, strafe, turn)) {
            stop();
            return;
        }

        double frontLeftPower = forward + strafe + turn;
        double frontRightPower = forward - strafe - turn;
        double backLeftPower = forward - strafe + turn;
        double backRightPower = forward + strafe - turn;

        double maxMagnitude = Math.max(1.0, Math.abs(frontLeftPower));
        maxMagnitude = Math.max(maxMagnitude, Math.abs(frontRightPower));
        maxMagnitude = Math.max(maxMagnitude, Math.abs(backLeftPower));
        maxMagnitude = Math.max(maxMagnitude, Math.abs(backRightPower));

        double maxPower = config == null ? 1.0 : Range.clip(config.maxPower, 0.0, 1.0);
        frontLeft.setPower((frontLeftPower / maxMagnitude) * maxPower);
        frontRight.setPower((frontRightPower / maxMagnitude) * maxPower);
        backLeft.setPower((backLeftPower / maxMagnitude) * maxPower);
        backRight.setPower((backRightPower / maxMagnitude) * maxPower);
    }

    // Arrete proprement les moteurs et remet les commandes a zero.
    @Override
    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setConfig(MecanumDriveConfig config) {
        this.config = config;

        if (config == null) {
            return;
        }

        frontLeft.setDirection(config.frontLeftDirection);
        frontRight.setDirection(config.frontRightDirection);
        backLeft.setDirection(config.backLeftDirection);
        backRight.setDirection(config.backRightDirection);

        frontLeft.setZeroPowerBehavior(config.zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(config.zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(config.zeroPowerBehavior);
        backRight.setZeroPowerBehavior(config.zeroPowerBehavior);

        frontLeft.setMode(config.runMode);
        frontRight.setMode(config.runMode);
        backLeft.setMode(config.runMode);
        backRight.setMode(config.runMode);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public MecanumDriveConfig getConfig() {
        return config;
    }

    // Calcule une valeur intermediaire utilisee pour limiter ou corriger la commande.
    public Pose applyPredictiveBraking(Pose driveCommand, Pose currentPose, Pose velocity, Pose targetPose) {
        if (config == null || !config.predictiveBrakingEnabled) {
            return driveCommand.copy();
        }

        double distanceToTarget = currentPose.distanceTo(targetPose);
        double linearVelocity = Math.hypot(velocity.getX(), velocity.getY());
        double linearStoppingDistance = calculateStoppingDistance(linearVelocity, config.maxLinearDeceleration);
        double translationScale = calculateBrakeScale(distanceToTarget, linearStoppingDistance);

        double headingError = Math.abs(currentPose.headingErrorTo(targetPose));
        double angularVelocity = Math.abs(velocity.getHeading());
        double angularStoppingDistance = calculateStoppingDistance(angularVelocity, config.maxAngularDeceleration);
        double rotationScale = calculateBrakeScale(headingError, angularStoppingDistance);

        return new Pose(
                driveCommand.getX() * translationScale,
                driveCommand.getY() * translationScale,
                driveCommand.getHeading() * rotationScale
        );
    }

    // Calcule une valeur intermediaire utilisee pour limiter ou corriger la commande.
    private double calculateBrakeScale(double remainingDistance, double stoppingDistance) {
        double brakingDistance = stoppingDistance + config.brakingDistanceBuffer;

        if (brakingDistance <= 0.0 || remainingDistance >= brakingDistance) {
            return 1.0;
        }

        return Range.clip(remainingDistance / brakingDistance, config.minimumBrakeScale, 1.0);
    }

    // Calcule une valeur intermediaire utilisee pour limiter ou corriger la commande.
    private static double calculateStoppingDistance(double velocity, double maxDeceleration) {
        if (maxDeceleration <= 0.0) {
            return 0.0;
        }

        return (velocity * velocity) / (2.0 * maxDeceleration);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    private boolean shouldXLock(double forward, double strafe, double turn) {
        if (config == null || !config.xLockEnabled) {
            return false;
        }

        double deadband = Math.abs(config.xLockDeadband);
        return Math.abs(forward) <= deadband
                && Math.abs(strafe) <= deadband
                && Math.abs(turn) <= deadband;
    }
}
