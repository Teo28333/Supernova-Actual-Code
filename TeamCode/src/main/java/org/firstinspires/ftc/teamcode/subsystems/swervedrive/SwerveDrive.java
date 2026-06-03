package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

public class SwerveDrive implements Drivetrain {
    private final SwervePods[] pods;
    private final Localizer localizer;
    private final double driveRadius;
    private SwerveDriveConfig config;

    public SwerveDrive(SwervePods frontLeft, SwervePods frontRight,
                       SwervePods backLeft, SwervePods backRight,
                       Localizer localizer) {
        this.localizer = localizer;
        this.pods = new SwervePods[] {frontLeft, frontRight, backLeft, backRight};
        this.driveRadius = calculateDriveRadius(pods);
    }

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

    public void driveFieldCentric(double forward, double strafe, double rotate) {
        double heading = localizer.getPose().getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotForward = (forward * cos) + (strafe * sin);
        double robotStrafe = (-forward * sin) + (strafe * cos);

        driveRobotCentric(robotForward, robotStrafe, rotate);
    }

    public void driveFieldCentric(Pose driveCommand) {
        driveFieldCentric(driveCommand.getY(), driveCommand.getX(), driveCommand.getHeading());
    }

    public void driveFieldCentric(Pose driveCommand, double robotHeading) {
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double robotForward = (driveCommand.getY() * cos) + (driveCommand.getX() * sin);
        double robotStrafe = (-driveCommand.getY() * sin) + (driveCommand.getX() * cos);

        driveRobotCentric(robotForward, robotStrafe, driveCommand.getHeading());
    }

    public void driveFieldCentricWithBraking(Pose driveCommand, Pose targetPose) {
        Pose scaledCommand = applyPredictiveBraking(driveCommand, targetPose);
        driveFieldCentric(scaledCommand);
    }

    public void driveFieldCentricWithBraking(Pose driveCommand, Pose currentPose,
                                             Pose velocity, Pose targetPose) {
        Pose scaledCommand = applyPredictiveBraking(driveCommand, currentPose, velocity, targetPose);
        driveFieldCentric(scaledCommand, currentPose.getHeading());
    }

    @Override
    public void driveFieldCentric(Pose driveCommand, Pose currentPose, Pose velocity, Pose targetPose) {
        driveFieldCentricWithBraking(driveCommand, currentPose, velocity, targetPose);
    }

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

    public Pose getPose() {
        return localizer.getPose();
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    public SwervePods[] getPods() {
        return pods.clone();
    }

    public SwerveDriveConfig getConfig() {
        return config;
    }

    public void setConfig(SwerveDriveConfig config) {
        this.config = config;

        if (config == null) {
            return;
        }

        for (SwervePods pod : pods) {
            pod.setDriveZeroPowerBehavior(config.getZeroPowerBehavior());
        }
    }

    public void resetPose(Pose pose) {
        localizer.setPose(pose);
    }

    @Override
    public void stop() {
        for (SwervePods pod : pods) {
            pod.stop();
        }
    }

    public void xLock() {
        for (SwervePods pod : pods) {
            pod.pointAt(Math.atan2(pod.getPodPoseX(), pod.getPodPoseY()));
        }
    }

    public void setAllTurnPidf(double kP, double kI, double kD, double kF) {
        for (SwervePods pod : pods) {
            pod.setTurnPidf(kP, kI, kD, kF);
        }
    }

    public void setAllTurnToleranceRadians(double toleranceRadians) {
        for (SwervePods pod : pods) {
            pod.setTurnToleranceRadians(toleranceRadians);
        }
    }

    public void setAllTurnIntegralLimit(double integralLimit) {
        for (SwervePods pod : pods) {
            pod.setIntegralLimit(integralLimit);
        }
    }

    public Pose applyPredictiveBraking(Pose driveCommand, Pose targetPose) {
        if (config == null || !config.predictiveBrakingEnabled) {
            return driveCommand.copy();
        }

        Pose currentPose = localizer.getPose();
        Pose velocity = localizer.getVelocity();
        return applyPredictiveBraking(driveCommand, currentPose, velocity, targetPose);
    }

    public Pose applyPredictiveBraking(Pose driveCommand, Pose currentPose,
                                       Pose velocity, Pose targetPose) {
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

    private static double calculateDriveRadius(SwervePods[] pods) {
        double radius = 0.0;

        for (SwervePods pod : pods) {
            radius = Math.max(radius, Math.hypot(pod.getPodPoseX(), pod.getPodPoseY()));
        }

        return radius == 0.0 ? 1.0 : radius;
    }

    private double calculateBrakeScale(double remainingDistance, double stoppingDistance) {
        double brakingDistance = stoppingDistance + config.brakingDistanceBuffer;

        if (brakingDistance <= 0.0 || remainingDistance >= brakingDistance) {
            return 1.0;
        }

        return Range.clip(remainingDistance / brakingDistance, config.minimumBrakeScale, 1.0);
    }

    private static double calculateStoppingDistance(double velocity, double maxDeceleration) {
        if (maxDeceleration <= 0.0) {
            return 0.0;
        }

        return (velocity * velocity) / (2.0 * maxDeceleration);
    }

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
