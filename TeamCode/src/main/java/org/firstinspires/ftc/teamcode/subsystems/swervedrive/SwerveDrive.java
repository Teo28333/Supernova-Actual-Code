package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.pathplanner.Pose;

public class SwerveDrive {
    private final SwervePods[] pods;
    private final Localizer localizer;
    private final double driveRadius;

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
    }

    public void driveFieldCentric(double forward, double strafe, double rotate) {
        double heading = localizer.getPose().getHeading();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double robotForward = (forward * cos) + (strafe * sin);
        double robotStrafe = (-forward * sin) + (strafe * cos);

        driveRobotCentric(robotForward, robotStrafe, rotate);
    }

    public void driveRobotCentric(double forward, double strafe, double rotate) {
        forward = Range.clip(forward, -1.0, 1.0);
        strafe = Range.clip(strafe, -1.0, 1.0);
        rotate = Range.clip(rotate, -1.0, 1.0);

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

    public void resetPose(Pose pose) {
        localizer.setPose(pose);
    }

    public void stop() {
        for (SwervePods pod : pods) {
            pod.stop();
        }
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

}
