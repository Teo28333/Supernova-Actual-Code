package org.firstinspires.ftc.teamcode.subsystems.mecanum;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

public class MecanumDrive implements Drivetrain {
    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private MecanumDriveConfig config;

    public MecanumDrive(HardwareMap hardwareMap, MecanumDriveConfig config) {
        this(
                hardwareMap.get(DcMotorEx.class, config.frontLeftMotorName),
                hardwareMap.get(DcMotorEx.class, config.frontRightMotorName),
                hardwareMap.get(DcMotorEx.class, config.backLeftMotorName),
                hardwareMap.get(DcMotorEx.class, config.backRightMotorName),
                config
        );
    }

    public MecanumDrive(DcMotorEx frontLeft, DcMotorEx frontRight,
                        DcMotorEx backLeft, DcMotorEx backRight,
                        MecanumDriveConfig config) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        setConfig(config);
    }

    @Override
    public void driveFieldCentric(Pose driveCommand, Pose currentPose, Pose velocity, Pose targetPose) {
        Pose scaledCommand = applyPredictiveBraking(driveCommand, currentPose, velocity, targetPose);
        driveFieldCentric(scaledCommand, currentPose.getHeading());
    }

    public void driveFieldCentric(Pose driveCommand, double robotHeading) {
        double cos = Math.cos(robotHeading);
        double sin = Math.sin(robotHeading);

        double robotForward = (driveCommand.getY() * cos) + (driveCommand.getX() * sin);
        double robotStrafe = (-driveCommand.getY() * sin) + (driveCommand.getX() * cos);

        driveRobotCentric(robotForward, robotStrafe, driveCommand.getHeading());
    }

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

    @Override
    public void stop() {
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

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

    public MecanumDriveConfig getConfig() {
        return config;
    }

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
