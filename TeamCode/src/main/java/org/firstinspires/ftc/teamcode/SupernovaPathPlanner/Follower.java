package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control.PIDFController;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.control.PredictiveBrakingCoefficients;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.callbacks.PathCallback;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;

public class Follower {
    private static final int CLOSEST_POINT_SAMPLES = 80;

    private final Drivetrain drivetrain;
    private final Localizer localizer;
    private FollowerConfig config;
    private PathChain pathChain;
    private SupernovaPath path;
    private int pathIndex = 0;
    private int segmentIndex = 0;
    private double segmentT = 0.0;
    private Pose pose = new Pose();
    private Pose velocity = new Pose();
    private Pose targetPose = new Pose();
    private Pose driveCommand = new Pose();
    private boolean busy = false;
    private boolean holdingPoint = false;
    private Pose holdPose = new Pose();
    private long followStartTimeNanos = 0L;
    private PIDFController translationXController;
    private PIDFController translationYController;
    private PIDFController headingController;

    public Follower(SupernovaConfig config, HardwareMap hardwareMap) {
        Follower follower = config.createFollower(hardwareMap);
        this.drivetrain = follower.drivetrain;
        this.localizer = follower.localizer;
        this.config = follower.config;
        configureControllers();
    }

    public Follower(Drivetrain drivetrain, Localizer localizer, FollowerConfig config) {
        this.drivetrain = drivetrain;
        this.localizer = localizer;
        this.config = config;
        configureControllers();
    }

    public void setConfig(FollowerConfig config) {
        this.config = config;
        configureControllers();
    }

    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }

    public void setStartingPose(Pose pose) {
        localizer.setStartingPose(pose);
        this.pose = pose.copy();
        this.velocity = new Pose();
        resetControllers();
    }

    public void followPath(SupernovaPath path) {
        followPath(path, false);
    }

    public void followPath(SupernovaPath path, boolean resetPose) {
        if (path == null || path.size() == 0) {
            throw new IllegalArgumentException("Path must contain at least one segment.");
        }

        this.path = path;
        this.pathChain = new PathChain().addPath(path);
        this.pathIndex = 0;
        segmentIndex = 0;
        segmentT = 0.0;
        targetPose = path.get(0).get(0.0);
        driveCommand = new Pose();
        busy = true;
        holdingPoint = false;
        followStartTimeNanos = System.nanoTime();
        pathChain.resetCallbacks();
        resetControllers();

        if (resetPose) {
            setStartingPose(path.getStartPose());
        }
    }

    public void followPathChain(PathChain pathChain) {
        followPathChain(pathChain, false);
    }

    public void followPathChain(PathChain pathChain, boolean resetPose) {
        if (pathChain == null || pathChain.size() == 0) {
            throw new IllegalArgumentException("Path chain must contain at least one path.");
        }

        this.pathChain = pathChain;
        this.pathIndex = 0;
        this.path = pathChain.get(0);
        segmentIndex = 0;
        segmentT = 0.0;
        targetPose = path.get(0).get(0.0);
        driveCommand = new Pose();
        busy = true;
        holdingPoint = false;
        followStartTimeNanos = System.nanoTime();
        pathChain.resetCallbacks();
        resetControllers();

        if (resetPose) {
            setStartingPose(pathChain.getStartPose());
        }
    }

    public void holdPoint(Pose pose) {
        holdPoint(pose, false);
    }

    public void holdPoint(Pose pose, boolean resetPose) {
        this.holdPose = pose.copy();
        this.targetPose = pose.copy();
        this.driveCommand = new Pose();
        this.busy = true;
        this.holdingPoint = true;
        this.path = null;
        this.pathChain = null;
        this.followStartTimeNanos = System.nanoTime();
        resetControllers();

        if (resetPose) {
            setStartingPose(pose);
        }
    }

    public void turnTo(double headingRadians) {
        Pose currentPose = localizer.getPose();
        holdPoint(currentPose.withHeading(headingRadians));
    }

    public void update() {
        localizer.update();
        pose = localizer.getPoseEstimate();
        velocity = localizer.getVelocity();

        if (!busy || path == null) {
            if (holdingPoint) {
                updateHoldPoint();
                return;
            }

            drivetrain.stop();
            return;
        }

        PathSegment segment = path.get(segmentIndex);
        segmentT = Math.max(segmentT, findClosestT(segment, pose, segmentT));

        while (segmentT >= 1.0 - config.segmentAdvanceTolerance && segmentIndex < path.size() - 1) {
            segmentIndex++;
            segment = path.get(segmentIndex);
            segmentT = 0.0;
        }

        while (segmentT >= 1.0 - config.segmentAdvanceTolerance
                && segmentIndex == path.size() - 1
                && pathChain != null
                && pathIndex < pathChain.size() - 1) {
            pathIndex++;
            path = pathChain.get(pathIndex);
            segmentIndex = 0;
            segmentT = 0.0;
            segment = path.get(segmentIndex);
            resetControllers();
        }

        double targetT = Range.clip(segmentT + config.lookaheadDistance / Math.max(segment.length(), 1.0), 0.0, 1.0);
        targetPose = segment.get(targetT);
        updateCallbacks();

        Pose endPose = pathChain == null ? path.getEndPose() : pathChain.getEndPose();
        double distanceToEnd = pose.distanceTo(endPose);
        double headingErrorToEnd = Math.abs(pose.headingErrorTo(endPose));
        boolean atEndSegment = segmentIndex == path.size() - 1
                && (pathChain == null || pathIndex == pathChain.size() - 1);

        PathConstraints activeConstraints = getActiveConstraints();

        if (atEndSegment
                && distanceToEnd <= activeConstraints.positionTolerance
                && headingErrorToEnd <= activeConstraints.headingTolerance) {
            busy = false;
            driveCommand = new Pose();
            drivetrain.stop();
            return;
        }

        driveCommand = calculateDriveCommand(pose, targetPose, endPose, atEndSegment, activeConstraints)
                .plus(calculateCentripetalCorrection(segment, targetT, velocity));
        driveCommand = clampDriveCommand(driveCommand, activeConstraints, atEndSegment);
        drivetrain.driveFieldCentric(driveCommand, pose, velocity, endPose);
    }

    public void stop() {
        busy = false;
        holdingPoint = false;
        driveCommand = new Pose();
        drivetrain.stop();
    }

    public boolean isBusy() {
        return busy;
    }

    public Pose getPose() {
        return pose.copy();
    }

    public Pose getVelocity() {
        return velocity.copy();
    }

    public Pose getTargetPose() {
        return targetPose.copy();
    }

    public Pose getDriveCommand() {
        return driveCommand.copy();
    }

    public int getSegmentIndex() {
        return segmentIndex;
    }

    public int getPathIndex() {
        return pathIndex;
    }

    public double getSegmentT() {
        return segmentT;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Drivetrain getDrive() {
        return drivetrain;
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    private Pose calculateDriveCommand(Pose currentPose, Pose lookaheadPose, Pose endPose,
                                       boolean atEndSegment, PathConstraints activeConstraints) {
        Pose trackingPose = atEndSegment && currentPose.distanceTo(endPose) < config.finalApproachDistance
                ? endPose
                : lookaheadPose;

        if (config.predictiveBrakingEnabled && atEndSegment) {
            return calculatePredictiveBrakingCommand(currentPose, endPose, activeConstraints);
        }

        double xPower = translationXController.updateError(trackingPose.getX() - currentPose.getX());
        double yPower = translationYController.updateError(trackingPose.getY() - currentPose.getY());
        double headingPower = headingController.updateError(
                Pose.normalizeRadians(trackingPose.getHeading() - currentPose.getHeading())
        );

        double translationMagnitude = Math.hypot(xPower, yPower);
        double maxTranslationPower = atEndSegment
                ? activeConstraints.maxFinalTranslationPower
                : activeConstraints.maxTranslationPower;

        if (translationMagnitude > maxTranslationPower) {
            double scale = maxTranslationPower / translationMagnitude;
            xPower *= scale;
            yPower *= scale;
        }

        return new Pose(
                Range.clip(xPower, -maxTranslationPower, maxTranslationPower),
                Range.clip(yPower, -maxTranslationPower, maxTranslationPower),
                Range.clip(headingPower, -activeConstraints.maxHeadingPower, activeConstraints.maxHeadingPower)
        );
    }

    private Pose calculatePredictiveBrakingCommand(Pose currentPose, Pose endPose,
                                                   PathConstraints activeConstraints) {
        PredictiveBrakingCoefficients coefficients = config.predictiveBrakingCoefficients;
        double speed = Math.hypot(velocity.getX(), velocity.getY());
        double brakingDistance = (coefficients.kLinear * speed)
                + (coefficients.kQuadratic * speed * speed);

        double predictedX = currentPose.getX();
        double predictedY = currentPose.getY();

        if (speed > 1e-6) {
            predictedX += (velocity.getX() / speed) * brakingDistance;
            predictedY += (velocity.getY() / speed) * brakingDistance;
        }

        double xPower = (endPose.getX() - predictedX) * coefficients.kP;
        double yPower = (endPose.getY() - predictedY) * coefficients.kP;
        double headingPower = headingController.updateError(
                Pose.normalizeRadians(endPose.getHeading() - currentPose.getHeading())
        );

        double maxTranslationPower = activeConstraints.maxFinalTranslationPower;
        double translationMagnitude = Math.hypot(xPower, yPower);

        if (translationMagnitude > maxTranslationPower) {
            double scale = maxTranslationPower / translationMagnitude;
            xPower *= scale;
            yPower *= scale;
        }

        return new Pose(
                Range.clip(xPower, -maxTranslationPower, maxTranslationPower),
                Range.clip(yPower, -maxTranslationPower, maxTranslationPower),
                Range.clip(headingPower, -activeConstraints.maxHeadingPower, activeConstraints.maxHeadingPower)
        );
    }

    private Pose calculateCentripetalCorrection(PathSegment segment, double t, Pose velocity) {
        if (!config.centripetalCorrectionEnabled || config.centripetalCorrectionScale == 0.0) {
            return new Pose();
        }

        double step = config.curvatureSampleStep <= 0.0 ? 0.01 : config.curvatureSampleStep;
        double previousT = Range.clip(t - step, 0.0, 1.0);
        double nextT = Range.clip(t + step, 0.0, 1.0);

        if (previousT == nextT) {
            return new Pose();
        }

        Pose previous = segment.get(previousT);
        Pose current = segment.get(t);
        Pose next = segment.get(nextT);
        double previousHeading = Math.atan2(current.getY() - previous.getY(), current.getX() - previous.getX());
        double nextHeading = Math.atan2(next.getY() - current.getY(), next.getX() - current.getX());
        double headingChange = Pose.normalizeRadians(nextHeading - previousHeading);
        double arcDistance = previous.distanceTo(current) + current.distanceTo(next);

        if (arcDistance <= 0.0) {
            return new Pose();
        }

        double curvature = headingChange / arcDistance;
        double speed = Math.hypot(velocity.getX(), velocity.getY());
        double correction = speed * speed * curvature * config.centripetalCorrectionScale;
        double tangentHeading = Math.atan2(next.getY() - previous.getY(), next.getX() - previous.getX());
        double normalHeading = tangentHeading + Math.copySign(Math.PI / 2.0, curvature);

        return new Pose(
                Math.cos(normalHeading) * Math.abs(correction),
                Math.sin(normalHeading) * Math.abs(correction),
                0.0
        );
    }

    private static Pose clampDriveCommand(Pose command, PathConstraints activeConstraints, boolean atEndSegment) {
        double maxTranslationPower = atEndSegment
                ? activeConstraints.maxFinalTranslationPower
                : activeConstraints.maxTranslationPower;
        double translationMagnitude = Math.hypot(command.getX(), command.getY());
        double xPower = command.getX();
        double yPower = command.getY();

        if (translationMagnitude > maxTranslationPower) {
            double scale = maxTranslationPower / translationMagnitude;
            xPower *= scale;
            yPower *= scale;
        }

        return new Pose(
                Range.clip(xPower, -maxTranslationPower, maxTranslationPower),
                Range.clip(yPower, -maxTranslationPower, maxTranslationPower),
                Range.clip(command.getHeading(), -activeConstraints.maxHeadingPower, activeConstraints.maxHeadingPower)
        );
    }

    private static double findClosestT(PathSegment segment, Pose pose, double startT) {
        double bestT = Range.clip(startT, 0.0, 1.0);
        double bestDistance = Double.POSITIVE_INFINITY;

        for (int i = 0; i <= CLOSEST_POINT_SAMPLES; i++) {
            double t = startT + ((1.0 - startT) * i / CLOSEST_POINT_SAMPLES);
            double distance = pose.distanceTo(segment.get(t));

            if (distance < bestDistance) {
                bestDistance = distance;
                bestT = t;
            }
        }

        return bestT;
    }

    private void configureControllers() {
        PIDFCoefficients translationCoefficients = config.getTranslationalPidf();
        PIDFCoefficients headingCoefficients = config.getTurnPidf();

        translationXController = new PIDFController(translationCoefficients);
        translationYController = new PIDFController(translationCoefficients);
        headingController = new PIDFController(headingCoefficients);

        translationXController.setIntegralLimit(config.translationIntegralLimit);
        translationYController.setIntegralLimit(config.translationIntegralLimit);
        headingController.setIntegralLimit(config.headingIntegralLimit);
        translationXController.setOutputBounds(-config.maxTranslationPower, config.maxTranslationPower);
        translationYController.setOutputBounds(-config.maxTranslationPower, config.maxTranslationPower);
        headingController.setOutputBounds(-config.maxHeadingPower, config.maxHeadingPower);
    }

    private void resetControllers() {
        translationXController.reset();
        translationYController.reset();
        headingController.reset();
    }

    private void updateHoldPoint() {
        targetPose = holdPose.copy();
        PathConstraints constraints = new PathConstraints(config);
        driveCommand = calculateDriveCommand(pose, holdPose, holdPose, true, constraints);
        driveCommand = clampDriveCommand(driveCommand, constraints, true);
        drivetrain.driveFieldCentric(driveCommand, pose, velocity, holdPose);
    }

    private PathConstraints getActiveConstraints() {
        if (path != null && path.getConstraints() != null) {
            return path.getConstraints();
        }

        return new PathConstraints(config);
    }

    private void updateCallbacks() {
        double elapsedSeconds = (System.nanoTime() - followStartTimeNanos) / 1_000_000_000.0;

        if (pathChain != null) {
            for (PathCallback callback : pathChain.getCallbacks()) {
                callback.update(pose, pathIndex, segmentIndex, segmentT, elapsedSeconds);
            }
        }

        if (path != null) {
            for (PathCallback callback : path.getCallbacks()) {
                callback.update(pose, pathIndex, segmentIndex, segmentT, elapsedSeconds);
            }
        }
    }
}
