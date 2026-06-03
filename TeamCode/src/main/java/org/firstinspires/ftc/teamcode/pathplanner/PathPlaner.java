package org.firstinspires.ftc.teamcode.pathplanner;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveDrive;

public class PathPlaner {
    private static final int CLOSEST_POINT_SAMPLES = 80;

    public Pose pose = new Pose();

    private final SwerveDrive drive;
    private final Localizer localizer;
    private Config config = new Config();
    private PedroPath path;
    private int segmentIndex = 0;
    private double segmentT = 0.0;
    private Pose targetPose = new Pose();
    private Pose driveCommand = new Pose();
    private boolean busy = false;

    public PathPlaner(SwerveDrive drive) {
        this.drive = drive;
        this.localizer = drive.getLocalizer();
    }

    public PathPlaner(SwerveDrive drive, Config config) {
        this(drive);
        this.config = config;
    }

    public void setConfig(Config config) {
        this.config = config;
    }

    public void followPath(PedroPath path) {
        followPath(path, false);
    }

    public void followPath(PedroPath path, boolean resetPose) {
        if (path == null || path.size() == 0) {
            throw new IllegalArgumentException("Path must contain at least one segment.");
        }

        this.path = path;
        this.segmentIndex = 0;
        this.segmentT = 0.0;
        this.targetPose = path.get(0).get(0.0);
        this.driveCommand = new Pose();
        this.busy = true;

        if (resetPose) {
            localizer.setPose(path.getStartPose());
        }
    }

    public void update() {
        localizer.update();
        pose = localizer.getPose();

        if (!busy || path == null) {
            drive.stop();
            return;
        }

        PathSegment segment = path.get(segmentIndex);
        segmentT = Math.max(segmentT, findClosestT(segment, pose, segmentT));

        while (segmentT >= 1.0 - config.segmentAdvanceTolerance && segmentIndex < path.size() - 1) {
            segmentIndex++;
            segment = path.get(segmentIndex);
            segmentT = 0.0;
        }

        double targetT = Range.clip(segmentT + config.lookaheadDistance / Math.max(segment.length(), 1.0), 0.0, 1.0);
        targetPose = segment.get(targetT);

        Pose endPose = path.getEndPose();
        double distanceToEnd = pose.distanceTo(endPose);
        double headingErrorToEnd = Math.abs(pose.headingErrorTo(endPose));
        boolean atEndSegment = segmentIndex == path.size() - 1;

        if (atEndSegment && distanceToEnd <= config.positionTolerance && headingErrorToEnd <= config.headingTolerance) {
            busy = false;
            driveCommand = new Pose();
            drive.stop();
            return;
        }

        driveCommand = calculateDriveCommand(pose, targetPose, endPose, atEndSegment);
        drive.driveFieldCentric(driveCommand.getY(), driveCommand.getX(), driveCommand.getHeading());
    }

    public void stop() {
        busy = false;
        driveCommand = new Pose();
        drive.stop();
    }

    public boolean isBusy() {
        return busy;
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

    public double getSegmentT() {
        return segmentT;
    }

    private Pose calculateDriveCommand(Pose currentPose, Pose lookaheadPose, Pose endPose, boolean atEndSegment) {
        Pose trackingPose = atEndSegment && currentPose.distanceTo(endPose) < config.finalApproachDistance
                ? endPose
                : lookaheadPose;

        double xPower = (trackingPose.getX() - currentPose.getX()) * config.translationKp;
        double yPower = (trackingPose.getY() - currentPose.getY()) * config.translationKp;
        double headingPower = Pose.normalizeRadians(trackingPose.getHeading() - currentPose.getHeading()) * config.headingKp;

        double translationMagnitude = Math.hypot(xPower, yPower);
        double maxTranslationPower = atEndSegment ? config.maxFinalTranslationPower : config.maxTranslationPower;

        if (translationMagnitude > maxTranslationPower) {
            double scale = maxTranslationPower / translationMagnitude;
            xPower *= scale;
            yPower *= scale;
        }

        return new Pose(
                Range.clip(xPower, -maxTranslationPower, maxTranslationPower),
                Range.clip(yPower, -maxTranslationPower, maxTranslationPower),
                Range.clip(headingPower, -config.maxHeadingPower, config.maxHeadingPower)
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

    public static class Config {
        public double translationKp = 0.045;
        public double headingKp = 1.6;
        public double maxTranslationPower = 0.85;
        public double maxFinalTranslationPower = 0.35;
        public double maxHeadingPower = 0.75;
        public double lookaheadDistance = 8.0;
        public double finalApproachDistance = 12.0;
        public double positionTolerance = 1.0;
        public double headingTolerance = Math.toRadians(3.0);
        public double segmentAdvanceTolerance = 0.02;

        public Config withTranslationKp(double translationKp) {
            this.translationKp = translationKp;
            return this;
        }

        public Config withHeadingKp(double headingKp) {
            this.headingKp = headingKp;
            return this;
        }

        public Config withMaxTranslationPower(double maxTranslationPower) {
            this.maxTranslationPower = maxTranslationPower;
            return this;
        }

        public Config withMaxFinalTranslationPower(double maxFinalTranslationPower) {
            this.maxFinalTranslationPower = maxFinalTranslationPower;
            return this;
        }

        public Config withMaxHeadingPower(double maxHeadingPower) {
            this.maxHeadingPower = maxHeadingPower;
            return this;
        }

        public Config withLookaheadDistance(double lookaheadDistance) {
            this.lookaheadDistance = lookaheadDistance;
            return this;
        }

        public Config withFinalApproachDistance(double finalApproachDistance) {
            this.finalApproachDistance = finalApproachDistance;
            return this;
        }

        public Config withPositionTolerance(double positionTolerance) {
            this.positionTolerance = positionTolerance;
            return this;
        }

        public Config withHeadingTolerance(double headingTolerance) {
            this.headingTolerance = headingTolerance;
            return this;
        }
    }
}
