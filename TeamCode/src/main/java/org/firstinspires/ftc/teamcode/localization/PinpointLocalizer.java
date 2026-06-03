package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pinpoint;
    private Pose rawPose = new Pose();
    private Pose pose = new Pose();
    private Pose velocity = new Pose();
    private Pose previousPose = new Pose();
    private long lastUpdateTimeNanos = 0L;
    private boolean hasUpdated = false;

    public PinpointLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, "pinpoint");
    }

    public PinpointLocalizer(HardwareMap hardwareMap, String deviceName) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
    }

    public void configure(Config config) {
        pinpoint.setOffsets(config.xPodOffset, config.yPodOffset, config.offsetDistanceUnit);

        if (config.usingGoBildaPods) {
            pinpoint.setEncoderResolution(config.goBildaPodType);
        } else {
            pinpoint.setEncoderResolution(config.ticksPerUnit, config.encoderDistanceUnit);
        }

        pinpoint.setEncoderDirections(config.xEncoderDirection, config.yEncoderDirection);

        if (config.yawScalar != null) {
            pinpoint.setYawScalar(config.yawScalar);
        }

        if (config.resetPositionAndImu) {
            pinpoint.resetPosAndIMU();
        } else if (config.recalibrateImu) {
            pinpoint.recalibrateIMU();
        }

        setPose(config.startPose);
    }

    @Override
    public void update() {
        pinpoint.update();

        previousPose = pose.copy();
        long now = System.nanoTime();

        Pose2D pinpointPose = pinpoint.getPosition();
        rawPose = new Pose(
                pinpointPose.getX(DistanceUnit.INCH),
                pinpointPose.getY(DistanceUnit.INCH),
                pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)
        );
        pose = rawPose.copy();

        velocity = new Pose(
                pinpoint.getVelX(DistanceUnit.INCH),
                pinpoint.getVelY(DistanceUnit.INCH),
                pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)
        );

        if (hasUpdated && isZeroVelocity(velocity)) {
            double dt = (now - lastUpdateTimeNanos) / 1_000_000_000.0;
            if (dt > 0.0) {
                velocity = new Pose(
                        (pose.getX() - previousPose.getX()) / dt,
                        (pose.getY() - previousPose.getY()) / dt,
                        Pose.normalizeRadians(pose.getHeading() - previousPose.getHeading()) / dt
                );
            }
        }

        hasUpdated = true;
        lastUpdateTimeNanos = now;
    }

    @Override
    public Pose getPose() {
        update();
        return pose.copy();
    }

    public Pose getPoseEstimate() {
        return getPose();
    }

    public Pose getRawPose() {
        return rawPose.copy();
    }

    public double getHeading() {
        return pose.getHeading();
    }

    public double getNormalizedHeading() {
        return Pose.normalizeRadians(pose.getHeading());
    }

    public double getTotalHeading() {
        return pose.getHeading();
    }

    @Override
    public Pose getVelocity() {
        return velocity.copy();
    }

    public Pose getPoseVelocity() {
        return getVelocity();
    }

    public Pose getFieldVelocity() {
        return getVelocity();
    }

    public Pose getRobotVelocity() {
        return velocity.rotated(-pose.getHeading()).withHeading(velocity.getHeading());
    }

    public double getXVelocity() {
        return velocity.getX();
    }

    public double getYVelocity() {
        return velocity.getY();
    }

    public double getHeadingVelocity() {
        return velocity.getHeading();
    }

    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return pinpoint.getDeviceStatus();
    }

    public boolean isReady() {
        return getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    public boolean hasFault() {
        GoBildaPinpointDriver.DeviceStatus status = getDeviceStatus();
        return status == GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
    }

    public int getLoopTime() {
        return pinpoint.getLoopTime();
    }

    public double getFrequency() {
        return pinpoint.getFrequency();
    }

    public boolean hasUpdated() {
        return hasUpdated;
    }

    public double getSecondsSinceLastUpdate() {
        if (!hasUpdated) {
            return Double.POSITIVE_INFINITY;
        }

        return (System.nanoTime() - lastUpdateTimeNanos) / 1_000_000_000.0;
    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose.copy();
        this.rawPose = pose.copy();
        this.previousPose = pose.copy();
        this.velocity = new Pose();
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH,
                pose.getX(),
                pose.getY(),
                AngleUnit.RADIANS,
                pose.getHeading()
        ));
    }

    public void setPoseEstimate(Pose pose) {
        setPose(pose);
    }

    public void setHeading(double heading) {
        setPose(pose.withHeading(heading));
    }

    public void setHeadingDegrees(double headingDegrees) {
        setHeading(Math.toRadians(headingDegrees));
    }

    public void resetHeading() {
        setHeading(0.0);
    }

    public void resetPosition() {
        setPose(new Pose());
    }

    public void resetPoseAndImu() {
        pinpoint.resetPosAndIMU();
        pose = new Pose();
        rawPose = new Pose();
        previousPose = new Pose();
        velocity = new Pose();
        hasUpdated = false;
    }

    public void recalibrateImu() {
        pinpoint.recalibrateIMU();
    }

    public Pose fieldToRobotVelocity(double fieldXVelocity, double fieldYVelocity, double headingVelocity) {
        return new Pose(fieldXVelocity, fieldYVelocity, headingVelocity).rotated(-pose.getHeading())
                .withHeading(headingVelocity);
    }

    public Pose robotToFieldVelocity(double robotXVelocity, double robotYVelocity, double headingVelocity) {
        return new Pose(robotXVelocity, robotYVelocity, headingVelocity).rotated(pose.getHeading())
                .withHeading(headingVelocity);
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    private static boolean isZeroVelocity(Pose velocity) {
        return velocity.getX() == 0.0 && velocity.getY() == 0.0 && velocity.getHeading() == 0.0;
    }

    public static class Config {
        public double xPodOffset = 0.0;
        public double yPodOffset = 0.0;
        public DistanceUnit offsetDistanceUnit = DistanceUnit.INCH;
        public boolean usingGoBildaPods = true;
        public GoBildaPinpointDriver.GoBildaOdometryPods goBildaPodType =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        public double ticksPerUnit = 1.0;
        public DistanceUnit encoderDistanceUnit = DistanceUnit.INCH;
        public GoBildaPinpointDriver.EncoderDirection xEncoderDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public GoBildaPinpointDriver.EncoderDirection yEncoderDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        public boolean resetPositionAndImu = true;
        public boolean recalibrateImu = false;
        public Double yawScalar = null;
        public Pose startPose = new Pose();

        public Config withPodOffsets(double xPodOffset, double yPodOffset) {
            this.xPodOffset = xPodOffset;
            this.yPodOffset = yPodOffset;
            return this;
        }

        public Config withPodOffsets(double xPodOffset, double yPodOffset, DistanceUnit distanceUnit) {
            this.xPodOffset = xPodOffset;
            this.yPodOffset = yPodOffset;
            this.offsetDistanceUnit = distanceUnit;
            return this;
        }

        public Config withGoBildaPodType(GoBildaPinpointDriver.GoBildaOdometryPods goBildaPodType) {
            this.usingGoBildaPods = true;
            this.goBildaPodType = goBildaPodType;
            return this;
        }

        public Config withEncoderResolution(double ticksPerUnit, DistanceUnit encoderDistanceUnit) {
            this.usingGoBildaPods = false;
            this.ticksPerUnit = ticksPerUnit;
            this.encoderDistanceUnit = encoderDistanceUnit;
            return this;
        }

        public Config withEncoderDirections(GoBildaPinpointDriver.EncoderDirection xEncoderDirection,
                                            GoBildaPinpointDriver.EncoderDirection yEncoderDirection) {
            this.xEncoderDirection = xEncoderDirection;
            this.yEncoderDirection = yEncoderDirection;
            return this;
        }

        public Config withYawScalar(double yawScalar) {
            this.yawScalar = yawScalar;
            return this;
        }

        public Config withStartPose(Pose startPose) {
            this.startPose = startPose;
            return this;
        }

        public Config withResetPositionAndImu(boolean resetPositionAndImu) {
            this.resetPositionAndImu = resetPositionAndImu;
            return this;
        }

        public Config withRecalibrateImu(boolean recalibrateImu) {
            this.recalibrateImu = recalibrateImu;
            return this;
        }
    }
}
