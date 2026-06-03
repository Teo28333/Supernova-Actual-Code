package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

public class ThreeWheelLocalizer implements Localizer {
    private final DcMotorEx leftEncoder;
    private final DcMotorEx rightEncoder;
    private final DcMotorEx strafeEncoder;
    private Config config;
    private Pose pose = new Pose();
    private Pose velocity = new Pose();
    private int lastLeftTicks;
    private int lastRightTicks;
    private int lastStrafeTicks;
    private long lastUpdateTimeNanos;
    private boolean hasLastUpdate = false;

    public ThreeWheelLocalizer(HardwareMap hardwareMap, Config config) {
        this(
                hardwareMap.get(DcMotorEx.class, config.leftEncoderName),
                hardwareMap.get(DcMotorEx.class, config.rightEncoderName),
                hardwareMap.get(DcMotorEx.class, config.strafeEncoderName),
                config
        );
    }

    public ThreeWheelLocalizer(DcMotorEx leftEncoder, DcMotorEx rightEncoder,
                               DcMotorEx strafeEncoder, Config config) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.strafeEncoder = strafeEncoder;
        setConfig(config);
        setPose(config.startPose);
    }

    @Override
    public void update() {
        int leftTicks = leftEncoder.getCurrentPosition();
        int rightTicks = rightEncoder.getCurrentPosition();
        int strafeTicks = strafeEncoder.getCurrentPosition();
        long now = System.nanoTime();

        if (!hasLastUpdate) {
            resetLastReadings(leftTicks, rightTicks, strafeTicks, now);
            return;
        }

        double leftDelta = ticksToInches(leftTicks - lastLeftTicks) * config.leftMultiplier;
        double rightDelta = ticksToInches(rightTicks - lastRightTicks) * config.rightMultiplier;
        double strafeDelta = ticksToInches(strafeTicks - lastStrafeTicks) * config.strafeMultiplier;
        double headingDelta = (rightDelta - leftDelta) / config.trackWidth;
        double forwardDelta = (leftDelta + rightDelta) * 0.5;
        double correctedStrafeDelta = strafeDelta - (config.strafeWheelOffset * headingDelta);
        double midHeading = pose.getHeading() + (headingDelta * 0.5);

        double fieldXDelta = correctedStrafeDelta * Math.cos(midHeading)
                - forwardDelta * Math.sin(midHeading);
        double fieldYDelta = correctedStrafeDelta * Math.sin(midHeading)
                + forwardDelta * Math.cos(midHeading);

        Pose previousPose = pose.copy();
        pose = new Pose(
                pose.getX() + fieldXDelta,
                pose.getY() + fieldYDelta,
                Pose.normalizeRadians(pose.getHeading() + headingDelta)
        );

        double dt = (now - lastUpdateTimeNanos) / 1_000_000_000.0;
        if (dt > 0.0) {
            velocity = new Pose(
                    (pose.getX() - previousPose.getX()) / dt,
                    (pose.getY() - previousPose.getY()) / dt,
                    Pose.normalizeRadians(pose.getHeading() - previousPose.getHeading()) / dt
            );
        }

        resetLastReadings(leftTicks, rightTicks, strafeTicks, now);
    }

    @Override
    public Pose getPose() {
        update();
        return pose.copy();
    }

    @Override
    public Pose getPoseEstimate() {
        return pose.copy();
    }

    @Override
    public Pose getVelocity() {
        return velocity.copy();
    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose.copy();
        this.velocity = new Pose();
        resetLastReadings(
                leftEncoder.getCurrentPosition(),
                rightEncoder.getCurrentPosition(),
                strafeEncoder.getCurrentPosition(),
                System.nanoTime()
        );
    }

    @Override
    public void setStartingPose(Pose pose) {
        setPose(pose);
    }

    public void setConfig(Config config) {
        this.config = config;
    }

    public Config getConfig() {
        return config;
    }

    private double ticksToInches(int ticks) {
        return ticks / config.ticksPerInch;
    }

    private void resetLastReadings(int leftTicks, int rightTicks, int strafeTicks, long now) {
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastStrafeTicks = strafeTicks;
        lastUpdateTimeNanos = now;
        hasLastUpdate = true;
    }

    public static class Config {
        public String leftEncoderName = "leftEncoder";
        public String rightEncoderName = "rightEncoder";
        public String strafeEncoderName = "strafeEncoder";
        public double ticksPerInch = 8192.0 / (Math.PI * 2.0);
        public double trackWidth = 12.0;
        public double strafeWheelOffset = 0.0;
        public double leftMultiplier = 1.0;
        public double rightMultiplier = 1.0;
        public double strafeMultiplier = 1.0;
        public Pose startPose = new Pose();

        public Config withEncoderNames(String leftEncoderName, String rightEncoderName, String strafeEncoderName) {
            this.leftEncoderName = leftEncoderName;
            this.rightEncoderName = rightEncoderName;
            this.strafeEncoderName = strafeEncoderName;
            return this;
        }

        public Config withTicksPerInch(double ticksPerInch) {
            this.ticksPerInch = ticksPerInch;
            return this;
        }

        public Config withGeometry(double trackWidth, double strafeWheelOffset) {
            this.trackWidth = trackWidth;
            this.strafeWheelOffset = strafeWheelOffset;
            return this;
        }

        public Config withMultipliers(double leftMultiplier, double rightMultiplier, double strafeMultiplier) {
            this.leftMultiplier = leftMultiplier;
            this.rightMultiplier = rightMultiplier;
            this.strafeMultiplier = strafeMultiplier;
            return this;
        }

        public Config withEncoderDirections(double leftDirection, double rightDirection, double strafeDirection) {
            return withMultipliers(leftDirection, rightDirection, strafeDirection);
        }

        public Config withStartPose(Pose startPose) {
            this.startPose = startPose;
            return this;
        }
    }
}
