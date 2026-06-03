package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

public class OTOSLocalizer implements Localizer {
    private final SparkFunOTOS otos;
    private Pose pose = new Pose();
    private Pose velocity = new Pose();

    public OTOSLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, "sensor_otos");
    }

    public OTOSLocalizer(HardwareMap hardwareMap, String deviceName) {
        otos = hardwareMap.get(SparkFunOTOS.class, deviceName);
    }

    public void configure(Config config) {
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setOffset(new SparkFunOTOS.Pose2D(
                config.offsetPose.getX(),
                config.offsetPose.getY(),
                config.offsetPose.getHeading()
        ));
        otos.setLinearScalar(config.linearScalar);
        otos.setAngularScalar(config.angularScalar);

        if (config.calibrateImu) {
            otos.calibrateImu(config.imuCalibrationSamples, config.waitForImuCalibration);
        }

        if (config.resetTracking) {
            otos.resetTracking();
        }

        setPose(config.startPose);
    }

    @Override
    public void update() {
        SparkFunOTOS.Pose2D position = otos.getPosition();
        SparkFunOTOS.Pose2D poseVelocity = otos.getVelocity();

        pose = new Pose(position.x, position.y, position.h);
        velocity = new Pose(poseVelocity.x, poseVelocity.y, poseVelocity.h);
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
        otos.setPosition(new SparkFunOTOS.Pose2D(pose.getX(), pose.getY(), pose.getHeading()));
    }

    @Override
    public void setStartingPose(Pose pose) {
        setPose(pose);
    }

    public SparkFunOTOS getOtos() {
        return otos;
    }

    public static class Config {
        public Pose offsetPose = new Pose();
        public Pose startPose = new Pose();
        public double linearScalar = 1.0;
        public double angularScalar = 1.0;
        public boolean calibrateImu = true;
        public int imuCalibrationSamples = 255;
        public boolean waitForImuCalibration = true;
        public boolean resetTracking = true;

        public Config withOffsetPose(Pose offsetPose) {
            this.offsetPose = offsetPose;
            return this;
        }

        public Config withStartPose(Pose startPose) {
            this.startPose = startPose;
            return this;
        }

        public Config withScalars(double linearScalar, double angularScalar) {
            this.linearScalar = linearScalar;
            this.angularScalar = angularScalar;
            return this;
        }

        public Config withImuCalibration(boolean calibrateImu) {
            this.calibrateImu = calibrateImu;
            return this;
        }

        public Config withResetTracking(boolean resetTracking) {
            this.resetTracking = resetTracking;
            return this;
        }
    }
}
