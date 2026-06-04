package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Localizer base sur le capteur goBILDA Pinpoint avec pose, vitesse, statut et recalibrage.
 */
public class PinpointLocalizer implements Localizer {
    private final GoBildaPinpointDriver pinpoint;
    private Pose rawPose = new Pose();
    private Pose pose = new Pose();
    private Pose velocity = new Pose();
    private Pose previousPose = new Pose();
    private long lastUpdateTimeNanos = 0L;
    private boolean hasUpdated = false;

    // Construit cette classe avec les dependances et reglages necessaires.
    public PinpointLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, "pinpoint");
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public PinpointLocalizer(HardwareMap hardwareMap, String deviceName) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
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

    // Rafraichit les lectures, calcule les erreurs et met a jour l'etat interne.
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

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    @Override
    public Pose getPose() {
        update();
        return pose.copy();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getPoseEstimate() {
        return pose.copy();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getRawPose() {
        return rawPose.copy();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getHeading() {
        return pose.getHeading();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getNormalizedHeading() {
        return Pose.normalizeRadians(pose.getHeading());
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getTotalHeading() {
        return pose.getHeading();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    @Override
    public Pose getVelocity() {
        return velocity.copy();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getPoseVelocity() {
        return getVelocity();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getFieldVelocity() {
        return getVelocity();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getRobotVelocity() {
        return velocity.rotated(-pose.getHeading()).withHeading(velocity.getHeading());
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getXVelocity() {
        return velocity.getX();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getYVelocity() {
        return velocity.getY();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getHeadingVelocity() {
        return velocity.getHeading();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public GoBildaPinpointDriver.DeviceStatus getDeviceStatus() {
        return pinpoint.getDeviceStatus();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public boolean isReady() {
        return getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public boolean hasFault() {
        GoBildaPinpointDriver.DeviceStatus status = getDeviceStatus();
        return status == GoBildaPinpointDriver.DeviceStatus.FAULT_X_POD_NOT_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_Y_POD_NOT_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_NO_PODS_DETECTED
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_IMU_RUNAWAY
                || status == GoBildaPinpointDriver.DeviceStatus.FAULT_BAD_READ;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public int getLoopTime() {
        return pinpoint.getLoopTime();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getFrequency() {
        return pinpoint.getFrequency();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public boolean hasUpdated() {
        return hasUpdated;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getSecondsSinceLastUpdate() {
        if (!hasUpdated) {
            return Double.POSITIVE_INFINITY;
        }

        return (System.nanoTime() - lastUpdateTimeNanos) / 1_000_000_000.0;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
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

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setPoseEstimate(Pose pose) {
        setPose(pose);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setStartingPose(Pose pose) {
        setPose(pose);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setHeading(double heading) {
        setPose(pose.withHeading(heading));
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setHeadingDegrees(double headingDegrees) {
        setHeading(Math.toRadians(headingDegrees));
    }

    // Remet l'etat interne a une base propre pour eviter les anciennes erreurs accumulees.
    public void resetHeading() {
        setHeading(0.0);
    }

    // Remet l'etat interne a une base propre pour eviter les anciennes erreurs accumulees.
    public void resetPosition() {
        setPose(new Pose());
    }

    // Remet l'etat interne a une base propre pour eviter les anciennes erreurs accumulees.
    public void resetPoseAndImu() {
        pinpoint.resetPosAndIMU();
        pose = new Pose();
        rawPose = new Pose();
        previousPose = new Pose();
        velocity = new Pose();
        hasUpdated = false;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public void recalibrateImu() {
        pinpoint.recalibrateIMU();
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public Pose fieldToRobotVelocity(double fieldXVelocity, double fieldYVelocity, double headingVelocity) {
        return new Pose(fieldXVelocity, fieldYVelocity, headingVelocity).rotated(-pose.getHeading())
                .withHeading(headingVelocity);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public Pose robotToFieldVelocity(double robotXVelocity, double robotYVelocity, double headingVelocity) {
        return new Pose(robotXVelocity, robotYVelocity, headingVelocity).rotated(pose.getHeading())
                .withHeading(headingVelocity);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
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

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withPodOffsets(double xPodOffset, double yPodOffset) {
            this.xPodOffset = xPodOffset;
            this.yPodOffset = yPodOffset;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withPodOffsets(double xPodOffset, double yPodOffset, DistanceUnit distanceUnit) {
            this.xPodOffset = xPodOffset;
            this.yPodOffset = yPodOffset;
            this.offsetDistanceUnit = distanceUnit;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withGoBildaPodType(GoBildaPinpointDriver.GoBildaOdometryPods goBildaPodType) {
            this.usingGoBildaPods = true;
            this.goBildaPodType = goBildaPodType;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
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

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withYawScalar(double yawScalar) {
            this.yawScalar = yawScalar;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withStartPose(Pose startPose) {
            this.startPose = startPose;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withResetPositionAndImu(boolean resetPositionAndImu) {
            this.resetPositionAndImu = resetPositionAndImu;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withRecalibrateImu(boolean recalibrateImu) {
            this.recalibrateImu = recalibrateImu;
            return this;
        }
    }
}
