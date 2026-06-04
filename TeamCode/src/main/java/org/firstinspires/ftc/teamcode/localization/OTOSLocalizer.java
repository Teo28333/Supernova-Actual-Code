package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Localizer base sur SparkFun OTOS avec configuration d'unites, offsets et calibration IMU.
 */
public class OTOSLocalizer implements Localizer {
    private final SparkFunOTOS otos;
    private Pose pose = new Pose();
    private Pose velocity = new Pose();

    // Construit cette classe avec les dependances et reglages necessaires.
    public OTOSLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, "sensor_otos");
    }

    // Construit cette classe avec les dependances et reglages necessaires.
    public OTOSLocalizer(HardwareMap hardwareMap, String deviceName) {
        otos = hardwareMap.get(SparkFunOTOS.class, deviceName);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
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

    // Rafraichit les lectures, calcule les erreurs et met a jour l'etat interne.
    @Override
    public void update() {
        SparkFunOTOS.Pose2D position = otos.getPosition();
        SparkFunOTOS.Pose2D poseVelocity = otos.getVelocity();

        pose = new Pose(position.x, position.y, position.h);
        velocity = new Pose(poseVelocity.x, poseVelocity.y, poseVelocity.h);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    @Override
    public Pose getPose() {
        update();
        return pose.copy();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    @Override
    public Pose getPoseEstimate() {
        return pose.copy();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    @Override
    public Pose getVelocity() {
        return velocity.copy();
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    @Override
    public void setPose(Pose pose) {
        this.pose = pose.copy();
        this.velocity = new Pose();
        otos.setPosition(new SparkFunOTOS.Pose2D(pose.getX(), pose.getY(), pose.getHeading()));
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    @Override
    public void setStartingPose(Pose pose) {
        setPose(pose);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
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

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withOffsetPose(Pose offsetPose) {
            this.offsetPose = offsetPose;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withStartPose(Pose startPose) {
            this.startPose = startPose;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withScalars(double linearScalar, double angularScalar) {
            this.linearScalar = linearScalar;
            this.angularScalar = angularScalar;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withImuCalibration(boolean calibrateImu) {
            this.calibrateImu = calibrateImu;
            return this;
        }

        // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
        public Config withResetTracking(boolean resetTracking) {
            this.resetTracking = resetTracking;
            return this;
        }
    }
}
