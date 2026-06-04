package org.firstinspires.ftc.teamcode.subsystems.swervedrive;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;

/**
 * Documentation ajoutee: Controle un module swerve individuel: moteur de traction, servo de rotation et encodeur absolu.
 */
public class SwervePods {
    private static final double TWO_PI = 2.0 * Math.PI;
    private static final double DEFAULT_TURN_KP = 1.8;
    private static final double DEFAULT_TURN_KI = 0.0;
    private static final double DEFAULT_TURN_KD = 0.0;
    private static final double DEFAULT_TURN_KF = 0.0;
    private static final double DEFAULT_TURN_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final double DEFAULT_INTEGRAL_LIMIT = 0.5;

    private final DcMotorEx driveMotor;
    private final CRServo turnMotor;
    private final AnalogInput analogEncoder;
    private double minVoltage;
    private double maxVoltage;
    private double angleOffset = 0.0;
    private double podPoseX = 0.0;
    private double podPoseY = 0.0;
    private double turnKp = DEFAULT_TURN_KP;
    private double turnKi = DEFAULT_TURN_KI;
    private double turnKd = DEFAULT_TURN_KD;
    private double turnKf = DEFAULT_TURN_KF;
    private double turnToleranceRadians = DEFAULT_TURN_TOLERANCE_RADIANS;
    private double integralLimit = DEFAULT_INTEGRAL_LIMIT;
    private double integralSum = 0.0;
    private double lastAngleError = 0.0;
    private long lastPidTimeNanos = 0L;
    private double lastTargetAngle = 0.0;
    private double lastDrivePower = 0.0;
    private double lastTurnPower = 0.0;

    // Construit cette classe avec les dependances et reglages necessaires.
    public SwervePods(HardwareMap hwm, String motorName, String servoName, String encoderName,
                       double angleOffset, double minVoltage, double maxVoltage) {
        driveMotor = hwm.get(DcMotorEx.class, motorName);
        turnMotor = hwm.get(CRServo.class, servoName);
        analogEncoder = hwm.get(AnalogInput.class, encoderName);

        this.minVoltage = minVoltage;
        this.maxVoltage = maxVoltage;
        this.angleOffset = angleOffset;

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setPodPose(Pose pose) {
        podPoseX = pose.getX();
        podPoseY = pose.getY();
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public Pose getPodPose() {
        return new Pose(podPoseX, podPoseY, 0.0);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getPodPoseX() {
        return podPoseX;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getPodPoseY() {
        return podPoseY;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setTurnKp(double turnKp) {
        this.turnKp = turnKp;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setTurnPid(double kP, double kI, double kD) {
        setTurnPidf(kP, kI, kD, turnKf);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setTurnPidf(double kP, double kI, double kD, double kF) {
        this.turnKp = kP;
        this.turnKi = kI;
        this.turnKd = kD;
        this.turnKf = kF;
        resetTurnController();
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setTurnKf(double turnKf) {
        this.turnKf = turnKf;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setTurnToleranceRadians(double turnToleranceRadians) {
        this.turnToleranceRadians = Math.abs(turnToleranceRadians);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setIntegralLimit(double integralLimit) {
        this.integralLimit = Math.abs(integralLimit);
    }

    // Remet l'etat interne a une base propre pour eviter les anciennes erreurs accumulees.
    public void resetTurnController() {
        integralSum = 0.0;
        lastAngleError = 0.0;
        lastPidTimeNanos = 0L;
        lastTurnPower = 0.0;
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setDriveReversed(boolean reversed) {
        driveMotor.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setDriveZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        driveMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setTurnReversed(boolean reversed) {
        turnMotor.setDirection(reversed ? CRServo.Direction.REVERSE : CRServo.Direction.FORWARD);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getCurrentAngle() {
        double voltageRange = maxVoltage - minVoltage;
        if (voltageRange <= 0.0) {
            return 0.0;
        }

        double normalizedVoltage = Range.clip(
                (analogEncoder.getVoltage() - minVoltage) / voltageRange,
                0.0,
                1.0
        );

        return Pose.normalizeRadians((normalizedVoltage * TWO_PI) - angleOffset);
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getLastTargetAngle() {
        return lastTargetAngle;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getLastDrivePower() {
        return lastDrivePower;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getLastTurnPower() {
        return lastTurnPower;
    }

    // Donne l'etat courant sans exposer directement les objets internes modifiables.
    public double getLastAngleError() {
        return lastAngleError;
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    public void pointAt(double targetAngle) {
        setTargetState(0.0, targetAngle);
    }

    // Applique un reglage et renvoie/met a jour l'objet pour la configuration du robot.
    public void setTargetState(double drivePower, double targetAngle) {
        double currentAngle = getCurrentAngle();
        double optimizedDrivePower = drivePower;
        double optimizedTargetAngle = targetAngle;
        double angleError = Pose.normalizeRadians(optimizedTargetAngle - currentAngle);

        if (Math.abs(angleError) > Math.PI / 2.0) {
            optimizedDrivePower = -optimizedDrivePower;
            optimizedTargetAngle = Pose.normalizeRadians(optimizedTargetAngle + Math.PI);
            angleError = Pose.normalizeRadians(optimizedTargetAngle - currentAngle);
        }

        double turnPower = calculateTurnPower(angleError);

        lastTargetAngle = optimizedTargetAngle;
        lastDrivePower = Range.clip(optimizedDrivePower, -1.0, 1.0);
        lastTurnPower = turnPower;

        turnMotor.setPower(turnPower);
        driveMotor.setPower(lastDrivePower);
    }

    // Arrete proprement les moteurs et remet les commandes a zero.
    public void stop() {
        driveMotor.setPower(0.0);
        turnMotor.setPower(0.0);
        lastDrivePower = 0.0;
        resetTurnController();
    }

    // Calcule une valeur intermediaire utilisee pour limiter ou corriger la commande.
    private double calculateTurnPower(double angleError) {
        long now = System.nanoTime();

        if (Math.abs(angleError) <= turnToleranceRadians) {
            resetTurnController();
            lastAngleError = angleError;
            return 0.0;
        }

        double dt = lastPidTimeNanos == 0L
                ? 0.0
                : (now - lastPidTimeNanos) / 1_000_000_000.0;

        if (dt > 0.0) {
            integralSum += angleError * dt;
            integralSum = Range.clip(integralSum, -integralLimit, integralLimit);
        }

        double derivative = dt > 0.0 ? (angleError - lastAngleError) / dt : 0.0;
        double feedForward = turnKf == 0.0 ? 0.0 : Math.copySign(turnKf, angleError);
        double turnPower = (turnKp * angleError)
                + (turnKi * integralSum)
                + (turnKd * derivative)
                + feedForward;

        lastAngleError = angleError;
        lastPidTimeNanos = now;

        return Range.clip(turnPower, -1.0, 1.0);
    }

}
