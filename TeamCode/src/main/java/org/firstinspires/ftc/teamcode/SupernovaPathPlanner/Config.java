package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.localization.Encoder;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveDriveConfig;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveModuleConfig;

/**
 * Documentation ajoutee: Configuration rapide du robot Supernova: Pinpoint, swerve et follower pour creer les objets principaux.
 */
@com.acmerobotics.dashboard.config.Config
public class Config {
    public static String PINPOINT_NAME = "pinpoint";
    public static double PINPOINT_X_OFFSET_INCHES = -3.31;
    public static double PINPOINT_Y_OFFSET_INCHES = -6.61;
    public static double PINPOINT_X_ENCODER_DIRECTION = Encoder.FORWARD;
    public static double PINPOINT_Y_ENCODER_DIRECTION = Encoder.FORWARD;
    public static double PINPOINT_YAW_SCALAR = 1.0;
    public static boolean USE_PINPOINT_YAW_SCALAR = false;

    public static double TRACK_WIDTH = 13.0;
    public static double WHEEL_BASE = 13.0;

    public static String FRONT_LEFT_DRIVE = "frontLeftDrive";
    public static String FRONT_LEFT_TURN = "frontLeftTurn";
    public static String FRONT_LEFT_ENCODER = "frontLeftEncoder";
    public static String FRONT_RIGHT_DRIVE = "frontRightDrive";
    public static String FRONT_RIGHT_TURN = "frontRightTurn";
    public static String FRONT_RIGHT_ENCODER = "frontRightEncoder";
    public static String BACK_LEFT_DRIVE = "backLeftDrive";
    public static String BACK_LEFT_TURN = "backLeftTurn";
    public static String BACK_LEFT_ENCODER = "backLeftEncoder";
    public static String BACK_RIGHT_DRIVE = "backRightDrive";
    public static String BACK_RIGHT_TURN = "backRightTurn";
    public static String BACK_RIGHT_ENCODER = "backRightEncoder";

    public static double FRONT_LEFT_OFFSET = 0.0;
    public static double FRONT_RIGHT_OFFSET = 0.0;
    public static double BACK_LEFT_OFFSET = 0.0;
    public static double BACK_RIGHT_OFFSET = 0.0;

    public static boolean FRONT_LEFT_DRIVE_REVERSED = false;
    public static boolean FRONT_RIGHT_DRIVE_REVERSED = false;
    public static boolean BACK_LEFT_DRIVE_REVERSED = false;
    public static boolean BACK_RIGHT_DRIVE_REVERSED = false;
    public static boolean FRONT_LEFT_TURN_REVERSED = false;
    public static boolean FRONT_RIGHT_TURN_REVERSED = false;
    public static boolean BACK_LEFT_TURN_REVERSED = false;
    public static boolean BACK_RIGHT_TURN_REVERSED = false;

    public static double POD_KP = 1.8;
    public static double POD_KI = 0.0;
    public static double POD_KD = 0.05;
    public static double POD_KF = 0.0;

    public static boolean X_LOCK_ENABLED = true;
    public static boolean BRAKE_MODE_ENABLED = true;

    public static double TRANSLATIONAL_KP = 0.045;
    public static double TRANSLATIONAL_KI = 0.0;
    public static double TRANSLATIONAL_KD = 0.0;
    public static double TRANSLATIONAL_KF = 0.0;

    public static double TURN_KP = 1.6;
    public static double TURN_KI = 0.0;
    public static double TURN_KD = 0.0;
    public static double TURN_KF = 0.0;

    public static double DRIVING_MAX_OUTPUT = 0.85;
    public static double TURNING_MAX_OUTPUT = 0.75;
    public static double LOOKAHEAD_DISTANCE = 8.0;
    public static double POSITION_TOLERANCE = 1.0;
    public static double HEADING_TOLERANCE_DEGREES = 3.0;

    public static boolean PREDICTIVE_BRAKING_ENABLED = false;
    public static double PREDICTIVE_BRAKING_KP = 0.1;
    public static double PREDICTIVE_BRAKING_K_LINEAR = 0.04;
    public static double PREDICTIVE_BRAKING_K_QUADRATIC = 0.0016;

    public static boolean CENTRIPETAL_CORRECTION_ENABLED = true;
    public static double CENTRIPETAL_CORRECTION_SCALE = 0.0;

    // Construit cette classe avec les dependances et reglages necessaires.
    private Config() {
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static SupernovaConfig robot() {
        return new SupernovaConfig()
                .usePinpoint(PINPOINT_NAME, pinpoint())
                .useSwerve(coaxialSwerve())
                .withFollowerConfig(follower());
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static PinpointLocalizer.Config pinpoint() {
        PinpointLocalizer.Config config = new PinpointLocalizer.Config()
                .withPodOffsets(PINPOINT_X_OFFSET_INCHES, PINPOINT_Y_OFFSET_INCHES, DistanceUnit.INCH)
                .withGoBildaPodType(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .withEncoderDirections(
                        Encoder.toPinpointDirection(PINPOINT_X_ENCODER_DIRECTION),
                        Encoder.toPinpointDirection(PINPOINT_Y_ENCODER_DIRECTION)
                )
                .withResetPositionAndImu(true);

        if (USE_PINPOINT_YAW_SCALAR) {
            config.withYawScalar(PINPOINT_YAW_SCALAR);
        }

        return config;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public static SwerveDriveConfig coaxialSwerve() {
        double halfTrack = TRACK_WIDTH / 2.0;
        double halfWheelBase = WHEEL_BASE / 2.0;

        return new SwerveDriveConfig(
                module(FRONT_LEFT_DRIVE, FRONT_LEFT_TURN, FRONT_LEFT_ENCODER,
                        new Pose(halfTrack, halfWheelBase, 0.0), FRONT_LEFT_OFFSET,
                        FRONT_LEFT_DRIVE_REVERSED, FRONT_LEFT_TURN_REVERSED),
                module(FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURN, FRONT_RIGHT_ENCODER,
                        new Pose(-halfTrack, halfWheelBase, 0.0), FRONT_RIGHT_OFFSET,
                        FRONT_RIGHT_DRIVE_REVERSED, FRONT_RIGHT_TURN_REVERSED),
                module(BACK_LEFT_DRIVE, BACK_LEFT_TURN, BACK_LEFT_ENCODER,
                        new Pose(halfTrack, -halfWheelBase, 0.0), BACK_LEFT_OFFSET,
                        BACK_LEFT_DRIVE_REVERSED, BACK_LEFT_TURN_REVERSED),
                module(BACK_RIGHT_DRIVE, BACK_RIGHT_TURN, BACK_RIGHT_ENCODER,
                        new Pose(-halfTrack, -halfWheelBase, 0.0), BACK_RIGHT_OFFSET,
                        BACK_RIGHT_DRIVE_REVERSED, BACK_RIGHT_TURN_REVERSED)
        )
                .withXLock(X_LOCK_ENABLED)
                .withXLockDeadband(0.03)
                .withCentripetalCorrection(CENTRIPETAL_CORRECTION_ENABLED)
                .withCentripetalCorrectionScale(CENTRIPETAL_CORRECTION_SCALE)
                .withBrakeMode(BRAKE_MODE_ENABLED);
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static FollowerConfig follower() {
        return new FollowerConfig()
                .withDrivePidf(TRANSLATIONAL_KP, TRANSLATIONAL_KI, TRANSLATIONAL_KD, TRANSLATIONAL_KF)
                .withTranslationalPidf(TRANSLATIONAL_KP, TRANSLATIONAL_KI, TRANSLATIONAL_KD, TRANSLATIONAL_KF)
                .withTranslationIntegralLimit(4.0)
                .withTurnPidf(TURN_KP, TURN_KI, TURN_KD, TURN_KF)
                .withHeadingIntegralLimit(0.5)
                .withSecondaryDrivePidf(0.0, 0.0, 0.0, 0.0)
                .withSecondaryTranslationalPidf(0.0, 0.0, 0.0, 0.0)
                .withSecondaryTurnPidf(0.0, 0.0, 0.0, 0.0)
                .withMaxTranslationPower(DRIVING_MAX_OUTPUT)
                .withMaxFinalTranslationPower(DRIVING_MAX_OUTPUT)
                .withMaxHeadingPower(TURNING_MAX_OUTPUT)
                .withLookaheadDistance(LOOKAHEAD_DISTANCE)
                .withFinalApproachDistance(12.0)
                .withPositionTolerance(POSITION_TOLERANCE)
                .withHeadingTolerance(Math.toRadians(HEADING_TOLERANCE_DEGREES))
                .withSegmentAdvanceTolerance(0.02)
                .withPredictiveBraking(PREDICTIVE_BRAKING_ENABLED)
                .withPredictiveBrakingCoefficients(
                        PREDICTIVE_BRAKING_KP,
                        PREDICTIVE_BRAKING_K_LINEAR,
                        PREDICTIVE_BRAKING_K_QUADRATIC
                )
                .withCentripetalCorrection(CENTRIPETAL_CORRECTION_ENABLED)
                .withCentripetalCorrectionScale(CENTRIPETAL_CORRECTION_SCALE)
                .withCurvatureSampleStep(0.01);
    }

    private static SwerveModuleConfig module(String driveMotorName, String turnServoName,
                                             String absoluteEncoderName, Pose modulePose,
                                             double angleOffset, boolean driveReversed,
                                             boolean turnReversed) {
        return new SwerveModuleConfig(driveMotorName, turnServoName, absoluteEncoderName, modulePose)
                .withVoltageRange(0.0, 3.3)
                .withAngleOffset(angleOffset)
                .withTurnPidf(POD_KP, POD_KI, POD_KD, POD_KF)
                .withTurnToleranceRadians(Math.toRadians(2.0))
                .withTurnIntegralLimit(0.5)
                .withDriveReversed(driveReversed)
                .withTurnReversed(turnReversed);
    }
}
