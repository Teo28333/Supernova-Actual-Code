package org.firstinspires.ftc.teamcode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.FollowerConfig;
import org.firstinspires.ftc.teamcode.SupernovaPathPlanner.Pose;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.localization.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumDriveConfig;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveDriveConfig;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveModuleConfig;

/**
 * Documentation ajoutee: Configuration Dashboard centrale pour choisir le mode de tuning, le drivetrain et les gains.
 */
@Config
public class SwerveAutonTuningConfig {
    public static final double FIELD_MIN_X = 0.0;
    public static final double FIELD_MIN_Y = 0.0;
    public static final double FIELD_MAX_X = 144.0;
    public static final double FIELD_MAX_Y = 144.0;
    public static final double FIELD_CENTER_X = 72.0;
    public static final double FIELD_CENTER_Y = 72.0;

    public enum TuningMode {
        JOYSTICK,
        POD_ANGLE,
        X_LOCK,
        AUTON_HOLD
    }

    public enum DrivetrainType {
        SWERVE,
        MECANUM
    }

    public enum LocalizerType {
        PINPOINT,
        THREE_WHEEL,
        OTOS
    }

    public static TuningMode MODE = TuningMode.JOYSTICK;
    public static DrivetrainType DRIVETRAIN = DrivetrainType.SWERVE;
    public static LocalizerType LOCALIZER = LocalizerType.PINPOINT;
    public static double DRIVE_SCALE = 0.5;
    public static int SELECTED_POD = -1;
    public static double TARGET_POD_ANGLE_DEGREES = 45.0;
    public static double STARTING_X = FIELD_CENTER_X;
    public static double STARTING_Y = FIELD_CENTER_Y;
    public static double STARTING_HEADING_DEGREES = 0.0;
    public static double TARGET_X = FIELD_CENTER_X;
    public static double TARGET_Y = FIELD_CENTER_Y;
    public static double TARGET_HEADING_DEGREES = 0.0;

    public static String PINPOINT_NAME = "pinpoint";
    public static String OTOS_NAME = "sensor_otos";

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

    public static String MECANUM_FRONT_LEFT = "frontLeft";
    public static String MECANUM_FRONT_RIGHT = "frontRight";
    public static String MECANUM_BACK_LEFT = "backLeft";
    public static String MECANUM_BACK_RIGHT = "backRight";

    public static String THREE_WHEEL_LEFT_ENCODER = "leftEncoder";
    public static String THREE_WHEEL_RIGHT_ENCODER = "rightEncoder";
    public static String THREE_WHEEL_STRAFE_ENCODER = "strafeEncoder";

    public static double ANALOG_MIN_VOLTAGE = 0.0;
    public static double ANALOG_MAX_VOLTAGE = 3.3;
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

    public static boolean MECANUM_FRONT_LEFT_REVERSED = false;
    public static boolean MECANUM_FRONT_RIGHT_REVERSED = true;
    public static boolean MECANUM_BACK_LEFT_REVERSED = false;
    public static boolean MECANUM_BACK_RIGHT_REVERSED = true;

    public static double POD_KP = 1.8;
    public static double POD_KI = 0.0;
    public static double POD_KD = 0.05;
    public static double POD_KF = 0.0;
    public static double POD_TOLERANCE_DEGREES = 2.0;
    public static double POD_INTEGRAL_LIMIT = 0.5;

    public static double PINPOINT_X_OFFSET_INCHES = -3.31;
    public static double PINPOINT_Y_OFFSET_INCHES = -6.61;
    public static boolean PINPOINT_RESET_ON_INIT = true;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_X_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static GoBildaPinpointDriver.EncoderDirection PINPOINT_Y_ENCODER_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static double LOCALIZER_YAW_SCALAR = 1.0;
    public static boolean USE_LOCALIZER_YAW_SCALAR = false;

    public static double THREE_WHEEL_TICKS_PER_INCH = 8192.0 / (Math.PI * 2.0);
    public static double THREE_WHEEL_TRACK_WIDTH = 12.0;
    public static double THREE_WHEEL_STRAFE_OFFSET = 0.0;
    public static boolean THREE_WHEEL_LEFT_REVERSED = false;
    public static boolean THREE_WHEEL_RIGHT_REVERSED = false;
    public static boolean THREE_WHEEL_STRAFE_REVERSED = false;

    public static double OTOS_OFFSET_X = 0.0;
    public static double OTOS_OFFSET_Y = 0.0;
    public static double OTOS_OFFSET_HEADING_DEGREES = 0.0;
    public static double OTOS_LINEAR_SCALAR = 1.0;
    public static double OTOS_ANGULAR_SCALAR = 1.0;
    public static boolean OTOS_CALIBRATE_IMU = true;
    public static boolean OTOS_RESET_TRACKING = true;

    public static boolean X_LOCK_ENABLED = true;
    public static double X_LOCK_DEADBAND = 0.03;
    public static boolean BRAKE_MODE_ENABLED = true;
    public static boolean PREDICTIVE_BRAKING_ENABLED = true;
    public static double PREDICTIVE_BRAKING_KP = 0.1;
    public static double PREDICTIVE_BRAKING_K_LINEAR = 0.04;
    public static double PREDICTIVE_BRAKING_K_QUADRATIC = 0.0016;
    public static double MAX_LINEAR_DECELERATION = 40.0;
    public static double MAX_ANGULAR_DECELERATION_DEGREES = 360.0;
    public static double BRAKING_DISTANCE_BUFFER = 2.0;
    public static double MINIMUM_BRAKE_SCALE = 0.12;
    public static boolean CENTRIPETAL_CORRECTION_ENABLED = true;
    public static double CENTRIPETAL_CORRECTION_SCALE = 0.0;

    public static double TRANSLATION_KP = 0.045;
    public static double TRANSLATION_KI = 0.0;
    public static double TRANSLATION_KD = 0.0;
    public static double TRANSLATION_KF = 0.0;
    public static double TRANSLATION_INTEGRAL_LIMIT = 4.0;
    public static double HEADING_KP = 1.6;
    public static double HEADING_KI = 0.0;
    public static double HEADING_KD = 0.0;
    public static double HEADING_KF = 0.0;
    public static double HEADING_INTEGRAL_LIMIT = 0.5;
    public static double MAX_TRANSLATION_POWER = 0.85;
    public static double MAX_FINAL_TRANSLATION_POWER = 0.35;
    public static double MAX_HEADING_POWER = 0.75;
    public static double LOOKAHEAD_DISTANCE = 8.0;
    public static double FINAL_APPROACH_DISTANCE = 12.0;
    public static double POSITION_TOLERANCE = 1.0;
    public static double HEADING_TOLERANCE_DEGREES = 3.0;

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static PinpointLocalizer.Config pinpoint() {
        return new PinpointLocalizer.Config()
                .withPodOffsets(PINPOINT_X_OFFSET_INCHES, PINPOINT_Y_OFFSET_INCHES, DistanceUnit.INCH)
                .withGoBildaPodType(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                .withEncoderDirections(
                        PINPOINT_X_ENCODER_DIRECTION,
                        PINPOINT_Y_ENCODER_DIRECTION
                )
                .withStartPose(startingPose())
                .withResetPositionAndImu(PINPOINT_RESET_ON_INIT)
                .withYawScalar(USE_LOCALIZER_YAW_SCALAR ? LOCALIZER_YAW_SCALAR : 1.0);
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static ThreeWheelLocalizer.Config threeWheel() {
        return new ThreeWheelLocalizer.Config()
                .withEncoderNames(THREE_WHEEL_LEFT_ENCODER, THREE_WHEEL_RIGHT_ENCODER, THREE_WHEEL_STRAFE_ENCODER)
                .withTicksPerInch(THREE_WHEEL_TICKS_PER_INCH)
                .withGeometry(THREE_WHEEL_TRACK_WIDTH, THREE_WHEEL_STRAFE_OFFSET)
                .withMultipliers(
                        THREE_WHEEL_LEFT_REVERSED ? -1.0 : 1.0,
                        THREE_WHEEL_RIGHT_REVERSED ? -1.0 : 1.0,
                        THREE_WHEEL_STRAFE_REVERSED ? -1.0 : 1.0
                )
                .withStartPose(startingPose());
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static OTOSLocalizer.Config otos() {
        return new OTOSLocalizer.Config()
                .withOffsetPose(new Pose(
                        OTOS_OFFSET_X,
                        OTOS_OFFSET_Y,
                        Math.toRadians(OTOS_OFFSET_HEADING_DEGREES)
                ))
                .withStartPose(startingPose())
                .withScalars(OTOS_LINEAR_SCALAR, USE_LOCALIZER_YAW_SCALAR ? LOCALIZER_YAW_SCALAR : OTOS_ANGULAR_SCALAR)
                .withImuCalibration(OTOS_CALIBRATE_IMU)
                .withResetTracking(OTOS_RESET_TRACKING);
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    public static Pose startingPose() {
        return new Pose(STARTING_X, STARTING_Y, Math.toRadians(STARTING_HEADING_DEGREES));
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static SwerveDriveConfig swerveDrive() {
        double halfTrack = TRACK_WIDTH / 2.0;
        double halfWheelBase = WHEEL_BASE / 2.0;

        return new SwerveDriveConfig(
                module(FRONT_LEFT_DRIVE, FRONT_LEFT_TURN, FRONT_LEFT_ENCODER,
                        new Pose(halfTrack, halfWheelBase, 0.0), FRONT_LEFT_OFFSET),
                module(FRONT_RIGHT_DRIVE, FRONT_RIGHT_TURN, FRONT_RIGHT_ENCODER,
                        new Pose(-halfTrack, halfWheelBase, 0.0), FRONT_RIGHT_OFFSET),
                module(BACK_LEFT_DRIVE, BACK_LEFT_TURN, BACK_LEFT_ENCODER,
                        new Pose(halfTrack, -halfWheelBase, 0.0), BACK_LEFT_OFFSET),
                module(BACK_RIGHT_DRIVE, BACK_RIGHT_TURN, BACK_RIGHT_ENCODER,
                        new Pose(-halfTrack, -halfWheelBase, 0.0), BACK_RIGHT_OFFSET)
        )
                .withXLock(X_LOCK_ENABLED)
                .withXLockDeadband(X_LOCK_DEADBAND)
                .withCentripetalCorrection(CENTRIPETAL_CORRECTION_ENABLED)
                .withCentripetalCorrectionScale(CENTRIPETAL_CORRECTION_SCALE)
                .withBrakeMode(BRAKE_MODE_ENABLED);
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static MecanumDriveConfig mecanumDrive() {
        return new MecanumDriveConfig()
                .withMotorNames(MECANUM_FRONT_LEFT, MECANUM_FRONT_RIGHT, MECANUM_BACK_LEFT, MECANUM_BACK_RIGHT)
                .withMotorDirections(
                        motorDirection(MECANUM_FRONT_LEFT_REVERSED),
                        motorDirection(MECANUM_FRONT_RIGHT_REVERSED),
                        motorDirection(MECANUM_BACK_LEFT_REVERSED),
                        motorDirection(MECANUM_BACK_RIGHT_REVERSED)
                )
                .withBrakeMode(BRAKE_MODE_ENABLED)
                .withPredictiveBraking(false)
                .withMaxLinearDeceleration(MAX_LINEAR_DECELERATION)
                .withMaxAngularDeceleration(Math.toRadians(MAX_ANGULAR_DECELERATION_DEGREES))
                .withBrakingDistanceBuffer(BRAKING_DISTANCE_BUFFER)
                .withMinimumBrakeScale(MINIMUM_BRAKE_SCALE)
                .withXLock(X_LOCK_ENABLED)
                .withXLockDeadband(X_LOCK_DEADBAND);
    }

    // Fabrique l'objet configure qui sera utilise par l'OpMode ou le follower.
    public static FollowerConfig follower() {
        return new FollowerConfig()
                .withTranslationPidf(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD, TRANSLATION_KF)
                .withTranslationIntegralLimit(TRANSLATION_INTEGRAL_LIMIT)
                .withHeadingPidf(HEADING_KP, HEADING_KI, HEADING_KD, HEADING_KF)
                .withHeadingIntegralLimit(HEADING_INTEGRAL_LIMIT)
                .withMaxTranslationPower(MAX_TRANSLATION_POWER)
                .withMaxFinalTranslationPower(MAX_FINAL_TRANSLATION_POWER)
                .withMaxHeadingPower(MAX_HEADING_POWER)
                .withLookaheadDistance(LOOKAHEAD_DISTANCE)
                .withFinalApproachDistance(FINAL_APPROACH_DISTANCE)
                .withPositionTolerance(POSITION_TOLERANCE)
                .withHeadingTolerance(Math.toRadians(HEADING_TOLERANCE_DEGREES))
                .withCentripetalCorrection(CENTRIPETAL_CORRECTION_ENABLED)
                .withCentripetalCorrectionScale(CENTRIPETAL_CORRECTION_SCALE)
                .withPredictiveBraking(PREDICTIVE_BRAKING_ENABLED)
                .withPredictiveBrakingCoefficients(
                        PREDICTIVE_BRAKING_KP,
                        PREDICTIVE_BRAKING_K_LINEAR,
                        PREDICTIVE_BRAKING_K_QUADRATIC
                );
    }

    private static SwerveModuleConfig module(String driveMotorName, String turnServoName,
                                             String absoluteEncoderName, Pose modulePose,
                                             double angleOffset) {
        return new SwerveModuleConfig(driveMotorName, turnServoName, absoluteEncoderName, modulePose)
                .withVoltageRange(ANALOG_MIN_VOLTAGE, ANALOG_MAX_VOLTAGE)
                .withAngleOffset(angleOffset)
                .withTurnPidf(POD_KP, POD_KI, POD_KD, POD_KF)
                .withTurnToleranceRadians(Math.toRadians(POD_TOLERANCE_DEGREES))
                .withTurnIntegralLimit(POD_INTEGRAL_LIMIT)
                .withDriveReversed(driveReversedFor(driveMotorName))
                .withTurnReversed(turnReversedFor(turnServoName));
    }

    // Convertit une intention de mouvement en commandes concretes pour le drivetrain.
    private static boolean driveReversedFor(String driveMotorName) {
        if (FRONT_LEFT_DRIVE.equals(driveMotorName)) {
            return FRONT_LEFT_DRIVE_REVERSED;
        } else if (FRONT_RIGHT_DRIVE.equals(driveMotorName)) {
            return FRONT_RIGHT_DRIVE_REVERSED;
        } else if (BACK_LEFT_DRIVE.equals(driveMotorName)) {
            return BACK_LEFT_DRIVE_REVERSED;
        } else if (BACK_RIGHT_DRIVE.equals(driveMotorName)) {
            return BACK_RIGHT_DRIVE_REVERSED;
        }

        return false;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    private static boolean turnReversedFor(String turnServoName) {
        if (FRONT_LEFT_TURN.equals(turnServoName)) {
            return FRONT_LEFT_TURN_REVERSED;
        } else if (FRONT_RIGHT_TURN.equals(turnServoName)) {
            return FRONT_RIGHT_TURN_REVERSED;
        } else if (BACK_LEFT_TURN.equals(turnServoName)) {
            return BACK_LEFT_TURN_REVERSED;
        } else if (BACK_RIGHT_TURN.equals(turnServoName)) {
            return BACK_RIGHT_TURN_REVERSED;
        }

        return false;
    }

    // Section de logique dediee a cette responsabilite precise de la classe.
    private static DcMotorSimple.Direction motorDirection(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }
}
