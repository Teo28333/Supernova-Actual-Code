package org.firstinspires.ftc.teamcode.SupernovaPathPlanner;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.localization.OTOSLocalizer;
import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.localization.ThreeWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumDriveConfig;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.swervedrive.SwerveDriveConfig;

public class SupernovaConfig {
    public final LocalizerSetup localizer = new LocalizerSetup(this);
    public final DrivetrainSetup drivetrain = new DrivetrainSetup(this);
    public final PathFollowingSetup pathFollowing = new PathFollowingSetup(this);

    public String pinpointName = "pinpoint";
    public PinpointLocalizer.Config localizerConfig = new PinpointLocalizer.Config();
    public SwerveDriveConfig swerveDriveConfig;
    public MecanumDriveConfig mecanumDriveConfig;
    public FollowerConfig followerConfig = new FollowerConfig();

    private LocalizerFactory localizerFactory;
    private DrivetrainFactory drivetrainFactory;

    public SupernovaConfig() {
        usePinpoint(pinpointName, localizerConfig);
    }

    public SupernovaConfig(SwerveDriveConfig swerveDriveConfig) {
        this();
        useSwerveDrive(swerveDriveConfig);
    }

    public Localizer createLocalizer(HardwareMap hardwareMap) {
        if (localizerFactory == null) {
            usePinpoint(pinpointName, localizerConfig);
        }

        return localizerFactory.create(hardwareMap);
    }

    public Drivetrain createDrivetrain(HardwareMap hardwareMap, Localizer localizer) {
        if (drivetrainFactory == null) {
            if (swerveDriveConfig != null) {
                useSwerveDrive(swerveDriveConfig);
            } else if (mecanumDriveConfig != null) {
                useMecanumDrive(mecanumDriveConfig);
            } else {
                throw new IllegalStateException("Set a drivetrain with useSwerveDrive() or useMecanumDrive().");
            }
        }

        return drivetrainFactory.create(hardwareMap, localizer);
    }

    public SwerveDrive createSwerveDrive(HardwareMap hardwareMap, Localizer localizer) {
        if (swerveDriveConfig == null) {
            throw new IllegalStateException("Swerve drive config has not been set.");
        }

        return new SwerveDrive(hardwareMap, localizer, swerveDriveConfig);
    }

    public MecanumDrive createMecanumDrive(HardwareMap hardwareMap) {
        if (mecanumDriveConfig == null) {
            throw new IllegalStateException("Mecanum drive config has not been set.");
        }

        return new MecanumDrive(hardwareMap, mecanumDriveConfig);
    }

    public Follower createFollower(HardwareMap hardwareMap) {
        Localizer localizer = createLocalizer(hardwareMap);
        Drivetrain drivetrain = createDrivetrain(hardwareMap, localizer);
        return new Follower(drivetrain, localizer, followerConfig);
    }

    public PathBuilder pathBuilder() {
        return new PathBuilder();
    }

    public SupernovaConfig withLocalizer(LocalizerFactory localizerFactory) {
        this.localizerFactory = localizerFactory;
        return this;
    }

    public SupernovaConfig withDrivetrain(DrivetrainFactory drivetrainFactory) {
        this.drivetrainFactory = drivetrainFactory;
        return this;
    }

    public SupernovaConfig usePinpoint(String deviceName, PinpointLocalizer.Config config) {
        this.pinpointName = deviceName;
        this.localizerConfig = config;
        this.localizerFactory = hardwareMap -> {
            PinpointLocalizer localizer = new PinpointLocalizer(hardwareMap, deviceName);
            localizer.configure(config);
            return localizer;
        };
        return this;
    }

    public SupernovaConfig usePinpoint(PinpointLocalizer.Config config) {
        return usePinpoint(pinpointName, config);
    }

    public SupernovaConfig useOTOS(String deviceName, OTOSLocalizer.Config config) {
        this.localizerFactory = hardwareMap -> {
            OTOSLocalizer localizer = new OTOSLocalizer(hardwareMap, deviceName);
            localizer.configure(config);
            return localizer;
        };
        return this;
    }

    public SupernovaConfig useOtos(String deviceName, OTOSLocalizer.Config config) {
        return useOTOS(deviceName, config);
    }

    public SupernovaConfig useThreeWheelLocalizer(ThreeWheelLocalizer.Config config) {
        this.localizerFactory = hardwareMap -> new ThreeWheelLocalizer(hardwareMap, config);
        return this;
    }

    public SupernovaConfig useThreeWheel(ThreeWheelLocalizer.Config config) {
        return useThreeWheelLocalizer(config);
    }

    public SupernovaConfig useSwerveDrive(SwerveDriveConfig swerveDriveConfig) {
        this.swerveDriveConfig = swerveDriveConfig;
        this.drivetrainFactory = (hardwareMap, localizer) -> new SwerveDrive(hardwareMap, localizer, swerveDriveConfig);
        return this;
    }

    public SupernovaConfig useSwerve(SwerveDriveConfig swerveDriveConfig) {
        return useSwerveDrive(swerveDriveConfig);
    }

    public SupernovaConfig useMecanumDrive(MecanumDriveConfig mecanumDriveConfig) {
        this.mecanumDriveConfig = mecanumDriveConfig;
        this.drivetrainFactory = (hardwareMap, localizer) -> new MecanumDrive(hardwareMap, mecanumDriveConfig);
        return this;
    }

    public SupernovaConfig useMecanum(MecanumDriveConfig mecanumDriveConfig) {
        return useMecanumDrive(mecanumDriveConfig);
    }

    public SupernovaConfig withPinpointName(String pinpointName) {
        return usePinpoint(pinpointName, localizerConfig);
    }

    public SupernovaConfig withLocalizerConfig(PinpointLocalizer.Config localizerConfig) {
        return usePinpoint(pinpointName, localizerConfig);
    }

    public SupernovaConfig withSwerveDriveConfig(SwerveDriveConfig swerveDriveConfig) {
        return useSwerveDrive(swerveDriveConfig);
    }

    public SupernovaConfig withMecanumDriveConfig(MecanumDriveConfig mecanumDriveConfig) {
        return useMecanumDrive(mecanumDriveConfig);
    }

    public SupernovaConfig withFollowerConfig(FollowerConfig followerConfig) {
        this.followerConfig = followerConfig;
        return this;
    }

    public SupernovaConfig withPathFollowingConfig(FollowerConfig followerConfig) {
        return withFollowerConfig(followerConfig);
    }

    public FollowerConfig getPathFollowingConfig() {
        return followerConfig;
    }

    public SupernovaConfig withPathPidf(double kP, double kI, double kD, double kF) {
        followerConfig.withPathPidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withPrimaryPidf(double kP, double kI, double kD, double kF) {
        followerConfig.withPrimaryPidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withDrivePidf(double kP, double kI, double kD, double kF) {
        followerConfig.withDrivePidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withTranslationalPidf(double kP, double kI, double kD, double kF) {
        followerConfig.withTranslationalPidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withTurnPidf(double kP, double kI, double kD, double kF) {
        followerConfig.withTurnPidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withSecondaryPidf(double kP, double kI, double kD, double kF) {
        followerConfig.withSecondaryPidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withSecondaryDrivePidf(double kP, double kI, double kD, double kF) {
        followerConfig.withSecondaryDrivePidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withSecondaryTranslationalPidf(double kP, double kI, double kD, double kF) {
        followerConfig.withSecondaryTranslationalPidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withSecondaryTurnPidf(double kP, double kI, double kD, double kF) {
        followerConfig.withSecondaryTurnPidf(kP, kI, kD, kF);
        return this;
    }

    public SupernovaConfig withMaxPowers(double maxTranslationPower, double maxFinalTranslationPower,
                                         double maxHeadingPower) {
        followerConfig
                .withMaxTranslationPower(maxTranslationPower)
                .withMaxFinalTranslationPower(maxFinalTranslationPower)
                .withMaxHeadingPower(maxHeadingPower);
        return this;
    }

    public SupernovaConfig withLookaheadDistance(double lookaheadDistance) {
        followerConfig.withLookaheadDistance(lookaheadDistance);
        return this;
    }

    public SupernovaConfig withTolerances(double positionTolerance, double headingTolerance) {
        followerConfig
                .withPositionTolerance(positionTolerance)
                .withHeadingTolerance(headingTolerance);
        return this;
    }

    public SupernovaConfig withCentripetalCorrection(boolean enabled, double scale) {
        followerConfig
                .withCentripetalCorrection(enabled)
                .withCentripetalCorrectionScale(scale);
        return this;
    }

    public SupernovaConfig withPredictiveBraking(boolean enabled, double kP, double kLinear, double kQuadratic) {
        followerConfig
                .withPredictiveBraking(enabled)
                .withPredictiveBrakingCoefficients(kP, kLinear, kQuadratic);
        return this;
    }

    public interface LocalizerFactory {
        Localizer create(HardwareMap hardwareMap);
    }

    public interface DrivetrainFactory {
        Drivetrain create(HardwareMap hardwareMap, Localizer localizer);
    }

    public static class LocalizerSetup {
        private final SupernovaConfig owner;

        private LocalizerSetup(SupernovaConfig owner) {
            this.owner = owner;
        }

        public LocalizerSetup usePinpoint(String deviceName, PinpointLocalizer.Config config) {
            owner.usePinpoint(deviceName, config);
            return this;
        }

        public LocalizerSetup usePinpoint(PinpointLocalizer.Config config) {
            owner.usePinpoint(config);
            return this;
        }

        public LocalizerSetup useOTOS(String deviceName, OTOSLocalizer.Config config) {
            owner.useOTOS(deviceName, config);
            return this;
        }

        public LocalizerSetup useOtos(String deviceName, OTOSLocalizer.Config config) {
            owner.useOTOS(deviceName, config);
            return this;
        }

        public LocalizerSetup useThreeWheel(ThreeWheelLocalizer.Config config) {
            owner.useThreeWheel(config);
            return this;
        }

        public LocalizerSetup useCustom(LocalizerFactory localizerFactory) {
            owner.withLocalizer(localizerFactory);
            return this;
        }
    }

    public static class DrivetrainSetup {
        private final SupernovaConfig owner;

        private DrivetrainSetup(SupernovaConfig owner) {
            this.owner = owner;
        }

        public DrivetrainSetup useSwerve(SwerveDriveConfig swerveDriveConfig) {
            owner.useSwerve(swerveDriveConfig);
            return this;
        }

        public DrivetrainSetup useMecanum(MecanumDriveConfig mecanumDriveConfig) {
            owner.useMecanum(mecanumDriveConfig);
            return this;
        }

        public DrivetrainSetup useCustom(DrivetrainFactory drivetrainFactory) {
            owner.withDrivetrain(drivetrainFactory);
            return this;
        }
    }

    public static class PathFollowingSetup {
        private final SupernovaConfig owner;

        private PathFollowingSetup(SupernovaConfig owner) {
            this.owner = owner;
        }

        public PathFollowingSetup setConfig(FollowerConfig followerConfig) {
            owner.withFollowerConfig(followerConfig);
            return this;
        }

        public PathFollowingSetup withDrivePidf(double kP, double kI, double kD, double kF) {
            owner.withDrivePidf(kP, kI, kD, kF);
            return this;
        }

        public PathFollowingSetup withTranslationalPidf(double kP, double kI, double kD, double kF) {
            owner.withTranslationalPidf(kP, kI, kD, kF);
            return this;
        }

        public PathFollowingSetup withTurnPidf(double kP, double kI, double kD, double kF) {
            owner.withTurnPidf(kP, kI, kD, kF);
            return this;
        }

        public PathFollowingSetup withSecondaryDrivePidf(double kP, double kI, double kD, double kF) {
            owner.withSecondaryDrivePidf(kP, kI, kD, kF);
            return this;
        }

        public PathFollowingSetup withSecondaryTranslationalPidf(double kP, double kI, double kD, double kF) {
            owner.withSecondaryTranslationalPidf(kP, kI, kD, kF);
            return this;
        }

        public PathFollowingSetup withSecondaryTurnPidf(double kP, double kI, double kD, double kF) {
            owner.withSecondaryTurnPidf(kP, kI, kD, kF);
            return this;
        }

        public PathFollowingSetup withMaxPowers(double maxTranslationPower, double maxFinalTranslationPower,
                                                double maxHeadingPower) {
            owner.withMaxPowers(maxTranslationPower, maxFinalTranslationPower, maxHeadingPower);
            return this;
        }

        public PathFollowingSetup withLookaheadDistance(double lookaheadDistance) {
            owner.withLookaheadDistance(lookaheadDistance);
            return this;
        }

        public PathFollowingSetup withTolerances(double positionTolerance, double headingTolerance) {
            owner.withTolerances(positionTolerance, headingTolerance);
            return this;
        }

        public PathFollowingSetup withCentripetalCorrection(boolean enabled, double scale) {
            owner.withCentripetalCorrection(enabled, scale);
            return this;
        }

        public PathFollowingSetup withPredictiveBraking(boolean enabled,
                                                        double kP,
                                                        double kLinear,
                                                        double kQuadratic) {
            owner.withPredictiveBraking(enabled, kP, kLinear, kQuadratic);
            return this;
        }
    }
}
