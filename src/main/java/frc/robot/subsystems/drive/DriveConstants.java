package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveConstants {
    ///////////////////// DRIVE BASE \\\\\\\\\\\\\\\\\\\\\\\
    /* PHYSICAL CONSTANTS */
    public static final double kRobotWidthMeters = 0.9144;
    public static final double kTrackWidthXMeters = 0.6;
    public static final double kTrackWidthYMeters = 0.6;
    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0),
        new Translation2d(-kTrackWidthXMeters / 2.0, -kTrackWidthYMeters / 2.0)
      };
    public static final SwerveDriveKinematics kKinematics =
        new SwerveDriveKinematics(kModuleTranslations);

    public static final double kDrivebaseRadiusMeters =
        Math.hypot(kTrackWidthXMeters / 2.0, kTrackWidthYMeters / 2.0);
    
    /* DRIVEBASE CONSTRAINTS */
    public static final double kMaxLinearSpeedMPS = 4.5;
    public static final double kMaxLinearAccelerationMPSS = 12.0;

    public static final double kMaxRotationSpeedRadiansPS = Math.toRadians(360);
    public static final double kMaxRotationAccelRadiansPS = Math.toRadians(360) * 10;

    public static final double kMaxAzimuthAngularRadiansPS = Math.toRadians(1200);

    /* Plugged into setpoint generator */
    public static final PathConstraints kDriveConstraints = new PathConstraints(
        kMaxLinearSpeedMPS, kMaxLinearAccelerationMPSS, 
        kMaxRotationSpeedRadiansPS, kMaxRotationAccelRadiansPS);

    /* MISC */
    public static final double kDriftRate = RobotBase.isReal() ? 2.5 : 5.57;
    public static final double kSniperSpeed = 0.2;

    public static final boolean kDoExtraLogging = false;

    public static final PIDConstants kPPTranslationPID = new PIDConstants(5.00, 0.0, 0.0);
    public static final PIDConstants kPPRotationPID = new PIDConstants(5.00, 0.0, 0.0);

    ///////////////////// MODULES \\\\\\\\\\\\\\\\\\\\\\\
    /* GENERAL SWERVE MODULE CONSTANTS */
    public static final boolean kTurnMotorInvert = true;
    public static final double kAzimuthGearing = 150.0 / 7.0;
    public static final double kDriveGearing = 6.12 / 1.0;
    public static final double kRadiusMeters = 5.08 / 100.0;
    public static final double kCircumferenceMeters = 2 * Math.PI * kRadiusMeters;

    public static final ModuleControlConfig kModuleControllerConfigs = RobotBase.isReal() ? 
        new ModuleControlConfig(
            new PIDController(100.0, 0.0, 0.0), new SimpleMotorFeedforward(0, 0.0, 0.0),
            new PIDController(20.0, 0.0, 0.5), new SimpleMotorFeedforward(0.0, 0.0, 0.0)) :
        new ModuleControlConfig(
            new PIDController(0.1, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 2.36, 0.005), 
            new PIDController(4.5, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0));

    /* MODULE SPECIFIC CONSTANTS */
    public static final ModuleHardwareConfig kFrontLeftHardware =
        new ModuleHardwareConfig(
            11, 
            21, 
            31, 
            Rotation2d.fromRotations(0.083496).plus(Rotation2d.k180deg));

    public static final ModuleHardwareConfig kFrontRightHardware =
        new ModuleHardwareConfig(
            12, 
            22,
            32,
            Rotation2d.fromRotations(-0.461182).plus(Rotation2d.k180deg));
            
    public static final ModuleHardwareConfig kBackLeftHardware =
        new ModuleHardwareConfig(
            13, 
            23,
            33, 
            Rotation2d.fromRotations(0.322754).plus(Rotation2d.k180deg));

    public static final ModuleHardwareConfig kBackRightHardware =
        new ModuleHardwareConfig(
            14, 
            24,
            34,
            Rotation2d.fromRotations(0.073242).plus(Rotation2d.k180deg));

    public static record ModuleHardwareConfig(
        int driveID, int azimuthID, int encoderID, Rotation2d offset) {}

    public static record ModuleControlConfig(
        PIDController driveController,
        SimpleMotorFeedforward driveFF,
        PIDController azimuthController,
        SimpleMotorFeedforward azimuthFF) {}
}