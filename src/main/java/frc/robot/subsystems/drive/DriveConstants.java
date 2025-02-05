package frc.robot.subsystems.drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotBase;

public class DriveConstants {

    public static final double kTrackwidthXMeters = 0.6;
    public static final double kTrackwidthYMeters = 0.6;

    public static final double kDrivebaseRadius =  Math.hypot(kTrackwidthXMeters / 2.0, kTrackwidthYMeters / 2.0);;

    public static final int kGyroPort = 10;

    // TODO: Might need to change this to a lower value in pp and here
    public static final double kMaxLinearSpeed = 4.5;
    public static final double kMaxLinearAcceleration = 7.7;
    public static final double kMaxRotationalSpeedRadians = Math.toRadians(360.0);
    public static final double kMaxRotationalAccelerationRadians = Math.toRadians(360) * 10;
    public static final double kMaxAzimuthAngularRadiansPS = Math.toRadians(660.0);

    public static final double kAzimuthGearRatio = 150.0 / 7.0;
    public static final double kDriveGearRatio = 6.12 / 1.0;
    public static final double kWheelRadiusMeters = 5.08 / 100.0;
    public static final double kWheelCircumferenceMeters = 2 * Math.PI * kWheelRadiusMeters;

    public static final Translation2d[] kModuleTranslations = new Translation2d[] {
        new Translation2d(kTrackwidthXMeters / 2.0, kTrackwidthYMeters / 2.0),
        new Translation2d(kTrackwidthXMeters / 2.0, -kTrackwidthYMeters / 2.0),
        new Translation2d(-kTrackwidthXMeters / 2.0, kTrackwidthYMeters / 2.0),
        new Translation2d(-kTrackwidthXMeters / 2.0, -kTrackwidthYMeters / 2.0)
      };
    
    public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(kModuleTranslations);

    // TODO: Needs to be tuned //
    public static final double kDriftRate = RobotBase.isReal() ? 1 : 3.0;

    public static final SwerveModuleHardwareConfig kFrontLeft = new SwerveModuleHardwareConfig(
      "FrontLeft", 
      11, 
      21, 
      31, 
      Rotation2d.fromRotations(0.416016));
    
    public static final SwerveModuleHardwareConfig kFrontRight = new SwerveModuleHardwareConfig(
      "FrontRight", 
      12, 
      22,
      34,
      Rotation2d.fromRotations(0.2732));
   
    public static final SwerveModuleHardwareConfig kBackLeft = new SwerveModuleHardwareConfig(
      "BackLeft", 
      23, 
      13,
      33, 
      Rotation2d.fromRotations(-0.4204));
    
    public static final SwerveModuleHardwareConfig kBackRight = new SwerveModuleHardwareConfig(
      "BackRight",
      14, 
      24,
      32, 
      Rotation2d.fromRotations(-0.0935));

    public static final boolean kInvertAzimuths = true;

    public record SwerveModuleHardwareConfig(String name, int drivePort, int azimuthPort, int cancoderPort, Rotation2d offset) {}

    public static record ModuleControlConfig(
        PIDController driveController,
        SimpleMotorFeedforward driveFF,
        PIDController azimuthController,
        SimpleMotorFeedforward azimuthFF) {}


    // First set of ModuleControlConfig tell us the PID and FF gains in real life and the second set gives the sim ones //
    // The drive kS and kV can be found using a system indentication test, kA shoulud only be added for FOC //
    // kP is simply tuned and kD is not neccessary usually //
    // Important note: Update these values after each time you tune with LoggedTuneableNumbers to make sure that its the new default value //
    public static final ModuleControlConfig kModuleControllerConfigs = RobotBase.isReal() ? 
        new ModuleControlConfig(
            new PIDController(1.0, 0.0, 0.0), new SimpleMotorFeedforward(0.164, 2.33, 0.0),
            new PIDController(25.0, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0, 0.0)) :
        new ModuleControlConfig(
            new PIDController(2.0, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 2.1, 0.0), 
            new PIDController(12.0, 0.0, 0.0), new SimpleMotorFeedforward(0.0, 0.0, 0.0));
    
}