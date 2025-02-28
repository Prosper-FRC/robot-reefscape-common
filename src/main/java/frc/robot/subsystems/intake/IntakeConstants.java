
package frc.robot.subsystems.intake;

import frc.robot.Constants;
import frc.robot.utils.visualizers.PivotVisualizer.PivotVisualizerConfiguration;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public record IntakeHardware(
    int motorId,
    double gearing,
    double wheelRadiusMeters) {}

  public record IntakeMotorConfiguration(
    boolean invert,
    boolean enableStatorCurrentLimit,
    boolean enableSupplyCurrentLimit,
    double statorCurrentLimitAmps,
    double supplyCurrentLimitAmps,
    double peakForwardVoltage,
    double peakReverseVoltage,
    NeutralModeValue neutralMode) {}

  public record SparkConfiguration(
    boolean invert,
    int smartCurrentLimitAmps,
    int secondaryCurrentLimitAmps,
    IdleMode idleMode) {}

  public record PivotHardware(
    int motorId,
    double gearing) {}

  public record PivotGains(
    // Feedback control
    double p, 
    double i, 
    double d, 
    // Motion magic constraints
    double maxVelocityRotationsPerSecond, 
    double maxAccelerationRotationsPerSecondSquared, 
    double jerkRotationsPerSecondCubed, 
    // Pivot feedforward values
    double s, 
    double v, 
    double a, 
    double g) {}

  public record PivotTalonFXConfiguration(
   boolean invert,
    boolean enableStatorCurrentLimit,
    boolean enableSupplyCurrentLimit,
    double statorCurrentLimitAmps,
    double supplyCurrentLimitAmps,
    double peakForwardVoltage,
    double peakReverseVoltage,
    NeutralModeValue neutralMode) {}

  public record PivotSimulationConfiguration(
    DCMotor motorType,
    double momentOfInertiaJKgMetersSquared,
    double ligamentLengthMeters,
    Rotation2d minPosition,
    Rotation2d maxPosition,
    boolean simulateGravity,
    Rotation2d initialPosition,
    double measurementStdDevs) {}

  /**
   * See this comment and documentation about the units if x, y, w, h 
   * https://github.com/Prosper-FRC/robot-reefscape-common/pull/20#discussion_r1947983003
   * https://grapplerobotics.au/product/lasercan/
   */
  public record SensorConfiguration(
    int sensorId,
    double detectionThresholdMilimeters,
    int x,
    int y,
    int w,
    int h
  ) {}

  public record SimulationConfiguration(
    DCMotor motorType,
    double measurementStdDevs
  ) {}

  /* Intake constants */

  /** The intake subsystem will periodically compare the motors current amperage to this 
   * value, if it is exceeding this value over a certain interval of time, it likely has 
   * a gamepiece as intaking a gamepiece spikes the amperage of the motor */
  public static final int kCoralAmpFilterThreshold = 15; // TODO Tune this
  /** The intake subsystem will periodically compare the motors current amperage to this 
   * value, if it is exceeding this value over a certain interval of time, it likely has 
   * a gamepiece as intaking a gamepiece spikes the amperage of the motor */
  public static final int kAlgaeAmpFilterThreshold = 20; // TODO Tune this

  /** The number of motor current reading samples the gamepeice detection filter averages 
   * over, this number cannot be 0 */
  public static final int kLinearFilterSampleCount = 5;

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final IntakeHardware kIntakeHardware = new IntakeHardware(
    56, // Motor CAN ID
    3.0 /1.0, // Gearing
    Units.inchesToMeters(0.5)); // Wheel radius meters

  public static final IntakeMotorConfiguration kIntakeMotorConfiguration = new IntakeMotorConfiguration(
    false, 
    true, 
    true, 
    60.0, 
    45.0, 
    12.0,
    -12.0,
    NeutralModeValue.Brake);

  public static final SparkConfiguration kIntakeSparkConfiguration = new SparkConfiguration(
    false,
    60,
    80,
    IdleMode.kCoast);

  public static final SensorConfiguration kSensorConfiguration = new SensorConfiguration(
    57, // Sensor CAN ID
    40, // Sensor detection threshold distance meters (detection distance)
    8, // Region of interest SPADs starting x-coordinate
    8, // Region of interest SPADs starting y-coordinate
    16, // Total region of interest width
    16); // Totral region of interest height
    
  public static final SimulationConfiguration kIntakeSimulationConfiguration = new SimulationConfiguration(
    DCMotor.getKrakenX60(1), 
    0.0002);

  /* Pivot constants */
  
  public static final Rotation2d kMinPivotPosition = Rotation2d.fromDegrees(-70.0);
  public static final Rotation2d kMaxPivotPosition = Rotation2d.fromDegrees(64.331);

  public static final Rotation2d kPivotPositionTolerance = Rotation2d.fromDegrees(1.0);
  
  public static final double kPivotGearing = (48.0 / 1.0) * (48.0 / 22.0);

  public static final PivotHardware kPivotMotorHardware = new PivotHardware(
    58, // CAN ID
    kPivotGearing); // Gear ratio

  public static final PivotGains kPivotGains =  
    switch (Constants.kCurrentMode) {
      case REAL -> new PivotGains(
        5.0,
        0.0,
        0.0,
        1000,
        0.5,
        0.025,
        0.0,
        13.0,
        0.0, 
        0.0); 
      case SIM -> new PivotGains(
        550.0,
        0.0,
        0.0,
        10.0,
        4.0,
        0.0,
        0.0,
        0.17,
        0.06, 
        0.01); 
      default -> new PivotGains(
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);
    };

  public static final PivotTalonFXConfiguration kPivotMotorConfiguration = new PivotTalonFXConfiguration(
    true, // Invert
    true, // Enable stator current limiting
    true, // Enable supply current limiting
    60.0, // Stator limit
    50.0, // Supply limit
    12.0, // Peak forward voltage
    -12.0, // Peak reverse voltage
    NeutralModeValue.Brake); // Idle mode

  // Pivot mass: 2.6553 kg
  // Distance from COM: ~14in
  public static final PivotSimulationConfiguration kPivotSimulationConfiguration = new PivotSimulationConfiguration(
    DCMotor.getKrakenX60(1), // Motor type and count
    0.009261, // Moment of inertia (calculated from latest CAD)
    Units.inchesToMeters(20.0), // Ligament length meters
    kMinPivotPosition, // Min position
    kMaxPivotPosition, // Max position
    true, // Simulate gravity
    new Rotation2d(), // Initial position
    0.002); // Std devs

  public static final PivotVisualizerConfiguration kPivotVisualizerConfiguration = new PivotVisualizerConfiguration(
    new Pair<Double,Double>(1.0, 3.0),
    "boxPivotRoot",
    new Pair<Double,Double>(0.75, 2.0),
    "boxPivotLigament",
    Rotation2d.fromDegrees(270.0),
    Units.inchesToMeters(20.0),
    new Rotation2d());
}