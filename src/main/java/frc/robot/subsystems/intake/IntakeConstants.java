
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

  /**
   * See this comment and documentation about the units if x, y, w, h 
   * https://github.com/Prosper-FRC/robot-reefscape-common/pull/20#discussion_r1947983003
   * https://grapplerobotics.au/product/lasercan/
   */
  public record SensorConfiguration(
    int sensorId,
    double detectionThresholdMeters,
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
    1.0, // Sensor detection threshold distance meters (detection distance)
    8, // Region of interest SPADs starting x-coordinate
    8, // Region of interest SPADs starting y-coordinate
    16, // Total region of interest width
    16); // Totral region of interest height
    
  public static final SimulationConfiguration kIntakeSimulationConfiguration = new SimulationConfiguration(
    DCMotor.getKrakenX60(1), 
    0.0002);
}