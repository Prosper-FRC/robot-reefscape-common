
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public record IntakeHardware(
    int motorId,
    double gearing,
    double wheelRadiusMeters
  ) {}

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

  /** The intake subsystem will periodically compare the motors current amperage to this 
   * value, if it is exceeding this value over a certain interval of time, it likely has 
   * a gamepiece as intaking a gamepiece spikes the amperage of the motor */
  public static final int kAmpFilterThreshold = 15;

  /** The number of motor current reading samples the gamepeice detection filter averages 
   * over, this number cannot be 0 */
  public static final int kLinearFilterSampleCount = 5;

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final IntakeHardware kRoboIntakeHardware = new IntakeHardware(
    56, // Motor CAN ID
    3.0 /1.0, // Gearing
    Units.inchesToMeters(0.5)); // Wheel radius meters

  public static final IntakeMotorConfiguration kMotorConfiguration = new IntakeMotorConfiguration(
    false, 
    true, 
    true, 
    60.0, 
    45.0, 
    12.0,
    -12.0,
    NeutralModeValue.Brake);

  public static final SparkConfiguration kSparkConfiguration = new SparkConfiguration(
    false,
    60,
    80,
    IdleMode.kCoast);

  public static final SensorConfiguration kSensorConfiguration = new SensorConfiguration(
    57,
    1.0,
    8,
    8,
    16,
    16);
    
  public static final SimulationConfiguration kSimulationConfiguration = new SimulationConfiguration(
    DCMotor.getKrakenX60(1), 
    0.0002);

}