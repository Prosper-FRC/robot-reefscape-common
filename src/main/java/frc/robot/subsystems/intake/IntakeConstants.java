
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
  public record IntakeHardware(
    int kMotorId,
    double kGearing,
    double kWheelRadiusMeters
  ) {}

  public record IntakeMotorConfiguration(
    boolean kInvert,
    boolean kEnableStatorCurrentLimit,
    boolean kEnableSupplyCurrentLimit,
    double kStatorCurrentLimitAmps,
    double kSupplyCurrentLimitAmps,
    double kPeakForwardVoltage,
    double kPeakReverseVoltage,
    NeutralModeValue kNeutralMode) {}

  public record SparkConfiguration(
    boolean kInvert,
    int kSmartCurrentLimitAmps,
    int kSecondaryCurrentLimitAmps,
    IdleMode kIdleMode) {}

  public record SensorConfiguration(
    double kDetectionThresholdMeters
  ) {}

  public record SimulationConfiguration(
    DCMotor kMotorType,
    double kMeasurementStdDevs
  ) {}

  // Taken from mech and electrical
  public static final int kCANrangeID = 57;

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
    1.0);
    
  public static final SimulationConfiguration kSimulationConfiguration = new SimulationConfiguration(
    DCMotor.getKrakenX60(1), 
    0.0002);

}