
package frc.robot.subsystems.elevator;

import frc.robot.Constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public record ElevatorHardware(
    int motorId,
    double gearing,
    double drumRadiusMeters,
    double drumCircumferenceMeters
  ) {}

  public record ElevatorGains(
    // Feedback control
    double p, 
    double i, 
    double d, 
    // Motion magic constraints
    double maxVelocityMetersPerSecond, 
    double maxAccelerationMetersPerSecondSquared, 
    double jerkMetersPerSecondCubed, 
    // Elevator feedforward values
    double s, 
    double v, 
    double a, 
    double g) {}

  public record ElevatorMotorConfiguration(
    boolean invert,
    boolean enableStatorCurrentLimit,
    boolean enableSupplyCurrentLimit,
    double statorCurrentLimitAmps,
    double supplyCurrentLimitAmps,
    double peakForwardVoltage,
    double peakReverseVoltage,
    NeutralModeValue neutralMode) {}

  public record SimulationConfiguration(
    DCMotor motorType,
    double carriageMassKg,
    double drumRadiusMeters,
    boolean simulateGravity,
    double startingHeightMeters,
    double measurementStdDevs) {}

  public record MagneticSensorHardware(
    int sensorChannel
  ){}

  // Taken from mech and electrical
  public static final double kDrumRadiusMeters = Units.inchesToMeters(0.944);
  public static final double kDrumCircumferenceMeters = 2.0 * Math.PI * kDrumRadiusMeters;

  /** The number of motor current reading samples the homing filter averages over, 
   * this number cannot be 0 */
  public static final int kLinearFilterSampleCount = 5;
  public static final int kAmpFilterThreshold = 40;

  public static final double kMaxPositionMeters = 1.655;
  public static final double kMinPositionMeters = 0.0;

  /** Position tolerance when controlling the elevator via feedback */
  public static final double kPositionToleranceMeters = 0.01;

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final ElevatorHardware kRoboElevatorHardware = new ElevatorHardware(
    50, // Motor CAN ID
    15.0 / 1.0,  // Gearing
    /*
    * Outside sprocket radius: 0.944 in
    * Root sprocket radius: 0.819
    */
    kDrumRadiusMeters, // Drum (sprocket) radius
    kDrumCircumferenceMeters); // Drum (sprocket) circumference

  public static final ElevatorGains kElevatorGains = 
    switch (Constants.kCurrentMode) {
      case REAL -> new ElevatorGains(
        2.0,
        0.0,
        0.0,
        2.947, //2.947
        22.0, // 22
        0.0,
        0.0,
        0.5,
        0.0, // 0.01
        0.65); // 0.11
      case SIM -> new ElevatorGains(
        1.0,
        0.0,
        0.0,
        10.0,
        25.0,
        0.0,
        0.0,
        14.34,
        0.01,
        0.11);
      default -> new ElevatorGains(
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

  public static final ElevatorMotorConfiguration kMotorConfiguration = new ElevatorMotorConfiguration(
    false, 
    true, 
    true, 
    60.0, 
    50.0, 
    12.0,
    -12.0,
    NeutralModeValue.Brake);

  public static final SimulationConfiguration kSimulationConfiguration = new SimulationConfiguration(
    DCMotor.getKrakenX60(1), 
    // empty carriage load = .8kg
    // prototype carriage load = 13.61 kg
    5.0, 
    kDrumRadiusMeters, 
    true, 
    0.0, 
    0.0002);

  // TODO Replace with real channel later
  public static final MagneticSensorHardware kSensorHardware = new MagneticSensorHardware(
    0
  );
}

