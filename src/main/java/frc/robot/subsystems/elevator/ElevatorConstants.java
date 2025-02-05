
package frc.robot.subsystems.elevator;

import frc.robot.Constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class ElevatorConstants {
  public record ElevatorGains(
    // Feedback control
    double kP, 
    double kI, 
    double kD, 
    // Motion magic constraints
    double kMaxVelocityMetersPerSecond, 
    double kMaxAccelerationMetersPerSecondSquared, 
    double kJerkMetersPerSecondCubed, 
    // Elevator feedforward values
    double kS, 
    double kV, 
    double kA, 
    double kG) {}

  public record KrakenConfiguration(
    boolean kInvert,
    boolean kEnableStatorCurrentLimit,
    boolean kEnableSupplyCurrentLimit,
    double kStatorCurrentLimitAmps,
    double kSupplyCurrentLimitAmps,
    NeutralModeValue kNeutralMode) {}

  public record SimulationConfiguration(
    DCMotor kMotorType,
    double kCarriageMassKg,
    double kDrumRadiusMeters,
    boolean kSimulateGravity,
    double kStartingHeightMeters,
    double kMeasurementStdDevs
  ) {}

  // Taken from mech and electrical
  public static final int kMotorID = 50;

  public static final double kGearing = 9.0 / 1.0;
  /*
   * Outside sprocket radius: 0.944 in
   * Root sprocket radius: 0.819
   */
  public static final double kDrumRadiusMeters = Units.inchesToMeters(0.944);
  public static final double kDrumCircumferenceMeters = 2.0 * Math.PI * kDrumRadiusMeters;

  // empty carriage load = .8kg
  // prototype carriage load = 5kg

  public static final double kMaxPositionMeters = 1.65;
  public static final double kMinPositionMeters = 0.0;

  /** Position tolerance when controlling the elevator via feedback */
  public static final double kPositionToleranceMeters = 0.01;

  /** The frequency that telemetry form the motor is pushed to the CANBus */
  public static final double kStatusSignalUpdateFrequencyHz = 100.0;

  public static final ElevatorGains kElevatorGains = 
    switch (Constants.kCurrentMode) {
      case REAL -> new ElevatorGains(
        5.0,
        0.0,
        0.0,
        2.947, //2.947
        22.0, // 22
        0.0,
        0.0,
        0.5,
        0.01, // 0.01
        0.11); // 0.11
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

  public static final KrakenConfiguration kMotorConfiguration = new KrakenConfiguration(
    false, 
    true, 
    true, 
    60.0, 
    50.0, 
    NeutralModeValue.Brake);

  public static final SimulationConfiguration kSimulationConfiguration = new SimulationConfiguration(
    DCMotor.getKrakenX60(1), 
    5.0, 
    kDrumRadiusMeters, 
    true, 
    0.0, 
    0.0002);
}

