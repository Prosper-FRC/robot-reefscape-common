// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import frc.robot.Constants;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Constants for a pivot */
public class ClimbConstants {
  public record ClimbHardware(
    int motorId,
    double gearing
  ) {}

  public record ClimbGains(
    // Feedback control
    double p, 
    double i, 
    double d, 
    // Motion magic constraints
    double maxVelocityMetersPerSecond, 
    double maxAccelerationMetersPerSecondSquared, 
    double jerkMetersPerSecondCubed, 
    // Climb feedforward values
    double s, 
    double v, 
    double a, 
    double g) {}

  public record ClimbTalonFXConfiguration(
   boolean invert,
    boolean enableStatorCurrentLimit,
    boolean enableSupplyCurrentLimit,
    double statorCurrentLimitAmps,
    double supplyCurrentLimitAmps,
    double peakForwardVoltage,
    double peakReverseVoltage,
    NeutralModeValue neutralMode) {}

  public record ClimbSimulationConfiguration(
    DCMotor motorType,
    double momentOfInertiaJKgMetersSquared,
    double ligamentLengthMeters,
    Rotation2d minPosition,
    Rotation2d maxPosition,
    boolean simulateGravity,
    Rotation2d initialPosition,
    double meadurementStdDevs) {}

  public record DutyCycleConfiguration(
    int encoderChannel,
    int connectedFrequencyThresholdHz,
    double minimumDutyCyclerange,
    double maximumDutyCycleRange
  ) {}

  public static final Rotation2d kMinPosition = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d kMaxPosition = Rotation2d.fromDegrees(0.0);

  public static final double kGearing = 125.0 / 1.0;

  public static final double kStatusSignalUpdateFrequency = 20.0;

  public static final ClimbHardware kLeadMotorHardware = new ClimbHardware(
    0, // CAN ID
    kGearing); // Gear ratio
  public static final ClimbHardware kFollowerMotorHardware = new ClimbHardware(
    0, // CAN ID
    kGearing); // Gear ratio

  public static final ClimbGains kMotorGains =  
    switch (Constants.kCurrentMode) {
      case REAL -> new ClimbGains(
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
      case SIM -> new ClimbGains(
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
      default -> new ClimbGains(
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

  public static final ClimbTalonFXConfiguration kLeadMotorConfiguration = new ClimbTalonFXConfiguration(
    true, // Invert
    true, // Enable stator current limiting
    true, // Enable supply current limiting
    60.0, // Stator limit
    50.0, // Supply limit
    12.0, // Peak forward voltage
    -12.0, // Peak reverse voltage
    NeutralModeValue.Brake); // Idle mode
  public static final ClimbTalonFXConfiguration kFollowMotorConfiguration = new ClimbTalonFXConfiguration(
    true, // Invert
    true, // Enable stator current limiting
    true, // Enable supply current limiting
    60.0, // Stator limit
    50.0, // Supply limit
    12.0, // Peak forward voltage
    -12.0, // Peak reverse voltage
    NeutralModeValue.Brake); // Idle mode

  public static final ClimbSimulationConfiguration kSimulationConfiguration = new ClimbSimulationConfiguration(
    DCMotor.getKrakenX60(2), // Motor type and count
    0.1, // Moment of inertia
    Units.inchesToMeters(12.0), // Ligament length meters
    kMinPosition, // Min position
    kMaxPosition, // Max position
    true, // Simulate gravity
    new Rotation2d(), // Initial position
    0.002); // Std devs

  public static final DutyCycleConfiguration kDutyCycleConfiguration = new DutyCycleConfiguration(
    0,
    0,
    0.0,
    1.0);
}
