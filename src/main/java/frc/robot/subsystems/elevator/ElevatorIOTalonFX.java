// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorGains;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHardware;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorMotorConfiguration;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX kMotor;

  private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

  // Motor data we wish to log
  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSec;
  private StatusSignal<Voltage> appliedVolts;
  private StatusSignal<Current> supplyCurrentAmps;
  private StatusSignal<Current> statorCurrentAmps;
  private StatusSignal<Temperature> temperatureCelsius;

  // Control modes
  private final VoltageOut kVoltageControl = new VoltageOut(0.0);
  private final MotionMagicVoltage kPositionControl = new MotionMagicVoltage(0.0);

  // Save drum circumference for converting
  private final double kDrumCircumferenceMeters;

  public ElevatorIOTalonFX(
    String canbus,
    ElevatorHardware hardware,
    ElevatorMotorConfiguration configuration,
    ElevatorGains gains) {
    kMotor = new TalonFX(hardware.motorId(), canbus);
    kDrumCircumferenceMeters = hardware.drumCircumferenceMeters();

    // Apply configurations
    motorConfiguration.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    motorConfiguration.Slot0.kP = metersToRotations(gains.p());
    motorConfiguration.Slot0.kI = metersToRotations(gains.i());
    motorConfiguration.Slot0.kD = metersToRotations(gains.d());
    motorConfiguration.Slot0.kS = gains.s();
    motorConfiguration.Slot0.kV = metersToRotations(gains.v());
    motorConfiguration.Slot0.kA = metersToRotations(gains.a());
    motorConfiguration.Slot0.kG = gains.g();
    motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = metersToRotations(gains.maxVelocityMetersPerSecond());
    motorConfiguration.MotionMagic.MotionMagicAcceleration = metersToRotations(gains.maxAccelerationMetersPerSecondSquared());
    motorConfiguration.MotionMagic.MotionMagicJerk = metersToRotations(gains.jerkMetersPerSecondCubed());

    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = configuration.enableSupplyCurrentLimit();
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = configuration.supplyCurrentLimitAmps();
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = configuration.enableStatorCurrentLimit();
    motorConfiguration.CurrentLimits.StatorCurrentLimit = configuration.statorCurrentLimitAmps();
    motorConfiguration.Voltage.PeakForwardVoltage = configuration.peakForwardVoltage();
    motorConfiguration.Voltage.PeakReverseVoltage = configuration.peakReverseVoltage();

    motorConfiguration.MotorOutput.NeutralMode = configuration.neutralMode();
    motorConfiguration.MotorOutput.Inverted = 
      configuration.invert() 
        ? InvertedValue.CounterClockwise_Positive 
        : InvertedValue.Clockwise_Positive;

    // Reset position on startup
    kMotor.setPosition(0.0);

    motorConfiguration.Feedback.SensorToMechanismRatio = hardware.gearing();
    // Rotor sensor is the built-in sensor
    motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    kMotor.getConfigurator().apply(motorConfiguration, 1.0);

    // Get status signals from the motor controller
    positionRotations = kMotor.getPosition();
    velocityRotationsPerSec = kMotor.getVelocity();
    appliedVolts = kMotor.getMotorVoltage();
    supplyCurrentAmps = kMotor.getSupplyCurrent();
    statorCurrentAmps = kMotor.getStatorCurrent();
    temperatureCelsius = kMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(ElevatorConstants.kStatusSignalUpdateFrequencyHz,
      positionRotations,
      velocityRotationsPerSec,
      appliedVolts,
      supplyCurrentAmps,
      supplyCurrentAmps,
      statorCurrentAmps,
      temperatureCelsius);

    // Optimize the CANBus utilization by explicitly telling all CAN signals we
    // are not using to simply not be sent over the CANBus
    kMotor.optimizeBusUtilization(0.0, 1.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.isMotorConnected = BaseStatusSignal.refreshAll(
      positionRotations,
      velocityRotationsPerSec,
      appliedVolts,
      supplyCurrentAmps,
      supplyCurrentAmps,
      statorCurrentAmps,
      temperatureCelsius).isOK();

    inputs.positionMeters =  rotationsToMeters(positionRotations.getValueAsDouble());
    inputs.velocityMetersPerSec = rotationsToMeters(velocityRotationsPerSec.getValueAsDouble());
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrentAmps.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentAmps.getValueAsDouble();
    inputs.temperatureCelsius = temperatureCelsius.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    kMotor.setControl(kVoltageControl.withOutput(volts));
  }

  @Override
  public void setPosition(double positionMeters) {
    kMotor.setControl(
      kPositionControl.withPosition(metersToRotations(positionMeters)).withSlot(0));
  }

  @Override
  public void stop() {
    kMotor.setControl(new NeutralOut());
  }

  @Override
  public void resetPosition() {
    kMotor.setPosition(0.0);
  }

  @Override
  public void setGains(double p, double i, double d, double s, double g, double v, double a) {
    var slotConfiguration = new Slot0Configs();
    slotConfiguration.GravityType = GravityTypeValue.Elevator_Static;

    slotConfiguration.kP = metersToRotations(p);
    slotConfiguration.kI = metersToRotations(i);
    slotConfiguration.kD = metersToRotations(d);
    slotConfiguration.kS = s;
    slotConfiguration.kG = g;
    slotConfiguration.kV = metersToRotations(v);
    slotConfiguration.kA = metersToRotations(a);

    kMotor.getConfigurator().apply((slotConfiguration));
  }

  @Override
  public void setMotionMagicConstraints(double maxVelocity, double maxAcceleration) {
    var motionMagicConfiguration = new MotionMagicConfigs();

    motionMagicConfiguration.MotionMagicCruiseVelocity = metersToRotations(maxVelocity);
    motionMagicConfiguration.MotionMagicAcceleration = metersToRotations(maxAcceleration);
    motionMagicConfiguration.MotionMagicJerk = 10.0 * metersToRotations(maxAcceleration);

    kMotor.getConfigurator().apply(motionMagicConfiguration);
  }

  @Override
  public void setBrakeMode(boolean enableBrake) {
    kMotor.setNeutralMode(enableBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  private double rotationsToMeters(double rotations) {
    /*
     * Multiply by two since this is a two-stage cascading-elevator where both stages move
     * simultaneously. If this was a three-stage cascading-elevator we would multiply by 3
     */
    return rotations * kDrumCircumferenceMeters * 2.0;
  }

  private double metersToRotations(double meters) {
    /*
     * Divide by two since this is a two-stage cascading-elevator where both stages move
     * simultaneously. If this was a three-stage cascading-elevator we would divide by 3
     */
    return (meters / kDrumCircumferenceMeters) / 2.0;
  }
}
