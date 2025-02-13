// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climb.ClimbConstants.ClimbGains;
import frc.robot.subsystems.climb.ClimbConstants.ClimbHardware;
import frc.robot.subsystems.climb.ClimbConstants.ClimbTalonFXConfiguration;

public class ClimbIOTalonFX implements ClimbIO {
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

  public ClimbIOTalonFX(
    String canbus,
    ClimbHardware hardware,
    ClimbTalonFXConfiguration configuration,
    ClimbGains gains,
    double statusSignalUpdateFrequency) {

    kMotor = new TalonFX(hardware.motorId(), canbus);

    // Apply configurations
    motorConfiguration.Slot0.kP = gains.p();
    motorConfiguration.Slot0.kI = gains.i();
    motorConfiguration.Slot0.kD = gains.d();
    motorConfiguration.Slot0.kS = gains.s();
    motorConfiguration.Slot0.kV = gains.v();
    motorConfiguration.Slot0.kA = gains.a();
    motorConfiguration.Slot0.kG = gains.g();
    motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = gains.maxVelocityMetersPerSecond();
    motorConfiguration.MotionMagic.MotionMagicAcceleration = gains.maxAccelerationMetersPerSecondSquared();
    motorConfiguration.MotionMagic.MotionMagicJerk = gains.jerkMetersPerSecondCubed();

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

    BaseStatusSignal.setUpdateFrequencyForAll(statusSignalUpdateFrequency,
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

  public ClimbIOTalonFX(
    ClimbHardware hardware,
    ClimbTalonFXConfiguration configuration,
    ClimbGains gains,
    double statusSignalUpdateFrequency) {

    // Assumes the rio is the CANBus
    this("rio", hardware, configuration, gains, statusSignalUpdateFrequency);
  }

  public ClimbIOTalonFX(
    String canbus,
    ClimbHardware hardware,
    ClimbTalonFXConfiguration configuration,
    ClimbGains gains,
    double statusSignalUpdateFrequency,
    int leadMotorId,
    boolean opposeLeadMotorDirection) {

    this(canbus, hardware, configuration, gains, statusSignalUpdateFrequency);
    kMotor.setControl(new Follower(leadMotorId, opposeLeadMotorDirection));
  }
}
