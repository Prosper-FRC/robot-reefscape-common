// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorHardware;
import frc.robot.subsystems.intake.IntakeConstants.SparkConfiguration;

// NOTE This is a temporary class and is only used for testing. As such the code 
// in this file should not be used in production
public class IntakeIOSparkMax implements IntakeIO {
  private final SparkBase kMotor;
  private final RelativeEncoder kEncoder;

  private SparkMaxConfig motorConfiguration = new SparkMaxConfig();

  public IntakeIOSparkMax(ElevatorHardware hardware, SparkConfiguration configuration) {
    kMotor = new SparkMax(hardware.motorId(), MotorType.kBrushless);
    kEncoder = kMotor.getEncoder();

    motorConfiguration.inverted(
      configuration.invert());
    motorConfiguration.smartCurrentLimit(
      configuration.smartCurrentLimitAmps());
    motorConfiguration.secondaryCurrentLimit(
      configuration.secondaryCurrentLimitAmps());
    motorConfiguration.idleMode(
      configuration.idleMode());

    kMotor.configure(motorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Setting this to true since we don't care about this value
    inputs.isMotorConnected = true;

    inputs.velocityRotationsPerSec = kEncoder.getVelocity() / 60.0;
    inputs.appliedVoltage = kMotor.getAppliedOutput() * kMotor.getBusVoltage();
    inputs.supplyCurrentAmps = kMotor.getOutputCurrent();
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = kMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    volts = MathUtil.applyDeadband(volts, -12.0, 12.0);
    kMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    kMotor.stopMotor();
  }
}
