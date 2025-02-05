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

// NOTE This is a temporary class and is only used for testing. As such the code 
// in this file should not be used in production
public class IntakeIOSparkMax implements IntakeIO {
  private final SparkBase kMotor = 
    new SparkMax(IntakeConstants.kMotorID, MotorType.kBrushless);
  private final RelativeEncoder kEncoder = kMotor.getEncoder();

  private SparkMaxConfig motorConfiguration = new SparkMaxConfig();

  public IntakeIOSparkMax() {
    motorConfiguration.inverted(
      IntakeConstants.kSparkConfiguration.kInvert());
    motorConfiguration.smartCurrentLimit(
      IntakeConstants.kSparkConfiguration.kSmartCurrentLimitAmps());
    motorConfiguration.secondaryCurrentLimit(
      IntakeConstants.kSparkConfiguration.kSecondaryCurrentLimitAmps());
    motorConfiguration.idleMode(
      IntakeConstants.kSparkConfiguration.kIdleMode());

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
