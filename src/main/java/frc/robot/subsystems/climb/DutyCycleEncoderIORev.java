// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.climb.ClimbConstants.DutyCycleConfiguration;

/** 
 * A class to interact with the REV Through Bore encoder set up as a duty
 * cycle encoder (absolute encoder configuration)
 * 
 * Related documentation:
 * https://docs.revrobotics.com/rev-crossover-products/sensors/tbe
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html
 * https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/wpilibj/DutyCycleEncoder.html
 */
public class DutyCycleEncoderIORev implements DutyCycleEncoderIO {
  private final DutyCycleEncoder kEncoder;

  public DutyCycleEncoderIORev(DutyCycleConfiguration configuration) {
    kEncoder = new DutyCycleEncoder(configuration.encoderChannel());

    kEncoder.setConnectedFrequencyThreshold(configuration.connectedFrequencyThresholdHz());
    kEncoder.setDutyCycleRange(
      configuration.minimumDutyCycleRange(), 
      configuration.maximumDutyCycleRange());
  }

  @Override
  public void updateInputs(DutyCycleEncoderIOInputs inputs) {
    inputs.isConnected = kEncoder.isConnected();

    inputs.frequencyHz = kEncoder.getFrequency();
    inputs.dutyCycleReading = kEncoder.get();
  }
}
