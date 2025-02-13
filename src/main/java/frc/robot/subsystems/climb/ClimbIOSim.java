// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climb;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.climb.ClimbConstants.ClimbGains;
import frc.robot.subsystems.climb.ClimbConstants.ClimbHardware;
import frc.robot.subsystems.climb.ClimbConstants.ClimbSimulationConfiguration;

public class ClimbIOSim implements ClimbIO {
  private final double kLoopPeriodSec;

  private final SingleJointedArmSim kPivot;

  // Create and use the feedback and feedforware controllers in here since we 
  // are using the internal motor controllers
  private TrapezoidProfile kProfile;

  private final PIDController kFeedback;

  // Feedforward still a constant, but we have to reinstantiate the model in 
  // order to change the gains
  private ElevatorFeedforward kFeedforward;

  // Final goal of the mechanism
  private TrapezoidProfile.State goal = new TrapezoidProfile.State();
  // Setpoint to go to following the profile
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

  private double appliedVoltage = 0.0;

  // Used to determine the current control mode of the simulation hardware
  private boolean feedbackNeedsReset = false;
  private boolean closedLoopControl = false;

  public ClimbIOSim(
    double loopPeriodSec,
    ClimbHardware hardware,
    ClimbSimulationConfiguration configuration, 
    ClimbGains gains) {

    kLoopPeriodSec = loopPeriodSec;

    kPivot = new SingleJointedArmSim(
      configuration.motorType(),
      hardware.gearing(),
      configuration.momentOfInertiaJKgMetersSquared(),
      configuration.ligamentLengthMeters(),
      configuration.minPosition().getRadians(),
      configuration.maxPosition().getRadians(), 
      configuration.simulateGravity(), 
      configuration.initialPosition().getRadians(), 
      configuration.meadurementStdDevs());

    kProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
      gains.maxVelocityMetersPerSecond(), 
      gains.maxAccelerationMetersPerSecondSquared()));

    kFeedback = new PIDController(gains.p(), gains.i(), gains.d());

    kFeedforward = 
      new ElevatorFeedforward(gains.s(), gains.g(), gains.v(), gains.a());
  }

  @Override
  public void updateInputs(ClimbIOInputs inputs) {
    kPivot.update(kLoopPeriodSec);

    inputs.isMotorConnected = true;

    inputs.position = Rotation2d.fromRadians(kPivot.getAngleRads());
    inputs.velocityUnitsPerSec = Rotation2d.fromRadians(kPivot.getVelocityRadPerSec());
    inputs.appliedVoltage = appliedVoltage;
    // These are 0 cause we don't care able these in simulation
    inputs.supplyCurrentAmps = 0.0;
    inputs.statorCurrentAmps = 0.0;
    inputs.temperatureCelsius = 0.0;
  }
}
