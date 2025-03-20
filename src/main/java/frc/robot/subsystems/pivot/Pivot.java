// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.visualizers.PivotVisualizer;

public class Pivot {
  /** List of position setpoints for the pivot */
  public enum PivotGoal {
    kStowScore(() -> Rotation2d.fromDegrees(64.0)),
    kStowPickup(() -> Rotation2d.fromDegrees(54.0)),
    kIntakeReef(() -> Rotation2d.fromDegrees(5.0)),
    kIntakeGround(() -> Rotation2d.fromDegrees(-55.5)),
    kProcessorScore(() -> Rotation2d.fromDegrees(-30.0)),
    kScore(() -> Rotation2d.fromDegrees(40.0)),
    kBargeScore(() -> Rotation2d.fromDegrees(46.0)),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(() -> Rotation2d.fromDegrees(
      new LoggedTunableNumber("Intake/Feedback/PivotSetpointDegrees", 0.0).get()));

    private Supplier<Rotation2d> goalPosition;

    PivotGoal(Supplier<Rotation2d> goalPosition) {
      this.goalPosition = goalPosition;
    }

    public Rotation2d getGoalPosition() {
      return this.goalPosition.get();
    }
  }

  private final PivotIO kPivotHardware;
  private final PivotIOInputsAutoLogged kPivotInputs = new PivotIOInputsAutoLogged();

  private final LoggedTunableNumber kP =
      new LoggedTunableNumber("Pivot/Gains/kP", PivotConstants.kPivotGains.p());
  private final LoggedTunableNumber kI =
      new LoggedTunableNumber("Pivot/Gains/kI", PivotConstants.kPivotGains.i());
  private final LoggedTunableNumber kD =
      new LoggedTunableNumber("Pivot/Gains/kD", PivotConstants.kPivotGains.d());
  private final LoggedTunableNumber kS =
      new LoggedTunableNumber("Pivot/Gains/kS", PivotConstants.kPivotGains.s());
  private final LoggedTunableNumber kV =
      new LoggedTunableNumber("Pivot/Gains/kV", PivotConstants.kPivotGains.v());
  private final LoggedTunableNumber kA =
      new LoggedTunableNumber("Pivot/Gains/kA", PivotConstants.kPivotGains.a());
  private final LoggedTunableNumber kG =
      new LoggedTunableNumber("Pivot/Gains/kG", PivotConstants.kPivotGains.g());
  private final LoggedTunableNumber kMaxVelocity =
      new LoggedTunableNumber(
          "Pivot/MotionMagic/kMaxVelocity", 
          PivotConstants.kPivotGains.maxVelocityRotationsPerSecond());
  private final LoggedTunableNumber kMaxAcceleration =
      new LoggedTunableNumber(
          "Pivot/MotionMagic/kMaxAcceleration", 
          PivotConstants.kPivotGains.maxAccelerationRotationsPerSecondSquared());

  private PivotGoal pivotGoal = null;

  // Object used to visualize the mechanism over network tables, useful in simulation
  private final PivotVisualizer kPivotVisualizer;

  public Pivot(PivotIO pivotIO) {
    kPivotHardware = pivotIO;

    kPivotVisualizer = new PivotVisualizer(
      "Pivot/Visualizer", 
      PivotConstants.kPivotVisualizerConfiguration, 
      4.0, 
      new Color8Bit(Color.kBlue));
  }
}
