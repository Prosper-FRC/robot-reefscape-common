// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.visualizers;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// How to use the visualizer with AdvantageScope:
// https://docs.advantagescope.org/tab-reference/mechanism

/** 
 * Class to handle graphical visualization of a pivot (arm) mechanism. 
 * 
 * @author barbute
 */
public class PivotVisualizer {
  public record PivotVisualizerConfiguration(
    Pair<Double, Double> visualFieldSize,
    String rootName,
    Pair<Double, Double> rootPoisition,
    String ligamentName,
    Rotation2d offset,
    double ligamentLengthMeters,
    Rotation2d initialPosition
  ) {}

  // Standard path that the visual data should be published to
  private final String kLogKey;

  private LoggedMechanism2d visualField;

  private LoggedMechanismRoot2d root;
  private LoggedMechanismLigament2d ligament;

  private Rotation2d offset;

  private double rootXPosition = 0.0;

  /**
   * Creates a new visualizer
   * 
   * @param logKey The NT key this will be published under. MUST BE UNIQUE
   * @param configuration The configuration for the visualizer's measure settings
   * @param lineWeight Ligament line weight on visualizer
   * @param color Ligament line color on visualizer
   */
  public PivotVisualizer(
    String logKey, 
    PivotVisualizerConfiguration configuration, 
    double lineWeight, 
    Color8Bit color) {
    kLogKey = logKey;

    visualField = new LoggedMechanism2d(
      configuration.visualFieldSize.getFirst(), 
      configuration.visualFieldSize.getSecond());
    
    root = visualField.getRoot(
      configuration.rootName, 
      configuration.rootPoisition.getFirst(), 
      configuration.rootPoisition.getSecond());

    rootXPosition = configuration.rootPoisition.getFirst();

    offset = configuration.offset;

    ligament = root.append(
      new LoggedMechanismLigament2d(
        configuration.ligamentName(), 
        configuration.ligamentLengthMeters(), 
        configuration.initialPosition().getDegrees()));

    ligament.setLineWeight(lineWeight);
    ligament.setColor(color);

    Logger.recordOutput(kLogKey, visualField);
  }

  /**
   * Creates a new visualizer
   * 
   * @param configuration The configuration for the visualizer's measure settings 
   */
  public PivotVisualizer(PivotVisualizerConfiguration configuration) {
    this("Pivot/Visualier", configuration, 4.0, new Color8Bit(Color.kBlue));
  }

  /**
   * Updates the position of the pivot on the visualizer
   * 
   * @param position The current positiopn of the pivot mechanism
   */
  public void updatePosition(Rotation2d position) {
    ligament.setAngle(position.minus(offset));

    Logger.recordOutput(kLogKey, visualField);
  }

  /**
   * Updates the position of the pivot root on the visualizer
   * 
   * @param positionMeters The current vertical position of the pivot mechanism
   */
  public void setRootVerticalPositionMeters(double positionMeters) {
    root.setPosition(rootXPosition, positionMeters);
  }
}

