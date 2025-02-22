
package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.debugging.LoggedTunableNumber;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
public class Intake extends SubsystemBase {
  /** List of voltage setpoints for the intake in voltage */
  public enum RollerGoal {
    kIntakeCoral(() -> 2.0),
    kIntakeAlgae(() -> 5.0),
    kScoreCoral(() -> -4.0),
    kScoreAlgae(() -> -5.0),
    /** Custom setpoint that can be modified over network tables; Useful for debugging */
    custom(new LoggedTunableNumber("Intake/Feedback/RollerSetpointVolts", 0.0));

    private DoubleSupplier goalVoltage;

    RollerGoal(DoubleSupplier goalVoltage) {
      this.goalVoltage = goalVoltage;
    }

    public double getGoalVolts() {
      return this.goalVoltage.getAsDouble();
    }
  }

  /*
   * TODO At some point this can be moved out in favor of using some form of higher level 
   * robot state control
   */
  /** Specify which gamepiece the mechanism is attempting to manipulate */
  public enum Gamepiece {
    kCoral,
    kAlgae
  }

  private final IntakeIO kRollerHardware;
  private final IntakeIOInputsAutoLogged kRollerInputs = new IntakeIOInputsAutoLogged();

  private final SensorIO kSensor;
  private final SensorIOInputsAutoLogged kSensorInputs = new SensorIOInputsAutoLogged();

  private boolean detectedGamepiece = false;
  private LinearFilter ampFilter = LinearFilter.movingAverage(
    IntakeConstants.kLinearFilterSampleCount);

  private RollerGoal rollerGoal = null;

  // The default gamepiece is coral, this is because we will start preloaded with coral and will
  // assume throughout the rest of the code the robot will always default to scoring coral
  @AutoLogOutput(key ="Intake/Gamepiece")
  private Gamepiece selectedGamepiece = Gamepiece.kCoral;

  private final LoggedNetworkBoolean kOverrideDetectGamepiece = 
    new LoggedNetworkBoolean("Intake/OverrideDetectGamepiece", false);

  public Intake(IntakeIO hardwareIO, SensorIO sensorIO) {
    kRollerHardware = hardwareIO;
    kSensor = sensorIO;
  }

  @Override
  public void periodic() {
    kRollerHardware.updateInputs(kRollerInputs);
    Logger.processInputs("Intake/Inputs/Rollers", kRollerInputs);
    kSensor.updateInputs(kSensorInputs);
    Logger.processInputs("Intake/Inputs/Sensor", kSensorInputs);

    // Stop and clear goal if disabled. Used if copilot is still pressing button to command
    // intake when the disabled key is pressed
    if (DriverStation.isDisabled()) {
      stop(true);
    }

    if (selectedGamepiece == Gamepiece.kCoral) {
      // If the CANrange disconnects we can use motor current to detect when we have a coral
      // TODO Test this fallback to see if it actually works
      if (!kOverrideDetectGamepiece.get()) {
        if (kSensorInputs.isConnected) {
          detectedGamepiece = kSensorInputs.detectsObject;
        } else {
          // Checks for spike in amperage, and if greater than the value then
          // the intake motor probbaly has the coral
          detectedGamepiece = ampFilter.calculate(
            kRollerInputs.statorCurrentAmps) > IntakeConstants.kCoralAmpFilterThreshold;
        }
      } else {
        detectedGamepiece = true;
      }
    } else if (selectedGamepiece == Gamepiece.kAlgae) {
      if (!kOverrideDetectGamepiece.get()) {
        // Checks for spike in amperage, and if greater than the value then
        // the intake motor probbaly has the algae
        detectedGamepiece = ampFilter.calculate(
          kRollerInputs.statorCurrentAmps) > IntakeConstants.kAlgaeAmpFilterThreshold;
      } else {
        detectedGamepiece = true;
      }
    } else {
      System.out.println("INTAKE: selectedGamepiece is invalid");
    }

    if (rollerGoal != null) {
      setRollerVoltage(rollerGoal.getGoalVolts());
      Logger.recordOutput("Intake/RollerGoal", rollerGoal);
    } else {
      Logger.recordOutput("Intake/RollerGoal", "NONE");
    }
  }

  /**
   * Sets the voltage goal of the roller mechanism, logic runs in subsystem periodic method
   * 
   * @param desiredGoal The desired voltage goal
   */
  public void setRollerGoal(RollerGoal desiredGoal) {
    rollerGoal = desiredGoal;
  }

  /**
   * Sets the desired gamepiece of the subsystem, used when checking for if the robot has a
   * gamepiece or not
   * 
   * @param desiredGamepiece The gamepiece type
   */
  public void selectGamepiece(Gamepiece desiredGamepiece) {
    if (desiredGamepiece != null) {
      selectedGamepiece = desiredGamepiece;
    } else {
      System.out.println("INTAKE: Invalid gamepiece selected");
      selectedGamepiece= Gamepiece.kCoral;
    }
  }

  /** Stops the motor */
  public void stop(boolean stopRollers) {
    if(stopRollers){
      rollerGoal = null;
      kRollerHardware.stop();
    }
  }

  /**
   * Sets the voltage of the roller motor
   * 
   * @param voltage
   */
  public void setRollerVoltage(double voltage) {
    kRollerHardware.setVoltage(voltage);
  }

  /**
   * The subsystem runs a linear filter that accepts the motor's current and 
   * compares the moving average against a threshold. If that threshold is exceeded, 
   * it is very likely that the motor has a gamepiece (or is jammed)
   * 
   * @return If the motor thinks it has a gamepiece
   */
  @AutoLogOutput(key = "Intake/detectedGamepiece")
  public boolean detectedGamepiece() {
    return detectedGamepiece;
  }
}