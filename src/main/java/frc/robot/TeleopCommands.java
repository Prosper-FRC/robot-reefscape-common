
package frc.robot;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Gamepiece;
import frc.robot.subsystems.intake.Intake.PivotGoal;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.Climb.ClimbVoltageGoal;
import frc.robot.utils.debugging.LoggedTunableNumber;

/**
 * <p> A commands factory for the teleoperated period. 
 * 
 * <p> Note that when creating a command it should just be that base command, with no 
 * decorators or "special logic" unless it is absolutely needed. Generally speaking, 
 * that logic should be handled by the caller.
 * 
 * <p> While it is not necessary to create a command that always has an action to run
 * when its end condition is invoked or is interrupted, it is generally preferred to have
 * one in the event the caller does not specify what the command should do if it is
 * interrupted
 */
public class TeleopCommands {
    private final Elevator kElevator;
    private final Intake kIntake;
    private final Climb kClimb;

    /** 
     * Internal state to decide whether or not to stop the rollers when the intake's 
     * stop method is invoked. When creating a command that requires the rollers, this
     * variable should be set to true when the rollers are running then set to false
     * when they should no longer run
     */
    private boolean stopRollers = false;
    /** 
     * Internal state to decide whether or not to stop the algae picker when the intake's 
     * stop method is invoked. When creating a command that requires the algae picker, this
     * variable should be set to true when the algae picker is running then set to false
     * when it should no longer run
     */
    private boolean stopPivot = false;

    /**
     * Creates a new TeleopCommands factory
     * 
     * @param elevator The elevator subsystem instance
     * @param intake The intake subsystem intance
     * @param climb The climb subsystem instance
     */
    public TeleopCommands(Elevator elevator, Intake intake, Climb climb) {
        kElevator = elevator;
        kIntake = intake;
        kClimb = climb;
    }

    /**
     * Runs the elevator to a given position, it will then hold what ever position it is
     * at when this command ends. This command should be decorated with an end condition
     * specified by the caller
     * 
     * @param elevatorGoal The elevator goal
     * @return The command to move the elevator and then hold its current position
     */
    public Command runElevatorAndHoldCommand(ElevatorGoal elevatorGoal) {
        return Commands.startEnd(
            () -> kElevator.setGoal(elevatorGoal), 
            () -> kElevator.setPosition(kElevator.getPositionMeters()), 
            kElevator);
    }

    /**
     * Runs the rollers and then stops them, this command should be decorated with an end
     * condition specified by the caller
     * 
     * @param rollerGoal The intake rollers goal
     * @return The command to start and stop the intake rollers
     */
    public Command runRollersAndStopCommand(RollerGoal rollerGoal) {
        return Commands.startEnd(
            () -> {
                stopRollers = false;
                kIntake.setRollerGoal(rollerGoal);
            }, 
            () -> {
                stopRollers = true;
                kIntake.stop(stopRollers, stopPivot);
            },
            kIntake);
    }

    /**
     * Runs the elevator at the specified voltage, this command should be decorated with an
     * end condition specified by the caller
     * 
     * @param voltage The voltage the elevator motor will be set to
     * @return The command to start and stop the elevator at a specified voltage
     */
    public Command runElevatorVoltage(double voltage) {
        return Commands.startEnd(
            () -> {
                kElevator.setVoltage(voltage);
            }, 
            () -> {
                kElevator.stop();
            }, 
            kElevator);
    }

    public Command runTunableVoltage() {
        LoggedTunableNumber voltageTunable = 
            new LoggedTunableNumber("Elevator/VariableVoltage", 3.0);

        return new FunctionalCommand(
            () -> {},
            () -> {
                double voltage = voltageTunable.get();
                kElevator.setVoltage(voltage);
            },
            (interrupted) -> {
                kElevator.stop();
            },
            () -> false,
            kElevator);    
        }

    /**
     * Runs the intake rollers when a trigger that is passed in is set to true. It will continuously
     * check against that trigger and run the rollers if the trigger is true until the command is 
     * cancelled
     * 
     * @param rollerGoal The intake rollers goal
     * @param confirmScoreTrigger The trigger that dictates if it should run the rollers or not
     * @return The command to run the intake rollers when a trigger is true
     */
    public Command runRollersWhenConfirmed(RollerGoal rollerGoal, Trigger confirmScoreTrigger) {
        return Commands.run(
            () -> {
                if (confirmScoreTrigger.getAsBoolean()) {
                    stopRollers = false;
                    kIntake.setRollerGoal(rollerGoal);
                } else {
                    stopRollers = true;
                    kIntake.stop(stopRollers, stopPivot);
                }
            }, 
            kIntake);
    }

    /**
     * Runs the algae picker pivot and then stops it, this command should be decorated with an 
     * end condition specified by the caller
     * 
     * @param pivotGoal The algae picker pivot goal
     * @return The command to start and stop the algae picker pivot
     */
    public Command runPivotAndStopCommand(PivotGoal pivotGoal) {
        return Commands.startEnd(
            ()-> {
                stopPivot = false;
                kIntake.setPivotGoal(pivotGoal);
            }, 
            () -> {
                stopPivot = true;
                kIntake.stop(stopRollers, stopPivot);
            }, 
            kIntake);
    }

    /**
     * Runs the algae picker pivot and then stops it as well as the rollers, this command should 
     * be decorated with an end condition specified by the caller
     * 
     * @param pivotGoal The algae picker pivot goal
     * @return The command to start the algae picker pivot and stop the entire intake
     */
    public Command runPivotAndStopIntakeCommand(PivotGoal pivotGoal) {
        return Commands.startEnd(
            ()-> {
                stopPivot = false;
                kIntake.setPivotGoal(pivotGoal);
            }, 
            () -> {
                stopPivot = true;
                stopRollers = true;
                kIntake.stop(stopRollers, stopPivot);
            }, 
            kIntake);
    }

    /**
     * Runs the climb at a specified goal voltage and then stops it, this command should be
     * deocrated with an end condition specified by the caller
     * 
     * @param climbVoltageGoal The climb's voltage goal
     * @return The command to start the climb and stop it when ended
     */
    public Command runClimbVoltage(ClimbVoltageGoal climbVoltageGoal) {
        return Commands.startEnd(
            () -> kClimb.setGoalVoltage(climbVoltageGoal), 
            () -> kClimb.stop(), 
            kClimb);
    }

    /**
     * Stops the intake rollers. This will also stop the algae picker pivot if the
     * stopPivot variable is set to true
     * 
     * @return The command to stop the rollers that runs once
     */
    public Command stopRollersCommand() {
        // Note that the state must be set via command and not in method since the method
        // only returns an instance of the command and does not run its internal logic
        return setStopRollersStateCommand(true)
            .andThen(
                Commands.runOnce(() -> kIntake.stop(stopRollers, stopPivot), kIntake));
    }

    /**
     * Stops the algae picker pivot. This will also stop the rollers if the stopRollers
     * internal variable is set to true
     * 
     * @return The command to stop the algae picker pivot that runs once
     */
    public Command stopPivotCommand() {
        // Note that the state must be set via command and not in method since the method
        // only returns an instance of the command and does not run its internal logic
        return setStopPivotStateCommand(true)
            .andThen(
                Commands.runOnce(() -> kIntake.stop(stopRollers, stopPivot), kIntake));
    }

    /**
     * Stops the elevator
     * 
     * @return The command to stop the elevator that runs once
     */
    public Command stopElevatorCommand() {
        return Commands.runOnce(() -> kElevator.stop(), kElevator);
    }

    /**
     * Stops the climb
     * 
     * @return The command to stop the climb that runs once
     */
    public Command stopClimbCommand() {
        return Commands.runOnce(() -> kClimb.stop(), kClimb);
    }

    /**
     * Selects the gamepiece for the intake, this is important for gamepiece detection
     * when in real mode
     * 
     * @param gamepiece The selected gamepiece
     * @return The command to select the gamepiece
     */
    public Command selectGamepieceCommand(Gamepiece gamepiece) {
        return Commands.runOnce(() -> kIntake.selectGamepiece(gamepiece), kIntake);
    }

    /**
     * Since changing the state requires a command to be scheduled and ran, this method
     * returns a command to change the rollers state
     * 
     * @param stopRollersState The desired state
     * @return The command to chagne the rollers state
     */
    private Command setStopRollersStateCommand(boolean stopRollersState) {
        return Commands.runOnce(() -> stopRollers = stopRollersState);
    }

    /**
     * Since changing the state requires a command to be scheduled and ran, this method
     * returns a command to change the pivot state
     * 
     * @param stopRollersState The desired state
     * @return The command to chagne the pivot state
     */
    private Command setStopPivotStateCommand(boolean stopPivotState) {
        return Commands.runOnce(() -> stopPivot = stopPivotState);
    }

    /*
    return new FunctionalCommand(
        () -> {},
        () -> {},
        (interrupted) -> {},
        () -> false,
        elevator);
     */
}
