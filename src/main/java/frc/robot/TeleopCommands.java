
package frc.robot;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;

public class TeleopCommands {
    private final Elevator kElevator;
    private final Intake kIntake;
    private final Climb kClimb;

    private boolean stopRollers;
    private boolean stopPivot;

    public TeleopCommands(Elevator elevator, Intake intake, Climb climb) {
        kElevator = elevator;
        kIntake = intake;
        kClimb = climb;
    }

    public Command scoreCoralReefCommand(ElevatorGoal level) {
        return runElevatorCommand(level)
            .andThen(
                runRollersCommand(RollerGoal.kScoreCoral));
    }

    public Command intakeCoralCommand() {
        return runElevatorCommand(ElevatorGoal.kIntake)
            .andThen(
                runRollersCommand(RollerGoal.kIntakeCoral));
    }

    public Command runElevatorCommand(ElevatorGoal goal) {
        return Commands.runOnce(() -> kElevator.setGoal(goal), kElevator);
    }

    public Command runRollersCommand(RollerGoal goal) {
        return Commands.runOnce(() -> kIntake.setRollerGoal(goal), kIntake);
    }

    public Command stopRollersCommand() {
        stopRollers = true;
        return Commands.runOnce(() -> kIntake.stop(true, false), kIntake);
    }

    public Command stopElevatorCommand() {
        return Commands.runOnce(() -> kElevator.stop(), kElevator);
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
