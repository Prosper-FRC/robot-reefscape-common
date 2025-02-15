
package frc.robot;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.climb.Climb;

public class TeleopCommands {
    private final Elevator kElevator;
    private final Intake kIntake;
    private final Climb kClimb;

    public TeleopCommands(Elevator elevator, Intake intake, Climb climb) {
        kElevator = elevator;
        kIntake = intake;
        kClimb = climb;
    }

    public Command scoreCoralReefCommand(ElevatorGoal level) {
        return runElevatorCommand(level, kElevator)
            .andThen(
                runRollersCommand(RollerGoal.kScoreCoral, kIntake));
    }

    public Command intakeCoralCommand() {
        return runElevatorCommand(ElevatorGoal.kIntake, kElevator)
            .andThen(
                runRollersCommand(RollerGoal.kIntakeCoral, kIntake));
    }

    public Command runElevatorCommand(ElevatorGoal goal, Elevator elevator) {
        return new FunctionalCommand(
            () -> {
                elevator.setGoal(goal);
            },
            () -> {},
            (interrupted) -> {
                elevator.stop();
            },
            () -> elevator.atGoal(),
            elevator);
    }

    public Command runRollersCommand(RollerGoal goal, Intake intake) {
        return new FunctionalCommand(
            () -> {
                intake.setRollerGoal(goal);
            },
            () -> {},
            (interrupted) -> {
                intake.stop(true, false);
            },
            () -> !intake.detectedGamepiece(),
            intake);
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
