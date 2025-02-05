package frc.robot;

public class TeleopCommands {

    // Define local storage of subsystems
    // ex: private Drive robotDrive;

    public TeleopCommands(/* pass subsystems */) {
        // store subsystems
        // ex: this.robotDrive = robotDrive;
    }

    // Define methods that return commands that will assist the driver and operator
    //  during tele-op mode.  This allows orchestrating movement among the various subsystems
    //  through a combination of ParallelCommandGroup, SequentialCommandGroup, ParallelDeadlineGroup, etc.
    //  These commands will be bound to triggers on the controllers.

    /* Examples:
    public Command aimAmp() {
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                robotFlywheels.setGoalCommand(FlywheelSetpoint.STOP),
                Commands.waitSeconds(0.4),
                robotFlywheels.setGoalCommand(FlywheelSetpoint.AMP)),
            robotArm.setGoalCommand(ArmGoal.AMP)
        );
    }

    public Command aimFeed() {
        return new ParallelCommandGroup(
            robotArm.setGoalCommand(ArmGoal.SPEAKER),
            robotFlywheels.setGoalCommand(FlywheelSetpoint.FEED_SHOOT)
        );
    }

    public Command manualArmUp() {
        return robotArm.setGoalCommand(ArmGoal.MANUAL_UP);
    }

    public Command manualArmDown() {
        return robotArm.setGoalCommand(ArmGoal.MANUAL_DOWN);
    }

    public Command backToIdle() {
        return new ParallelCommandGroup(
            robotIntake.setGoalCommand(IndexerIntakeVoltageGoal.STOP),
            robotArm.setGoalCommand(ArmGoal.IDLE),
            robotFlywheels.setGoalCommand(FlywheelSetpoint.IDLE));
    }
    */
}
