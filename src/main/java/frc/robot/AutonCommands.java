package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class AutonCommands {

    // Define target poses for path planner (location and orientation)
    // ex: public static final Pose2d A1 = new Pose2d(2.55, 6.550, Rotation2d.fromRadians(0.464));

    // Define local storage of subsystems

    public static enum AutonState {
        ALLIANCE,
        CENTERLINE
    }

    @AutoLogOutput
    private AutonState autoState = AutonState.ALLIANCE;

    private SendableChooser<Command> autoChooser;

    Drive drive;

    public AutonCommands(Drive drive) {
        // store subsystems
        this.drive = drive;

        autoChooser = AutoBuilder.buildAutoChooser();

        // Define Auton choices for the dashboard.  Add a named option, and give it a method of this class to run.

        /* Example:
        autoChooser.setDefaultOption("SpeakerShot", runDefaultCommand());

        autoChooser.addOption("S2C3-5piece", runS2C35Piece());
        */

    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    // Write a method for each atomic action the robot might take.
    // Also write methods that string these actions together into larger actions with SequentialCommandGroup, etc.

    /* Examples:
    public Command runS2C35Piece() {
        return new SequentialCommandGroup(
            runS24Piece(),
            runA3ToC3(),
            runC3ToF2()
        );
    }

    public Command runA3ToC3() {
        return generalCenterLine("A3-C3");
    }

    public Command shoot() {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                robotArm.setGoalCommand(ArmGoal.AUTO_AIM),
                robotDrive.setDriveStateCommandContinued(DriveState.AUTO_HEADING),
                robotFlywheels.setGoalCommand(FlywheelSetpoint.SHOOT))
                    .deadlineWith(Commands.waitUntil(()->
                robotArm.inTolerance() && robotFlywheels.inTolerance() && robotDrive.inHeadingTolerance()))
                    .withTimeout(0.75),
            robotIntake.setGoalCommand(IndexerIntakeVoltageGoal.SHOOT)
        );
    }

    */

    // Helper functions can be written to make defining commands easier, and to avoid code duplication.
    /* Example
    public Command generalStartAuton(String path, Rotation2d startingRotation) {
        return new SequentialCommandGroup(
            speakerShoot(),
            //new PrintCommand("\n\n\n\n\n\n\n\n\n4?\n\n\n\n\n\n\n\n\n"),
            Commands.waitSeconds(0.2),
            //new PrintCommand("\n\n\n\n\n\n\n\n\n5?\n\n\n\n\n\n\n\n\n"),
            pickup().withTimeout(0.1),
            // new PrintCommand("\n\n\n\n\n\n\n\n\n3?\n\n\n\n\n\n\n\n\n"),
            robotDrive.followFirstChoreoPath(path, startingRotation),
            Commands.waitUntil(()->hasNoteInIndexer().getAsBoolean()).withTimeout(0.5),
            shoot(),
            Commands.waitSeconds(0.25),
            pickup());
    }

    public Command generalCenterLine(String path) {
        return new SequentialCommandGroup(
            pickup(),
            robotDrive.followChoreoPath(path),
            Commands.waitUntil(()->hasNoteInIndexer().getAsBoolean()).withTimeout(0.5),
            pickup());
    }

    public Command generalCenterToShoot(String path) {
        return new SequentialCommandGroup(
            robotDrive.followChoreoPath(path),
            shoot(), Commands.waitSeconds(0.25), pickup());
    }

    public Trigger hasNoteInIndexer() {
        return new Trigger(robotIntake::getHasPiece);
    }
    */



}
