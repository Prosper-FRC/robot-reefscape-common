package frc.robot;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser.SIDE;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorGoal;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Gamepiece;
import frc.robot.subsystems.intake.Intake.RollerGoal;
import frc.robot.utils.VirtualSubsystem;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.math.AllianceFlipUtil;

public class AutonCommands {
    public static final LoggedTunableNumber kCoralIntakeTriggerDistanceMeters = 
        new LoggedTunableNumber("Auto/CoralMeterTrigger", 0.5); 
    public static final LoggedTunableNumber kAlgaeIntakeTriggerDistanceMeters = 
        new LoggedTunableNumber("Auto/AlgaeMeterTrigger", 0.5); 

    private final double kElevatorPositionTimeoutSeconds = 2.5;
    private final double kScoreCoralTimeoutSeconds = 0.5;

    // private final double kElevatorPositionTimeoutSeconds = 2.5;
    // private final double kScoreCoralTimeoutSeconds = 0.75;

    private final double kIntakeCoralTimeoutSeconds = 2.5;

    private final double kAutoAlignActivationDistance = 1.5;

    private SendableChooser<Command> autoChooser;

    private Drive robotDrive;
    private Elevator mElevator;
    private Intake mIntake;

    private VirtualSubsystem virtualElevator= new VirtualSubsystem();
    private VirtualSubsystem virtualIntake= new VirtualSubsystem();

    public AutonCommands(Drive robotDrive, Elevator elevator, Intake intake) {
        // store subsystems
        this.robotDrive = robotDrive;
        mElevator = elevator;
        mIntake = intake;

        autoChooser = new SendableChooser<>();

        tryToAddPathToChooser(
            "OnePieceStationary", 
            scoreCoralCommand());

        tryToAddPathToChooser(
            "CriticalTest", 
            scoreFirstCoralPath("S_SL_CR_C", 
            intakeCoralPath("I_CR_IL_C", 
            scoreCoralPath("S_IL_BL_C", 
            null))));

        tryToAddPathToChooser(
            "FirstCoralTest",
            scoreFirstCoralPath("FirstTest", 
            intakeCoralPath("SecondTest",
            scoreCoralPath("ThirdTest", 
            null))));

        tryToAddPathToChooser(
            "FirstAlgaeTest(DONTUSE)", 
            intakeFirstAlgaePath("FirstTest", 
            intakeAlgaePath("SecondTest",
            scoreAlgaePath("I_FR_IR_C", 
            scoreCoralPath("S_IR_FL_C", 
            intakeCoralPath("I_FL_IR_C", 
            scoreCoralPath("S_IR_AR_C", 
            intakeCoralPath("I_AR_IR_C", 
            null))))))));

            // Auto Align testing
        tryToAddPathToChooser(
            "RightCoral", 
            scoreFirstCoralPath("S_SR_EL_C", 
            intakeCoralPath("I_EL_IR_C",
            scoreCoralPath("S_IR_FL_C", 
            intakeCoralPath("I_FL_IR_C",
            scoreCoralPath("S_IR_FR_C",
            intakeCoralPath("I_FR_IR_C", 
            scoreCoralPath("S_IR_AR_C",
            null))))))));

        tryToAddPathToChooser(
            "LeftCoral", 
            scoreFirstCoralPath("S_SL_CR_C", 
            intakeCoralPath("I_CR_IL_C", 
            scoreCoralPath("S_IL_BR_C", 
            intakeCoralPath("I_BR_IL_C", 
            scoreCoralPath("S_IL_BL_C", 
            intakeCoralPath("I_BL_IL_C", 
            scoreCoralPath("S_IL_AL_C", 
            intakeCoralPath("I_AL_IL_C", 
            null)))))))));

        tryToAddPathToChooser(
            "CenterCoral", 
            scoreFirstCoralPath("S_SM_DL_C", null));

        tryToAddPathToChooser(
            "Algae(DONTUSE)", 
            intakeFirstAlgaePath("I_SM_DM_A",
            scoreAlgaePath("S_DM_P_A", 
            intakeAlgaePath("I_P_EM_A", 
            scoreAlgaePath("S_EM_P_A", 
            null)))));

        autoChooser.setDefaultOption("Stationary", backUpAuton());
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public void tryToAddPathToChooser(String pathName, Command command) {
        tryToAddPathToChooser(pathName, new Runnable() {
            @Override
            public void run() {
                autoChooser.addOption(pathName, command);
            }
        });
    }  
    
    /* Stops magic auton errors from occuring due to FMS or some BS I cook up */
    public void tryToAddPathToChooser(String pathName, Runnable pathAdding) {
        try {
            pathAdding.run();
        } catch(Exception e) {
            autoChooser.addOption("Failed: "+pathName, backUpAuton());
        }
    }

    public SendableChooser<Command> getAutoChooser() {
        return autoChooser;
    }

    /* 
     * The first path of the robot, sets pose and rotation of robot 
     * Upon finishing will  score a coral, and have the trigger schedule the nextAuto
    */
    public Command scoreFirstCoralPath(String name, Rotation2d startingRotation, PathPlannerAuto nextAuto) {
        return new SequentialCommandGroup(
            GoalPoseChooser.setSideCommand(getSide(name)),
            new ParallelCommandGroup(
                //new InstantCommand(() ->mElevator.setGoal(ElevatorGoal.kL3Coral)),
                firstPath(
                    name, 
                    new Rotation2d(), 
                    () -> !PathPlannerAuto.currentPathName.equals(name), //|| robotDrive.distanceFromReefCenter() < kAutoAlignActivationDistance, 
                    robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_CORAL).withDeadline(
                        robotDrive.waitUnitllReefAutoAlignFinishes()).andThen(
                        scoreCoralCommand()), 
                    nextAutoChecker(nextAuto))));
    }

    /* 
     * The first path of the robot, sets pose and rotation of robot 
     * Upon finishing will  score a coral, and have the trigger schedule the nextAuto
    */
    public Command scoreFirstCoralPath(String name, Command nextAuto) {
        return new SequentialCommandGroup(
            GoalPoseChooser.setSideCommand(getSide(name)),
            new ParallelCommandGroup(
                new InstantCommand(() ->mElevator.setGoal(ElevatorGoal.kL1Coral))
                    .andThen(Commands.waitUntil(() -> robotDrive.distanceFromReefCenter() < 2.55 )
                    .andThen(new InstantCommand(() ->mElevator.setGoal(ElevatorGoal.kL4Coral)))),
                firstPath(
                    name, 
                    new Rotation2d(), 
                    () -> !PathPlannerAuto.currentPathName.equals(name), //|| robotDrive.distanceFromReefCenter() < kAutoAlignActivationDistance, 
                    robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_CORAL).withDeadline(
                        robotDrive.waitUnitllReefAutoAlignFinishes()).andThen(
                        scoreCoralCommand()), 
                    nextAutoChecker(nextAuto))));
    }

    /* 
     * The first path of the robot, sets pose and rotation of robot 
     * Upon finishing will  score an algae, and have the trigger schedule the nextAuto
    */
    public PathPlannerAuto intakeFirstAlgaePath(String name, Rotation2d startingRotation, PathPlannerAuto nextAuto) {
        PathPlannerAuto auto = firstPath(name, startingRotation, getHasPiece(), intakeAlgaeCommand(), nextAuto);
        auto.nearFieldPosition(AllianceFlipUtil.apply(FieldConstants.DM).getTranslation(), kAlgaeIntakeTriggerDistanceMeters.get()).or(
            auto.nearFieldPosition(AllianceFlipUtil.apply(FieldConstants.EM).getTranslation(), kAlgaeIntakeTriggerDistanceMeters.get())
        ).whileTrue(
            intakeAlgaeCommand() );
        return auto;
    }

    /* 
     * The first path of the robot, sets pose and rotation of robot 
     * Upon finishing will  score an algae, and have the trigger schedule the nextAuto
    */
    public PathPlannerAuto intakeFirstAlgaePath(String name, Command nextAuto) {
        return firstPath(name, new Rotation2d(), () -> !PathPlannerAuto.currentPathName.equals(name), scoreAlgaeCommand(), nextAuto);
    }

    /* 
     * Upon finishing will score the named path, the coral will be scored
     * and then the trigger schedules the nextAuto
    */
    public Command scoreCoralPath(String name, Command nextAuto) {
        return new SequentialCommandGroup(
            GoalPoseChooser.setSideCommand(getSide(name)),
            new ParallelCommandGroup(   
                new InstantCommand(() ->mElevator.setGoal(ElevatorGoal.kL1Coral))
                    .andThen(Commands.waitUntil(() -> robotDrive.distanceFromReefCenter() < 3.0 )
                    .andThen(new InstantCommand(() ->mElevator.setGoal(ElevatorGoal.kL4Coral)))),
                nextPath(
                    name, 
                    () -> !PathPlannerAuto.currentPathName.equals(name), //|| robotDrive.distanceFromReefCenter() < kAutoAlignActivationDistance, 
                    robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_CORAL)
                        .withDeadline(robotDrive.waitUnitllReefAutoAlignFinishes())
                    .andThen(scoreCoralCommand()), 
                    nextAutoChecker(nextAuto))));
    }

    /* 
     * Upon finishing will score the named path, the coral intake sequence will be started
     * and upon finishing then the nextAuto is scheduled
    */
    public Command intakeCoralPath(String name, Command nextAuto) {
        return new SequentialCommandGroup(
            new ParallelCommandGroup(
                new InstantCommand(() ->mElevator.setGoal(ElevatorGoal.kStow)),
                nextPath(
                    name, 
                    () -> !PathPlannerAuto.currentPathName.equals(name), 
                        robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_INTAKE)
                            .withDeadline(robotDrive.waitUnitllIntakeAutoAlignFinishes()).andThen(
                        intakeCoralCommand()),
                    nextAuto)));
    }

    /* 
     * Upon finishing will score the named path, the algae will be scored
     * and then the trigger schedules the nextAuto
    */
    public Command scoreAlgaePath(String name, Command nextAuto) {
        return new SequentialCommandGroup(
            nextPath(name, () -> !PathPlannerAuto.currentPathName.equals(name), scoreAlgaeCommand(), nextAuto),
            robotDrive.setDriveStateCommandContinued(DriveState.DRIVE_TO_ALGAE));
    }

    /* 
     * Upon finishing will score the named path, the algae intake sequence will be started
     * and upon finishing then the nextAuto is scheduled
    */
    public Command intakeAlgaePath(String name, Command nextAuto) {
        PathPlannerAuto auto = nextPath(name, getHasPiece(), intakeAlgaeCommand(), nextAuto);
        auto.nearFieldPosition(AllianceFlipUtil.apply(FieldConstants.DM).getTranslation(), kAlgaeIntakeTriggerDistanceMeters.get()).or(
            auto.nearFieldPosition(AllianceFlipUtil.apply(FieldConstants.EM).getTranslation(), kAlgaeIntakeTriggerDistanceMeters.get())
        ).whileTrue(
            intakeAlgaeCommand() );
        return auto;
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public PathPlannerAuto firstPath(String name, Rotation2d startingRotation, BooleanSupplier conditionSupplier, Command nextCommand, Command nextAuto) {
        PathPlannerAuto firstAuto = new PathPlannerAuto(followFirstChoreoPath(name, startingRotation));
        firstAuto.condition(conditionSupplier).onTrue(nextCommand.andThen(Commands.runOnce(() -> nextAutoChecker(nextAuto).schedule())));
        return firstAuto;
    }

    public PathPlannerAuto nextPath(String name, BooleanSupplier conditionSupplier, Command nextCommand, Command nextAuto) {
        PathPlannerAuto auto = new PathPlannerAuto(followChoreoPath(name));
        auto.condition(conditionSupplier).onTrue(nextCommand.andThen(Commands.runOnce(() -> nextAutoChecker(nextAuto).schedule())));
        return auto;
    }

    public Command nextAutoChecker(Command auto) {
        return (auto == null) ? robotDrive.setDriveStateCommand(Drive.DriveState.STOP) : auto;
    }

    public Command backUpAuton() {
        return new InstantCommand();
    }

    ///////////////// SUPERSTRUCTURE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\
    public Command scoreCoralCommand() {
        Command command = new SequentialCommandGroup(
            new FunctionalCommand(
                () -> {
                    mElevator.setGoal(ElevatorGoal.kL4Coral);
                }, 
                () -> {}, 
                (interrupted) -> {
                    mElevator.setPosition(mElevator.getPositionMeters());
                }, 
                getElevatorAtGoal(),
                virtualElevator)
                .withTimeout(kElevatorPositionTimeoutSeconds),
            new FunctionalCommand(
                () -> {
                    mIntake.setRollerGoal(RollerGoal.kAutonScoreCoral);
                }, 
                () -> {}, 
                (interrupted) -> {
                    mIntake.stop(true, false);
                }, 
                () -> false,
                virtualIntake)
                .withTimeout(kScoreCoralTimeoutSeconds)
            // new FunctionalCommand(
            //     () -> {
            //         mElevator.setGoal(ElevatorGoal.kStow);
            //     }, 
            //     () -> {}, 
            //     (interrupted) -> {}, 
            //     () -> Math.abs(mElevator.getErrorMeters()) < 1.0,
            //     virtualElevator)
            //     .withTimeout(kElevatorPositionTimeoutSeconds)
        );

        return command;
    }

    public Command elevatorToStowCommand() {
        // return new FunctionalCommand(
        //     () -> {
        //         mElevator.setGoal(ElevatorGoal.kStow);
        //     }, 
        //     () -> {}, 
        //     (interrupted) -> {
        //         mElevator.stop();
        //     }, 
        //     getElevatorAtGoal(),
        //     virtualElevator)
        //     .withTimeout(kElevatorPositionTimeoutSeconds);
        return Commands.runOnce(() -> mElevator.setGoal(ElevatorGoal.kStow));
    }

    public Command elevatorToL2Command() {
        // return new FunctionalCommand(
        //     () -> {
        //         mElevator.setGoal(ElevatorGoal.kStow);
        //     }, 
        //     () -> {}, 
        //     (interrupted) -> {
        //         mElevator.stop();
        //     }, 
        //     getElevatorAtGoal(),
        //     virtualElevator)
        //     .withTimeout(kElevatorPositionTimeoutSeconds);
        return Commands.runOnce(() -> mElevator.setGoal(ElevatorGoal.kL205Coral));
    }

    public Command intakeCoralCommand() {
        return Commands.startEnd(
            () -> mIntake.setRollerGoal(RollerGoal.kIntakeCoral), 
            () -> mIntake.stop(true, false), 
            mIntake).onlyWhile(() -> getHasPiece().getAsBoolean())
            .beforeStarting(Commands.runOnce(() -> mIntake.selectGamepiece(Gamepiece.kCoral)));
    }

    public Command scoreAlgaeCommand() {
        return new PrintCommand("Score Algae");
    }

    public Command intakeAlgaeCommand() {
        return new PrintCommand("Intake Algae");
    }

    public BooleanSupplier getHasPiece() {
        return () -> !mIntake.detectedGamepiece();
    }

    public BooleanSupplier getElevatorAtGoal() {
        return () -> mElevator.atGoal();
    }

    ///////////////// PATH CREATION LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public Command followFirstChoreoPath(String pathName, Rotation2d startingRotation) {
        PathPlannerPath path = getTraj(pathName).get();
        double totalTimeSeconds = path.getIdealTrajectory(Drive.robotConfig).get().getTotalTimeSeconds();

        return new SequentialCommandGroup(
            new InstantCommand(() -> {
                robotDrive.setDriveState(DriveState.AUTON);
                robotDrive.setPose(AllianceFlipUtil.apply(new Pose2d(path.getPathPoses().get(0).getTranslation(), startingRotation)));
            }), 
            AutoBuilder.followPath(path).withTimeout(totalTimeSeconds), 
            robotDrive.setDriveStateCommand(DriveState.STOP));
    }

    public Command followChoreoPath(String pathName) {
        PathPlannerPath path = getTraj(pathName).get();
        path.getIdealTrajectory(Drive.robotConfig);
        double totalTimeSeconds = path.getIdealTrajectory(Drive.robotConfig).get().getTotalTimeSeconds();
        return 
            robotDrive.setDriveStateCommand(DriveState.AUTON).andThen(
                AutoBuilder.followPath(path).withTimeout(totalTimeSeconds), 
                robotDrive.setDriveStateCommand(DriveState.STOP));
    }

    public Optional<PathPlannerPath> getTraj(String pathName) {
        try {
            return Optional.of(PathPlannerPath.fromChoreoTrajectory(pathName));
        } catch(Exception e) {
            e.printStackTrace();
            return Optional.empty();
        }
    }

    public SIDE getSide(String name){
        String n = name.substring(6, 7);
        return (n.equals("L")) ? SIDE.LEFT : SIDE.RIGHT;
    }
}