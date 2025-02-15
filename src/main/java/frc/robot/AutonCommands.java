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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.utils.math.AllianceFlipUtil;

public class AutonCommands {
    public static final double kCoralIntakeTriggerDistanceMeters = 0.5; 
    public static final double kAlgaeIntakeTriggerDistanceMeters = 0.5; 
    public static final Pose2d kIntakeLeftPlacedHolder = new Pose2d();
    public static final Pose2d kIntakeRightPlacedHolder = new Pose2d();

    public static final Pose2d kAlgaeD = new Pose2d();
    public static final Pose2d kAlgaeE = new Pose2d();

    private SendableChooser<Command> autoChooser;

    private Drive robotDrive;

    public AutonCommands(Drive robotDrive) {
        // store subsystems
        this.robotDrive = robotDrive;

        autoChooser = new SendableChooser<>();

        tryToAddPathToChooser(
            "FirstCoralTest",
            scoreFirstCoralPath("FirstTest", 
            intakeCoralPath("SecondTest",
            scoreCoralPath("ThirdTest", 
            null
        ))));

        tryToAddPathToChooser(
            "FirstAlgaeTest", 
            intakeFirstAlgaePath("FirstTest", 
            intakeAlgaePath("SecondTest",
            scoreAlgaePath("ThirdTest", 
            null
        ))));

        tryToAddPathToChooser(
            "RightCoral", 
            scoreFirstCoralPath("S_SR_EL_C", 
            intakeCoralPath("I_EL_IR_C", 
            scoreCoralPath("S_IR_FR_C", 
            intakeCoralPath("I_FR_IR_C", 
            scoreCoralPath("S_IR_FL_C", 
            intakeCoralPath("I_FL_IR_C", 
            scoreCoralPath("S_IR_AR_C", 
            intakeCoralPath("I_AR_IR_C", 
            null)))))))));

        tryToAddPathToChooser(
            "LeftCoral", 
            scoreFirstCoralPath("S_SL_CR_C", 
            intakeCoralPath("I_CR_IL_C", 
            scoreCoralPath("S_IL_BL_C", 
            intakeCoralPath("I_BL_IL_C", 
            scoreCoralPath("S_IL_BR_C", 
            intakeCoralPath("I_BR_IL_C", 
            scoreCoralPath("S_IL_AL_C", 
            intakeCoralPath("I_AL_IL_C", 
            null)))))))));

        tryToAddPathToChooser(
            "Algae", 
            intakeFirstAlgaePath("I_SM_DM_A",
            scoreAlgaePath("S_DM_P_A", 
            intakeAlgaePath("I_P_EM_A", 
            scoreAlgaePath("S_EM_P_A", 
            null)))));

        autoChooser.setDefaultOption("Mobility", backUpAuton());
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
    public PathPlannerAuto scoreFirstCoralPath(String name, Rotation2d startingRotation, PathPlannerAuto nextAuto) {
        return firstPath(name, startingRotation, () -> !PathPlannerAuto.currentPathName.equals(name), scoreCoralCommand(), nextAuto);
    }

    /* 
     * The first path of the robot, sets pose and rotation of robot 
     * Upon finishing will  score a coral, and have the trigger schedule the nextAuto
    */
    public PathPlannerAuto scoreFirstCoralPath(String name, PathPlannerAuto nextAuto) {
        return firstPath(name, new Rotation2d(), () -> !PathPlannerAuto.currentPathName.equals(name), scoreCoralCommand(), nextAuto);
    }

    /* 
     * The first path of the robot, sets pose and rotation of robot 
     * Upon finishing will  score an algae, and have the trigger schedule the nextAuto
    */
    public PathPlannerAuto intakeFirstAlgaePath(String name, Rotation2d startingRotation, PathPlannerAuto nextAuto) {
        return firstPath(name, startingRotation, () -> !PathPlannerAuto.currentPathName.equals(name), scoreAlgaeCommand(), nextAuto);
    }

    /* 
     * The first path of the robot, sets pose and rotation of robot 
     * Upon finishing will  score an algae, and have the trigger schedule the nextAuto
    */
    public PathPlannerAuto intakeFirstAlgaePath(String name, PathPlannerAuto nextAuto) {
        return firstPath(name, new Rotation2d(), () -> !PathPlannerAuto.currentPathName.equals(name), scoreAlgaeCommand(), nextAuto);
    }

    /* 
     * Upon finishing will score the named path, the coral will be scored
     * and then the trigger schedules the nextAuto
    */
    public PathPlannerAuto scoreCoralPath(String name, PathPlannerAuto nextAuto) {
        return nextPath(name, () -> !PathPlannerAuto.currentPathName.equals(name), scoreCoralCommand(), nextAuto);
    }

    /* 
     * Upon finishing will score the named path, the coral intake sequence will be started
     * and upon finishing then the nextAuto is scheduled
    */
    public PathPlannerAuto intakeCoralPath(String name, PathPlannerAuto nextAuto) {
        PathPlannerAuto auto = nextPath(name, getHasPiece(), intakeCoralCommand(), nextAuto);
        auto.nearFieldPosition(AllianceFlipUtil.apply(FieldConstants.IL).getTranslation(), kCoralIntakeTriggerDistanceMeters).or(
            auto.nearFieldPosition(AllianceFlipUtil.apply(FieldConstants.IR).getTranslation(), kCoralIntakeTriggerDistanceMeters)
        ).whileTrue(
            intakeAlgaeCommand() );
        return auto;
    }

    /* 
     * Upon finishing will score the named path, the algae will be scored
     * and then the trigger schedules the nextAuto
    */
    public PathPlannerAuto scoreAlgaePath(String name, PathPlannerAuto nextAuto) {
        return nextPath(name, () -> !PathPlannerAuto.currentPathName.equals(name), scoreAlgaeCommand(), nextAuto);
    }

    /* 
     * Upon finishing will score the named path, the algae intake sequence will be started
     * and upon finishing then the nextAuto is scheduled
    */
    public PathPlannerAuto intakeAlgaePath(String name, PathPlannerAuto nextAuto) {
        PathPlannerAuto auto = nextPath(name, getHasPiece(), intakeCoralCommand(), nextAuto);
        auto.nearFieldPosition(AllianceFlipUtil.apply(FieldConstants.DM).getTranslation(), kAlgaeIntakeTriggerDistanceMeters).or(
            auto.nearFieldPosition(AllianceFlipUtil.apply(FieldConstants.EM).getTranslation(), kAlgaeIntakeTriggerDistanceMeters)
        ).whileTrue(
            intakeCoralCommand() );
        return auto;
    }

    ///////////////// PATH CHAINING LOGIC \\\\\\\\\\\\\\\\\\\\\\
    public PathPlannerAuto firstPath(String name, Rotation2d startingRotation, BooleanSupplier conditionSupplier, Command nextCommand, PathPlannerAuto nextAuto) {
        PathPlannerAuto firstAuto = new PathPlannerAuto(followFirstChoreoPath(name, startingRotation));

        firstAuto.condition(conditionSupplier).onTrue(nextCommand.andThen(nextAutoChecker(nextAuto)));

        return firstAuto;
    }

    public PathPlannerAuto nextPath(String name, BooleanSupplier conditionSupplier, Command nextCommand, PathPlannerAuto nextAuto) {
        PathPlannerAuto auto = new PathPlannerAuto(followChoreoPath(name));

        auto.condition(conditionSupplier).onTrue(nextCommand.andThen(nextAutoChecker(nextAuto)));

        return auto;
    }

    public Command nextAutoChecker(PathPlannerAuto auto) {
        return (auto == null) ? robotDrive.setDriveStateCommand(Drive.DriveState.STOP) : auto;
    }

    public Command backUpAuton() {
        return new InstantCommand();
    }

    ///////////////// SUPERSTRUCTURE COMMANDS AND DATA \\\\\\\\\\\\\\\\\\\\\
    public Command scoreCoralCommand() {
        return new PrintCommand("Score Coral");
    }

    public Command intakeCoralCommand() {
        return new PrintCommand("intake Coral");
    }

    public Command scoreAlgaeCommand() {
        return new PrintCommand("Score Algae");
    }

    public Command intakeAlgaeCommand() {
        return new PrintCommand("Intake Algae");
    }

    public BooleanSupplier getHasPiece() {
        return () -> false;
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
            AutoBuilder.followPath(path).withTimeout(totalTimeSeconds + 0.5), 
            robotDrive.setDriveStateCommand(DriveState.STOP));
    }

    public Command followChoreoPath(String pathName) {
        PathPlannerPath path = getTraj(pathName).get();
        path.getIdealTrajectory(Drive.robotConfig);
        double totalTimeSeconds = path.getIdealTrajectory(Drive.robotConfig).get().getTotalTimeSeconds();
        return 
            robotDrive.setDriveStateCommand(DriveState.AUTON).andThen(
                AutoBuilder.followPath(path).withTimeout(totalTimeSeconds + 0.5), 
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
}
