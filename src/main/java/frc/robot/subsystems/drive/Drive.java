package frc.robot.subsystems.drive;

import static frc.robot.subsystems.drive.DriveConstants.*;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.controllers.HeadingController;
import frc.robot.subsystems.drive.controllers.GoalPoseChooser;

import static frc.robot.subsystems.drive.controllers.GoalPoseChooser.*;

import frc.robot.subsystems.drive.controllers.ManualTeleopController;
import frc.robot.subsystems.drive.controllers.HolonomicController;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.VisionObservation;
import frc.robot.utils.debugging.LoggedTunableNumber;
import frc.robot.utils.debugging.SysIDCharacterization;
import frc.robot.utils.math.AllianceFlipUtil;
import frc.robot.utils.math.GeomUtil;
import frc.robot.utils.ppMath.SwerveSetpoint;
import frc.robot.utils.ppMath.SwerveSetpointGenerator;
import frc.robot.utils.swerve.LocalADStarAK;
import frc.robot.utils.swerve.SwerveUtils;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/* This code is a swerve drivebase 
 * Main logic is handled in periodic() function
*/
public class Drive extends SubsystemBase {
    public static enum DriveState {
        // TELEOP AND AUTON CONTROLS
        TELEOP,
        TELEOP_SNIPER,
        POV_SNIPER,
        HEADING_ALIGN,
        AUTON, 
        AUTO_ALIGN,
        STOP,
        // TESTS
        DRIFT_TEST,
        LINEAR_TEST,
        SYSID_CHARACTERIZATION,
        WHEEL_CHARACTERIZATION
    }

    /* HARDWARE LAYERS */
    private Module[] modules;
    private GyroIO gyro;
    private GyroInputsAutoLogged gyroInputs = new GyroInputsAutoLogged();
    private Vision vision;

    /* LOCALIZATION */
    private Rotation2d robotRotation;
    private SwerveDriveOdometry odometry;
    private SwerveDrivePoseEstimator poseEstimator;
    private Field2d field = new Field2d();

    /* PATHPLANNER AND SETPOINT GENERATOR(USED IN TELEOP */
    public static RobotConfig robotConfig;
    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint = new SwerveSetpoint(
        new ChassisSpeeds(), SwerveUtils.zeroStates(), DriveFeedforwards.zeros(4));

    /* STATE OF DRIVEBASE */
    @AutoLogOutput(key="Drive/State")
    private DriveState driveState = DriveState.TELEOP;
    private boolean useGenerator = true;

    /* SETPOINTS */
    private ChassisSpeeds desiredSpeeds = new ChassisSpeeds();
    private ChassisSpeeds ppDesiredSpeeds = new ChassisSpeeds();
    private DriveFeedforwards pathPlanningFF = DriveFeedforwards.zeros(4);

    /* CONTROLLERS */
    private ManualTeleopController teleopController = new ManualTeleopController();

    private HeadingController headingController = new HeadingController();
    @AutoLogOutput(key="Drive/HeadingController/HeadingGoal")
    private Rotation2d headingGoal = new Rotation2d();

    private HolonomicController autoAlignController = new HolonomicController();
    private Pose2d goalPose = new Pose2d();

    /* TUNABLE NUMBERS FOR DRIVEBASE CONSTANTS AND TESTS */
    private final static LoggedTunableNumber kDriftRate = new LoggedTunableNumber("Drive/DriftRate", DriveConstants.kDriftRate);
    private final static LoggedTunableNumber kRotationDriftTestSpeed = new LoggedTunableNumber("Drive/DriftRotationTest", 360);
    private final static LoggedTunableNumber kLinearTestSpeed = new LoggedTunableNumber("Drive/DriftLinearTest", 4.5);

    public Drive(Module[] modules, GyroIO gyro, Vision vision) {
        this.modules = modules;
        this.gyro = gyro;
        this.vision = vision;

        robotRotation = gyroInputs.yawPosition;

        odometry = new SwerveDriveOdometry(kKinematics, getRobotRotation(), getModulePositions());
        poseEstimator = new SwerveDrivePoseEstimator(kKinematics, getRobotRotation(), getModulePositions(), new Pose2d());

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            e.printStackTrace();
        }

        setpointGenerator = new SwerveSetpointGenerator(
            robotConfig, // The robot configuration. This is the same config for pathplanner as well
            kMaxAzimuthAngularRadiansPS // The max rotation velocity of a swerve module in radians per second.
        );

        AutoBuilder.configure(
            this::getPoseEstimate, 
            this::setPose, 
            this::getChassisSpeeds, 
            (speeds, ff) -> {
                ppDesiredSpeeds = speeds;
                pathPlanningFF = ff;
            }, 
            new PPHolonomicDriveController( kPPTranslationPID, kPPRotationPID ), 
            robotConfig, 
            () -> DriverStation.getAlliance().isPresent() && 
                DriverStation.getAlliance().get() == Alliance.Red, this);

        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> Logger.recordOutput(
        "Drive/Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> Logger.recordOutput(
        "Drive/Odometry/TrajectorySetpoint", targetPose));

        SmartDashboard.putData(field);

        headingController.setHeadingGoal(() -> headingGoal);
    }

    @Override
    public void periodic() {
        ///////////////////// DATA COLLECTION AND UPDATE CONTROLLERS AND ODOMETRY \\\\\\\\\\\\\\\\\\
        /* Modules */
        for (Module module : modules) {
            module.periodic();
            if (DriverStation.isDisabled()) module.stop();
        }

        /* GYRO */
        gyro.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        if (gyroInputs.connected) {
            robotRotation = gyroInputs.yawPosition;
        } else {
            robotRotation = Rotation2d.fromRadians(
                (poseEstimator.getEstimatedPosition().getRotation().getRadians()
                    + getChassisSpeeds().omegaRadiansPerSecond * 0.02) % 360.0);
        }

        /* VISION */
        vision.periodic(poseEstimator.getEstimatedPosition(), odometry.getPoseMeters());
        VisionObservation[] observations = vision.getVisionObservations();
        for(VisionObservation observation : observations) {
            if(observation.hasObserved()) poseEstimator.addVisionMeasurement(
                observation.pose(), observation.timeStamp(), observation.stdDevs());

            Logger.recordOutput(observation.camName()+"/stdDevX", observation.stdDevs().get(0));
            Logger.recordOutput(observation.camName()+"/stdDevY", observation.stdDevs().get(1));
            Logger.recordOutput(observation.camName()+"/stdDevTheta", observation.stdDevs().get(2));
            Logger.recordOutput(observation.camName()+"/TransformFromOdometry", odometry.getPoseMeters().minus(observation.pose()));
        }

        poseEstimator.update(robotRotation, getModulePositions());
        odometry.update(robotRotation, getModulePositions());

        field.setRobotPose(getPoseEstimate());

        Logger.recordOutput("Drive/Odometry/FieldCurrentChassisSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), robotRotation));

        headingController.updateHeadingController();
        autoAlignController.updateAlignmentControllers();

        ///////////////////// SETTING DESIRED SPEEDS FROM DRIVE STATE \\\\\\\\\\\\\\\\\\
        ChassisSpeeds teleopSpeeds = teleopController.computeChassiSpeeds(
            poseEstimator.getEstimatedPosition().getRotation(), getChassisSpeeds(), false);
        switch (driveState) {
            case TELEOP:
                desiredSpeeds = teleopSpeeds;
                break;
            case TELEOP_SNIPER:
                desiredSpeeds = teleopController.computeChassiSpeeds(
                    poseEstimator.getEstimatedPosition().getRotation(), getChassisSpeeds(), true);
                break;
            case POV_SNIPER:
                desiredSpeeds = teleopController.computeSniperPOVChassisSpeeds(robotRotation);
                break;
            case HEADING_ALIGN:
                headingGoal = AllianceFlipUtil.apply(Rotation2d.fromDegrees(-90.0));
                desiredSpeeds = new ChassisSpeeds(
                    teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond,
                    headingController.getSnapOutput( poseEstimator.getEstimatedPosition().getRotation()));
                break;
            case AUTON:
                desiredSpeeds = ppDesiredSpeeds;
                break;
            case AUTO_ALIGN:
                desiredSpeeds = autoAlignController.calculate(goalPose, getPoseEstimate());
                break;    
            case STOP:
                desiredSpeeds = new ChassisSpeeds();
                break;
            case DRIFT_TEST:
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    new ChassisSpeeds(kLinearTestSpeed.get(), 0.0, 
                        Math.toRadians(kRotationDriftTestSpeed.get())), robotRotation);
                break;
            case LINEAR_TEST:
                desiredSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(kLinearTestSpeed.get(), 0.0, 0.0), robotRotation);
                break;
            case SYSID_CHARACTERIZATION:
            case WHEEL_CHARACTERIZATION:
            default:
                desiredSpeeds = null;
                break;
        }

        ///////////////////////// SETS SPEED TO MODULES \\\\\\\\\\\\\\\\\\\\\\\\
        if (desiredSpeeds != null) runSwerve(desiredSpeeds);
    }

    ///////////////////////// STATE SETTING \\\\\\\\\\\\\\\\\\\\\\\\
    public Command setDriveStateCommand(DriveState state) {
        return Commands.runOnce(() -> setDriveState(state), this);
    }

    /* Doesn't end till interuuped */
    public Command setDriveStateCommandContinued(DriveState state) {
        return new FunctionalCommand(
            () -> setDriveState(state), () -> {}, 
            (interrupted) -> {}, () -> false, this);
    }

    /* Sets the drive state used in periodic(), and handles init condtions like resetting PID controllers */
    public void setDriveState(DriveState state) {
        driveState = state;
        switch(driveState) {
            case HEADING_ALIGN:
                headingController.resetController(robotRotation, gyroInputs.yawVelocityPS);            
                break;
            case AUTO_ALIGN:
                autoAlignController.reset(getPoseEstimate(), getChassisSpeeds());
                goalPose = GoalPoseChooser.getGoalPose(CHOOSER_STRATEGY.HEXAGONAL, getPoseEstimate());
                break;
            default:
        }
    }

    ////////////// CHASSIS SPEEDS \\\\\\\\\\\\\\\\
    /* Sets the desired swerve module states to the robot */
    public void runSwerve(ChassisSpeeds speeds) {
        desiredSpeeds = SwerveUtils.discretize(speeds, kDriftRate.get());

        SwerveUtils.logPossibleDriveStates(kDoExtraLogging, desiredSpeeds, getModuleStates(), previousSetpoint, robotRotation);

        SwerveModuleState[] unOptimizedSetpointStates = new SwerveModuleState[4];
        SwerveModuleState[] setpointStates = kKinematics.toSwerveModuleStates(desiredSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, kMaxLinearSpeedMPS);

        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];

        previousSetpoint = setpointGenerator.generateSetpoint(
            previousSetpoint, desiredSpeeds, kDriveConstraints, 0.02);

        Logger.recordOutput("Drive/Odometry/generatedFieldSpeeds", ChassisSpeeds.fromRobotRelativeSpeeds(previousSetpoint.robotRelativeSpeeds(), robotRotation));

        for (int i = 0; i < 4; i++) {
            if(useGenerator) {
                SwerveUtils.logDriveFeedforward(previousSetpoint.feedforwards(), i);

                setpointStates[i] = new SwerveModuleState(
                    previousSetpoint.moduleStates()[i].speedMetersPerSecond, 
                    SwerveUtils.removeAzimuthJitter(
                        previousSetpoint.moduleStates()[i], modules[i].getCurrentState()));

                unOptimizedSetpointStates[i] = SwerveUtils.copyState(setpointStates[i]);

                setpointStates[i].optimize(modules[i].getCurrentState().angle);

                double driveAmps = calculateDriveFeedforward(
                    unOptimizedSetpointStates[i], setpointStates[i], i);
                
                setpointStates[i].cosineScale(modules[i].getCurrentState().angle);
                optimizedSetpointStates[i] = modules[i].setDesiredStateWithAmperage(setpointStates[i], driveAmps);
            } else {
                setpointStates[i] = new SwerveModuleState(
                    setpointStates[i].speedMetersPerSecond,
                    SwerveUtils.removeAzimuthJitter(
                        setpointStates[i], modules[i].getCurrentState()));

                setpointStates[i].optimize(modules[i].getCurrentState().angle);
                setpointStates[i].cosineScale(modules[i].getCurrentState().angle);
                optimizedSetpointStates[i] = modules[i].setDesiredState(setpointStates[i]);
            }
        }
        
        Logger.recordOutput("Drive/Swerve/Setpoints", unOptimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsOptimized", optimizedSetpointStates);
        Logger.recordOutput("Drive/Swerve/SetpointsChassisSpeeds", kKinematics.toChassisSpeeds(optimizedSetpointStates));
        Logger.recordOutput("Drive/Odometry/FieldSetpointChassisSpeed", ChassisSpeeds.fromRobotRelativeSpeeds(
            kKinematics.toChassisSpeeds(optimizedSetpointStates), robotRotation));
    }

    /* Calculates DriveFeedforward based off state */
    public double calculateDriveFeedforward(SwerveModuleState unoptimizedState, SwerveModuleState optimizedState, int i) {
        switch(driveState) {
            case AUTON:
                /* No need to optimize for Choreo */
                return SwerveUtils.convertChoreoNewtonsToAmps(pathPlanningFF, i);
            case AUTO_ALIGN:
                return SwerveUtils.optimizeTorque(unoptimizedState, optimizedState, pathPlanningFF.torqueCurrentsAmps()[i], i);
            default:
                return 0.0;
        }
    }

    ////////////// LOCALIZATION \\\\\\\\\\\\\\\\
    public void resetGyro() {
        robotRotation = Constants.kAlliance == Alliance.Blue ? 
            Rotation2d.fromDegrees(0.0) : Rotation2d.fromDegrees(180.0);
        gyro.resetGyro(robotRotation);
        setPose(new Pose2d(getPoseEstimate().getTranslation(), robotRotation));
    }

    public void setPose(Pose2d pose) {
        setPoses(pose, pose);
    }

    public void setPoses(Pose2d estimatorPose, Pose2d odometryPose) {
        robotRotation = estimatorPose.getRotation();
        gyro.resetGyro(robotRotation);
        // Safe to pass in odometry poses because of the syncing
        // between gyro and pose estimator in reset gyro function
        poseEstimator.resetPosition(getRobotRotation(), getModulePositions(), estimatorPose);
        odometry.resetPosition(getRobotRotation(), getModulePositions(), odometryPose);
    }

    public void resetModulesEncoders() {
        for (int i = 0; i < 4; i++) modules[i].resetAzimuthEncoder();
    }

    ///////////////////////// GETTERS \\\\\\\\\\\\\\\\\\\\\\\\
    @AutoLogOutput(key = "Drive/Swerve/MeasuredStates")
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) states[i] = modules[i].getCurrentState();
        return states;
    }

    @AutoLogOutput(key = "Drive/Swerve/ModulePositions")
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) positions[i] = modules[i].getCurrentPosition();
        return positions;
    }

    @AutoLogOutput(key = "Drive/Odometry/PoseEstimate")
    public Pose2d getPoseEstimate() {
        return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Drive/Odometry/DrivePose")
    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }

    @AutoLogOutput(key = "Drive/Odometry/RobotRotation")
    public Rotation2d getRobotRotation() {
        return robotRotation;
    }

    @AutoLogOutput(key = "Drive/Odometry/CurrentChassisSpeeds")
    public ChassisSpeeds getChassisSpeeds() {
        return kKinematics.toChassisSpeeds(getModuleStates());
    }

    @AutoLogOutput(key = "Drive/Odometry/DesiredChassisSpeeds")
    public ChassisSpeeds getDesiredChassisSpeeds() {
        return desiredSpeeds;
    }

    @AutoLogOutput(key = "Drive/Tolerance/HeadingController")
    public boolean inHeadingTolerance() {
        return GeomUtil.getSmallestChangeInRotation(robotRotation, headingGoal).getDegrees() < 1.0;
    }

    /////////// CHARACTERIZATION \\\\\\\\\\\\\\\
    /* LINEAR CHARACTERIZATION: The x-y movement of the drivetrain(basically drive motor feedforward) */
    public Command characterizeLinearMotion() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> {
                runLinearCharcterization(voltage);
        }, this));
    }

    public void runLinearCharcterization(double volts) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        for(int i = 0; i < 4; i++) modules[i].runCharacterization(volts);
    }

    public void setForwardAmperagesForAllModules(double amps) {
        for(int i = 0; i < 4; i++) {
            modules[i].setDesiredVelocity(null);
            modules[i].setDriveAmp(amps);
            modules[i].setDesiredRotation(Rotation2d.fromDegrees(0.0));
        }
    }

    /* ANGULAR CHARACTERIZATION: The angular movement of the drivetrain(basically used to get drivebase MOI)*/
    public Command characterizeAngularMotion() {
        return setDriveStateCommand(DriveState.SYSID_CHARACTERIZATION).andThen(
            SysIDCharacterization.runDriveSysIDTests( (voltage) -> runAngularCharacterization(voltage), this));
    }

    public void runAngularCharacterization(double volts) {
        setDriveState(DriveState.SYSID_CHARACTERIZATION);
        modules[0].runCharacterization( volts, Rotation2d.fromDegrees(-45.0));
        modules[1].runCharacterization(-volts, Rotation2d.fromDegrees( 45.0));
        modules[2].runCharacterization( volts, Rotation2d.fromDegrees( 45.0));
        modules[3].runCharacterization(-volts, Rotation2d.fromDegrees(-45.0));
    }

    public void acceptJoystickInputs(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, DoubleSupplier povSupplierDegrees) {
        teleopController.acceptJoystickInputs(xSupplier, ySupplier, thetaSupplier, povSupplierDegrees);
    }
}