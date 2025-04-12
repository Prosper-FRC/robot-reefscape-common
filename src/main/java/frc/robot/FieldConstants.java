package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kXNetLineMeters = 7.15;

    public static final Pose2d AL = new Pose2d(3.22, 4.43, Rotation2d.k180deg);
    public static final Pose2d AM = new Pose2d(3.22, 4.25, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.23, 4.09, Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(4.19, 5.31, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BM = new Pose2d(4.06, 5.24, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.93, 5.16, Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(5.45, 4.92, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CM = new Pose2d(5.335, 4.995, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(5.19, 5.07, Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(5.75, 3.64, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(5.76, 3.785, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.76, 3.95, Rotation2d.kZero);

    public static final Pose2d EL = new Pose2d(4.78, 2.7, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(4.915, 2.805, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.04, 2.88, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d FL = new Pose2d(3.47, 3.10, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FM = new Pose2d(3.645, 3.045, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.81, 2.94, Rotation2d.fromDegrees(-120.0));
    
    public static final Pose2d B_IL = new Pose2d(1.45, 7.29, Rotation2d.fromRadians(2.20));
    public static final Pose2d B_IR = new Pose2d(0.88, 1.64, Rotation2d.fromRadians(-2.20));

    public static final Pose2d R_IL = new Pose2d(16.01, 0.75, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg));
    public static final Pose2d R_IR = new Pose2d(16.40, 7.04, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg));
}