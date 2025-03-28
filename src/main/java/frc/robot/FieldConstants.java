package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));
    public static final double kXNetLineMeters = 7.15;

    public static final Pose2d AL = new Pose2d(3.22, 4.40, Rotation2d.k180deg);
    public static final Pose2d AM = new Pose2d(3.22, 4.25, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.22, 4.10, Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(4.20, 5.32, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BM = new Pose2d(4.06, 5.24, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.92, 5.16, Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(5.47, 4.91, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CM = new Pose2d(5.335, 4.995, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(5.20, 5.08, Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(5.76, 3.61, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(5.76, 3.785, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(5.76, 3.96, Rotation2d.kZero);

    public static final Pose2d EL = new Pose2d(4.76, 2.72, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(4.915, 2.805, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.07, 2.89, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d FL = new Pose2d(3.50, 3.13, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FM = new Pose2d(3.645, 3.045, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.79, 2.96, Rotation2d.fromDegrees(-120.0));
    
    public static final Pose2d B_IL = new Pose2d(1.58, 7.38, Rotation2d.fromRadians(2.20));
    public static final Pose2d B_IR = new Pose2d(1.10, 1.03, Rotation2d.fromRadians(-2.20));

    public static final Pose2d R_IL = new Pose2d(16.03, 0.72, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg));
    public static final Pose2d R_IR = new Pose2d(16.42, 7.01, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg));
}