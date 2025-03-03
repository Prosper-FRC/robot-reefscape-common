package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldConstants {
    public static final Pose2d kReefCenter = new Pose2d(4.48249, Constants.kFieldWidthMeters / 2.0, Rotation2d.fromDegrees(0.0));

    public static final double kXNetLineMeters = 7.15;

    public static final Pose2d AL = new Pose2d(3.24, 4.42, Rotation2d.k180deg);
    public static final Pose2d AR = new Pose2d(3.24, 4.11, Rotation2d.k180deg);

    public static final Pose2d BL = new Pose2d(4.17, 5.32, Rotation2d.fromDegrees(120.0));
    public static final Pose2d BR = new Pose2d(3.9, 5.15, Rotation2d.fromDegrees(120.0));

    public static final Pose2d CL = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(60.0));
    public static final Pose2d CR = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(60.0));

    public static final Pose2d DL = new Pose2d(0.0, 0.0, Rotation2d.kZero);
    public static final Pose2d DM = new Pose2d(0.0, 4.09, Rotation2d.kZero);
    public static final Pose2d DR = new Pose2d(0.0, 0.0, Rotation2d.kZero);

    public static final Pose2d FL = new Pose2d(3.54, 3.09, Rotation2d.fromDegrees(-120.0));
    public static final Pose2d FR = new Pose2d(3.79, 2.96, Rotation2d.fromDegrees(-120.0));

    public static final Pose2d EL = new Pose2d(4.81, 2.75, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d EM = new Pose2d(5.13, 2.90, Rotation2d.fromDegrees(-60.0));
    public static final Pose2d ER = new Pose2d(5.05, 2.88, Rotation2d.fromDegrees(-60.0));

    public static final Pose2d IL = new Pose2d(1.6, 7.42, Rotation2d.fromRadians(2.20).plus(Rotation2d.k180deg));
    public static final Pose2d IR = new Pose2d(1.57, 0.66, Rotation2d.fromRadians(-2.20).plus(Rotation2d.k180deg));
}
