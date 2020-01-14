/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static WPI_TalonSRX frontRight = new WPI_TalonSRX(3);
    public static WPI_TalonSRX backRight = new WPI_TalonSRX(2);
    public static WPI_TalonSRX frontLeft = new WPI_TalonSRX(1);
    public static WPI_TalonSRX backLeft = new WPI_TalonSRX(0);

    public static SpeedControllerGroup left = new SpeedControllerGroup(frontLeft, backLeft);
    public static SpeedControllerGroup right = new SpeedControllerGroup(frontRight, backRight);
    
    public static DifferentialDrive DiffDrive = new DifferentialDrive(left, right);

    public static final double ticksPerRevolution = 4096.0;
    public static final double wheelCircumferenceMeters = 3.9*Math.PI*0.0254; //0.0254 * 4 * Math.PI;

    public static final class DriveConstants {
        public static final int kFrontLeftDriveMotorPort = 1;
        public static final int kRearLeftDriveMotorPort = 0;
        public static final int kFrontRightDriveMotorPort = 3;
        public static final int kRearRightDriveMotorPort = 2;

        public static final double kTrackWidth = .61; //4.91;//0.61;

        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidth);

        public static final double kMaxSpeedMetersPerSecond = 1;

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 1.08;
        public static final double kvVoltSecondsPerMeter = 0.0638;
        public static final double kaVoltSecondsSquaredPerMeter = 0.0169;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 1.72; //.8
        public static final double kDDriveVel = 0.83;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI / 3; //\\
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI / 3; //\\

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new //\\
        TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
