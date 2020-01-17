/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    // public static CANSparkMax r1 = new CANSparkMax(10, MotorType.kBrushless);
    // public static CANSparkMax r2 = new CANSparkMax(11, MotorType.kBrushless);
    // public static CANSparkMax l1 = new CANSparkMax(12, MotorType.kBrushless);
    // public static CANSparkMax l2 = new CANSparkMax(13, MotorType.kBrushless);

    // public static SpeedControllerGroup left = new SpeedControllerGroup(l1, l2);
    // public static SpeedControllerGroup right = new SpeedControllerGroup(r1, r2);
    
    // public static DifferentialDrive DiffDrive = new DifferentialDrive(left, right);

    // Compressor compressor = new Compressor(1);
    // DoubleSolenoid sol1 = new DoubleSolenoid(1, 0, 1);

    public static final double ticksPerRevolution = 4096.0;
    public static final double wheelCircumferenceMeters = 6*Math.PI*0.0254;

    public static final class DriveConstants {
        public static final int kl1Port = 10;
        public static final int kl2Port = 11;
        public static final int kr1Port = 12;
        public static final int kr2Port = 13;

        // public static final double kTrackWidth = 0.61;
        public static final double kTrackWidthLow = 1.187;
        public static final DifferentialDriveKinematics kDriveKinematicsLow = new DifferentialDriveKinematics(kTrackWidthLow);

        public static final double ksVoltsLow = 1.08;
        public static final double kvVoltSecondsPerMeterLow = 0.0638;
        public static final double kaVoltSecondsSquaredPerMeterLow = 0.0169;

        public static final double kPDriveVelLow = 2.1;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecondLow = 3;
        public static final double kMaxAccelerationMetersPerSecondSquaredLow = 3;
        public static final double kMaxAngularSpeedRadiansPerSecondLow = Math.PI; 
        public static final double kMaxAngularSpeedRadiansPerSecondSquaredLow = Math.PI; 

        public static final TrapezoidProfile.Constraints kThetaControllerConstraintsLow = new 
        TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecondLow, kMaxAngularSpeedRadiansPerSecondSquaredLow);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
}
