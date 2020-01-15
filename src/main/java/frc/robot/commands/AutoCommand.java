// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot.commands;

// import java.util.List;

// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;

// public class AutoCommand extends CommandBase {

//   DifferentialDriveVoltageConstraint autoVoltageConstraint =
//   new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, 
//   DriveConstants.kaVoltSecondsSquaredPerMeter), DriveConstants.kDriveKinematics, 10);

// // TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
// // .setKinematics(DriveConstants.kDriveKinematics);

// TrajectoryConfig config =
//   new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//       // Add kinematics to ensure max speed is actually obeyed
//       .setKinematics(DriveConstants.kDriveKinematics)
//       // Apply the voltage constraint
//       .addConstraint(autoVoltageConstraint);

// Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
// new Pose2d(0,0,new Rotation2d(0)), 
// List.of(
//   new Translation2d(0.5, 0)
//   // new Translation2d(2 / 2, -1 / 2)
// ), 
// new Pose2d(1,0,new Rotation2d(0)), config);

// // final DiffDriveControllerCommand ddcc = new DiffDriveControllerCommand(exampleTrajectory, driveSub :: getPose, DriveConstants.kDriveKinematics, 
// // new PIDController(AutoConstants.kPYController, 0, 0),
// // new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints),
// //   driveSub);

// RamseteCommand ramseteCommand = new RamseteCommand(
//   exampleTrajectory,
//   Robot.driveSub::getPose,
//   new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
//   new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
//   DriveConstants.kDriveKinematics,
//   Robot.driveSub::getWheelSpeeds,
//   new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
//   new PIDController(DriveConstants.kPDriveVel, 0, DriveConstants.kDDriveVel),
//   // RamseteCommand passes volts to the callback
//   Robot.driveSub::tankDriveVolts,
//   Robot.driveSub
// );

//   public AutoCommand() {
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     ramseteCommand.schedule();


//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     Constants.left.set(0);
//     Constants.right.set(0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
