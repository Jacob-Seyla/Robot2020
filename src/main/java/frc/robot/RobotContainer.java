/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
// import java.nio.file.Paths;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.controller.PIDController;
// import edu.wpi.first.wpilibj.controller.RamseteController;
// import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj.trajectory.Trajectory;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
// import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
// import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Forward;
import frc.robot.commands.PathweaverPath;
// import frc.robot.commands.AutoCommand;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // private final Lift lift = new Lift();
  // private final GyroSwerveDrive drive = new GyroSwerveDrive();
  // private final Intake intake = new Intake();

  public static CANSparkMax r1 = new CANSparkMax(10, MotorType.kBrushless);
  public static CANSparkMax r2 = new CANSparkMax(11, MotorType.kBrushless);
  public static CANSparkMax l1 = new CANSparkMax(12, MotorType.kBrushless);
  public static CANSparkMax l2 = new CANSparkMax(13, MotorType.kBrushless);

  public static SpeedControllerGroup left = new SpeedControllerGroup(l1, l2);
  public static SpeedControllerGroup right = new SpeedControllerGroup(r1, r2);
  
  public static DifferentialDrive DiffDrive = new DifferentialDrive(left, right);

  public static Compressor compressor = new Compressor(1);
  public static DoubleSolenoid sol1 = new DoubleSolenoid(1, 0, 1);

  // private BasicArmCommand armCommand = new BasicArmCommand();


  PathweaverPath pathWeaver = new PathweaverPath();
  Forward forwardCommand = new Forward();

  public static Joystick driver = new Joystick(0);

  public static JoystickButton da = new JoystickButton(driver, 1);
  public static JoystickButton db = new JoystickButton(driver, 2);
  public static JoystickButton dx = new JoystickButton(driver, 3);
  public static JoystickButton dy = new JoystickButton(driver, 4);
  public static JoystickButton dbumperLeft = new JoystickButton(driver, 5);
  public static JoystickButton dbumperRight = new JoystickButton(driver, 6);
  public static JoystickButton dback = new JoystickButton(driver, 7);
  public static JoystickButton dstart = new JoystickButton(driver, 8);
  public static JoystickButton dterribleLeft = new JoystickButton(driver, 9);
  public static JoystickButton dterribleRight = new JoystickButton(driver, 10);

  public RobotContainer() throws IOException { 
    // Configure the button bindings
    configureButtonBindings();

    Robot.driveSub.setDefaultCommand(new RunCommand(() -> Robot.driveSub.teleDrive(), Robot.driveSub));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws IOException
   */
  public Command getAutonomousCommand() throws IOException {

    // DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
    //     new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
    //         DriveConstants.kaVoltSecondsSquaredPerMeter),
    //     DriveConstants.kDriveKinematics, 10);

    // TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(DriveConstants.kDriveKinematics)
    //         // Apply the voltage co nstraint
            // .addConstraint(autoVoltageConstraint);

    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0,0,new Rotation2d(0)),
    // List.of(
    // new Translation2d(1,0),
    // new Translation2d(1,-1),
    // new Translation2d(-1,-1),
    // new Translation2d(-1,1)
    // ),
    // new Pose2d(1,1,new Rotation2d(Math.toRadians(0))), config);
    // new Pose2d(1,1,new Rotation2d(Math.toRadians(90))), config);
    // Trajectory exampleTrajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/output/example.wpilib.json"));

      // RamseteCommand ramseteCommand = new RamseteCommand(
      //   exampleTrajectory,
      //   Robot.driveSub::getPose,
      //   new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
      //   new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
      //   DriveConstants.kDriveKinematics,
      //   Robot.driveSub::getWheelSpeeds,
      //   new PIDController(DriveConstants.kPDriveVel, 0, 0),
      //   new PIDController(DriveConstants.kPDriveVel, 0, 0),
      //   // RamseteCommand passes volts to the callback
      //   Robot.driveSub::tankDriveVolts,
      //   Robot.driveSub
      // );


    // An ExampleCommand will run in autonomous
    // return forwardCommand;
    return pathWeaver;
    // return ramseteCommand.andThen(() -> Robot.driveSub.drive(0, 0));
  }
}