/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.IOException;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
// import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private static final int k_ticks_per_rev = 4096;
 private static final double k_wheel_diameter = 4.0 / 12.0;
 private static final double k_max_velocity = 4;
 private static final int k_left_f = 1;
 private static final int k_right_f = 3;
 private static final int k_left_b = 0;
 private static final int k_right_b = 2;
 private static final String  k_path_name = "example";
 private SpeedController m_left_motor;
 private SpeedController m_right_motor;
 public static AHRS m_gyro;
 private EncoderFollower m_left_follower;
 private EncoderFollower  m_right_follower;
 private Notifier m_follower_notifier;

 public WPI_TalonSRX m_left_f;
 public WPI_TalonSRX m_right_f;
 public static WPI_TalonSRX m_left_b;
 public static WPI_TalonSRX m_right_b;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    Constants.init();
    m_left_f = new WPI_TalonSRX(k_left_f);
    m_right_f = new WPI_TalonSRX(k_right_f);
    m_left_b = new WPI_TalonSRX(k_left_b);
    m_right_b = new WPI_TalonSRX(k_right_b);

    m_left_motor = new SpeedControllerGroup(m_left_f, m_left_b);
    m_right_motor = new SpeedControllerGroup(m_right_f, m_right_b);
    
    m_gyro = new AHRS(SPI.Port.kMXP);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }


  private void followPath() {
    if (m_left_follower.isFinished() ||
   m_right_follower.isFinished()) {
    m_follower_notifier.stop();
    } else {
    double left_speed = m_left_follower.calculate(m_left_b.getSensorCollection().getQuadraturePosition());
    double right_speed = m_right_follower.calculate(m_right_b.getSensorCollection().getQuadraturePosition());
    double heading = m_gyro.getAngle();
    double desired_heading = Pathfinder.r2d(m_left_follower.getHeading());
    double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
    double turn = 0.8 * (-1.0/80.0) * heading_difference;
    m_left_motor.set(left_speed + turn);
    m_right_motor.set(right_speed - turn);
    }
   }
  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    try {
      // Trajectory left_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".right");
      // Trajectory right_trajectory = PathfinderFRC.getTrajectory(k_path_name + ".left");
      Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Paths.get("/home/lvuser/deploy/paths/example.wpilib.json"));
      
      // m_left_follower = new EncoderFollower(left_trajectory);
      // m_right_follower = new EncoderFollower(right_trajectory);
     
     m_left_follower.configureEncoder(m_left_b.getSensorCollection().getQuadraturePosition(), k_ticks_per_rev, k_wheel_diameter);
      // You must tune the PID values on
    //  the following line!
      m_left_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
     
     m_right_follower.configureEncoder(m_right_b.getSensorCollection().getQuadraturePosition(), k_ticks_per_rev, k_wheel_diameter);
      // You must tune the PID values on
    //  the following line!
      m_right_follower.configurePIDVA(1.0, 0.0, 0.0, 1 / k_max_velocity, 0);
      m_follower_notifier = new Notifier(this::followPath);
     
    //  m_follower_notifier.startPeriodic(left_trajectory.get(0).dt);
      } catch (IOException e) {
      e.printStackTrace();
      }

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    m_follower_notifier.stop();
    m_left_motor.set(0);
    m_right_motor.set(0);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
