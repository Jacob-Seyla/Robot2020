/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.BasicTurn;
import frc.robot.commands.Forward;
import frc.robot.commands.Forward2;
import frc.robot.subsystems.DriveSub;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  SendableChooser<Command> m_chooser;
  
  private RobotContainer m_robotContainer;

  public static DriveSub driveSub;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    driveSub = new DriveSub();
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    RobotContainer.r1.setIdleMode(IdleMode.kCoast);
    RobotContainer.r2.setIdleMode(IdleMode.kCoast);
    RobotContainer.l1.setIdleMode(IdleMode.kCoast);
    RobotContainer.l2.setIdleMode(IdleMode.kCoast);
    // RobotContainer.r1.setIdleMode(IdleMode.kBrake);
    // RobotContainer.r2.setIdleMode(IdleMode.kBrake);
    // RobotContainer.l1.setIdleMode(IdleMode.kBrake);
    // RobotContainer.l2.setIdleMode(IdleMode.kBrake);
    double ramprate = .25;
    RobotContainer.l1.setSmartCurrentLimit(40);
    RobotContainer.l1.setClosedLoopRampRate(ramprate);
    RobotContainer.l1.setOpenLoopRampRate(ramprate);
    RobotContainer.l2.setSmartCurrentLimit(40);
    RobotContainer.l2.setClosedLoopRampRate(ramprate);
    RobotContainer.l2.setOpenLoopRampRate(ramprate);
    RobotContainer.r1.setSmartCurrentLimit(40);
    RobotContainer.r1.setClosedLoopRampRate(ramprate);
    RobotContainer.r1.setOpenLoopRampRate(ramprate);
    RobotContainer.r2.setSmartCurrentLimit(40);
    RobotContainer.r2.setClosedLoopRampRate(ramprate);
    RobotContainer.r2.setOpenLoopRampRate(ramprate);

    
    try {
      m_robotContainer = new RobotContainer();
    } catch (IOException e1) {
      // TODO Auto-generated catch block
      e1.printStackTrace();
    }

    RobotContainer.DiffDrive.setSafetyEnabled(false);

    m_chooser = new SendableChooser<>();
    m_chooser.addOption("forward", new Forward());
    m_chooser.addOption("forward2", new Forward2());
    m_chooser.addOption("Basic Turning", new BasicTurn());
    // try {
      // m_chooser.addOption("pathweaver1", new PathweaverPath());
      // m_chooser.addOption("chain", new Chain());
      // m_chooser.addOption("smiles", new Smiles());
    // } catch (IOException e1) {
      // e1.printStackTrace();
    // }
    SmartDashboard.putData("Auto choochooer", m_chooser);
    // try {
    //   m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // } catch (IOException e) {
    //   e.printStackTrace();
    // }
    RobotContainer.sol1.set(Value.kForward);

    driveSub.resetEncoders();
    driveSub.zeroHeading();
    driveSub.resetOdometry(new Pose2d());
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
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

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */

  @Override
  public void autonomousInit() {
    driveSub.resetEncoders();
    driveSub.zeroHeading();
    driveSub.resetOdometry(new Pose2d());
    m_autonomousCommand = m_chooser.getSelected();
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
    SmartDashboarding();
}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    RobotContainer.r1.setIdleMode(IdleMode.kBrake);
    RobotContainer.r2.setIdleMode(IdleMode.kBrake);
    RobotContainer.l1.setIdleMode(IdleMode.kBrake);
    RobotContainer.l2.setIdleMode(IdleMode.kBrake);

    SmartDashboarding();
    if(RobotContainer.dbumperRight.get()){
      driveSub.resetEncoders();
      driveSub.resetOdometry(new Pose2d());
      driveSub.zeroHeading();
    }
    // SmartDashboard.putNumber("Drive Fowards???", driveSub.);
    // SmartDashboard.putNumber("axis 4", RobotContainer.driver.getRawAxis(4));
  }

  public static void SmartDashboarding(){
    // System.out.println(" gyro actual upfate rate:    " + driveSub.m_gyro.getActualUpdateRate());
    // System.out.println(" gyro get last sensor times tsamp:    " + driveSub.m_gyro.getLastSensorTimestamp());


    SmartDashboard.putString("Wheel Speeds", driveSub.getWheelSpeeds().toString());
    SmartDashboard.putNumber("l1",RobotContainer.l1.getEncoder().getPosition());
    SmartDashboard.putNumber("l2",RobotContainer.l2.getEncoder().getPosition());
    SmartDashboard.putNumber("r1",-RobotContainer.r1.getEncoder().getPosition());
    SmartDashboard.putNumber("r2",-RobotContainer.r2.getEncoder().getPosition());
    SmartDashboard.putNumber("Gyro Heading", driveSub.getHeading());
    SmartDashboard.putNumber("Raw gyro", driveSub.m_gyro.getAngle());
    SmartDashboard.putString("Pose", driveSub.getPose().toString());
    SmartDashboard.putString("Pose Rot", driveSub.getPose().getRotation().toString());
    SmartDashboard.putString("Pose Dist", driveSub.getPose().getTranslation().toString());
    if(RobotContainer.sol1.get() == Value.kForward){
      SmartDashboard.putNumber("R Dist", (-RobotContainer.r1.getEncoder().getPosition() / Constants.ticksPerRevolutionLow) * Constants.wheelCircumferenceMeters);
      SmartDashboard.putNumber("L Dist", (RobotContainer.l1.getEncoder().getPosition() / Constants.ticksPerRevolutionLow) * Constants.wheelCircumferenceMeters);
      SmartDashboard.putNumber("R Vel", (-RobotContainer.r1.getEncoder().getVelocity()/60 / Constants.ticksPerRevolutionLow) * Constants.wheelCircumferenceMeters);
      SmartDashboard.putNumber("L Vel", (RobotContainer.l1.getEncoder().getVelocity()/60 / Constants.ticksPerRevolutionLow) * Constants.wheelCircumferenceMeters);
    }else{
      SmartDashboard.putNumber("R Dist", (-RobotContainer.r1.getEncoder().getPosition() / Constants.ticksPerRevolutionHigh) * Constants.wheelCircumferenceMeters);
      SmartDashboard.putNumber("L Dist", (RobotContainer.l1.getEncoder().getPosition() / Constants.ticksPerRevolutionHigh) * Constants.wheelCircumferenceMeters);
      SmartDashboard.putNumber("R Vel", (-RobotContainer.r1.getEncoder().getVelocity()/60 / Constants.ticksPerRevolutionHigh) * Constants.wheelCircumferenceMeters);
      SmartDashboard.putNumber("L Vel", (RobotContainer.l1.getEncoder().getVelocity()/60 / Constants.ticksPerRevolutionHigh) * Constants.wheelCircumferenceMeters);  
    }
    SmartDashboard.putNumber("axis 1", RobotContainer.driver.getRawAxis(1));
    SmartDashboard.putNumber("axis 4", RobotContainer.driver.getRawAxis(4));
    SmartDashboard.putString("Sol", RobotContainer.sol1.get().toString());
    // SmartDashboard.putNumber("R volts", RobotContainer.backRight.getMotorOutputVoltage());
    // SmartDashboard.putNumber("L volts", -RobotContainer.backLeft.getMotorOutputVoltage());
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
