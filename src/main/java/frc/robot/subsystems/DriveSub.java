/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSub extends SubsystemBase {
  
  DifferentialDriveOdometry m_odometry;
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public DriveSub() {
    resetEncoders();
    zeroHeading();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  public void teleDrive(){
    Constants.DiffDrive.arcadeDrive(RobotContainer.driver.getRawAxis(1), -RobotContainer.driver.getRawAxis(4));
  }

  public void drive(double xSpeed, double rotation){
    Constants.DiffDrive.arcadeDrive(xSpeed, rotation);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      (Constants.backLeft.getSensorCollection().getQuadratureVelocity()/Constants.ticksPerRevolution)*Constants.wheelCircumferenceMeters, 
      (-Constants.backRight.getSensorCollection().getQuadratureVelocity()/Constants.ticksPerRevolution)*Constants.wheelCircumferenceMeters);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    Constants.left.setVoltage(-leftVolts);
    Constants.right.setVoltage(rightVolts);
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(pose, getAngle());
  }

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  //robot's heading in degrees from -180 to 180
  public double getHeading(){
    return Math.IEEEremainder(m_gyro.getAngle(), 360) ;
  }

  public void zeroHeading(){
    m_gyro.reset();
  }

  public void resetEncoders(){
    Constants.backLeft.getSensorCollection().setQuadraturePosition(0, 10);
    Constants.backRight.getSensorCollection().setQuadraturePosition(0, 10);
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), 
    (Constants.backLeft.getSensorCollection().getQuadraturePosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters, 
    (-Constants.backRight.getSensorCollection().getQuadraturePosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters);
  }
}