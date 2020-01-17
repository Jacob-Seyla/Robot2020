package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class DriveSub extends SubsystemBase {
  
  DifferentialDriveOdometry m_odometry;
  public AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  public DriveSub() {
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    resetEncoders();
    zeroHeading();
    resetOdometry(new Pose2d());
  }

  public double getAverageEncoderDistance(){
    return ((-RobotContainer.l1.getEncoder().getPosition()/ Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters
    + (-RobotContainer.l2.getEncoder().getPosition()/ Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters
    + (RobotContainer.r1.getEncoder().getPosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters
    + (RobotContainer.r2.getEncoder().getPosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters) / 4.0;
  }

  public double getTurnRate(){
    return gyroReversed*m_gyro.getRate();
  }

  public void teleDrive(){

    if(RobotContainer.dback.get()){
      RobotContainer.sol1.set(Value.kForward);
    }else if(RobotContainer.dstart.get()){
      RobotContainer.sol1.set(Value.kReverse);
    }

    if(RobotContainer.dx.get()){
      tankDriveVolts(2, 2);
    }else if(RobotContainer.dbumperLeft.get() || RobotContainer.da.get()){
      RobotContainer.DiffDrive.arcadeDrive(RobotContainer.driver.getRawAxis(1) * 0.5, (-RobotContainer.driver.getRawAxis(4) * 0.5));
    }else {
      RobotContainer.DiffDrive.arcadeDrive(RobotContainer.driver.getRawAxis(1), -RobotContainer.driver.getRawAxis(4));
    }
  }

  public void drive(double xSpeed, double rotation){
    RobotContainer.DiffDrive.arcadeDrive(xSpeed, rotation);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      ((-RobotContainer.l1.getEncoder().getVelocity()/60 / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters +
      (-RobotContainer.l2.getEncoder().getVelocity()/60 / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters)/2, 
      ((RobotContainer.r1.getEncoder().getVelocity()/60 / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters +
      (RobotContainer.r2.getEncoder().getVelocity()/60 / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters)/2);
  }

  double maxVolt = 100; //5
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    RobotContainer.left.setVoltage(MathUtil.clamp(-rightVolts, -maxVolt, maxVolt));
    RobotContainer.right.setVoltage(MathUtil.clamp(leftVolts, -maxVolt, maxVolt));
    // System.out.println("L Volt:     " + leftVolts + "     R Volt      " + rightVolts);
  }

  public Pose2d getPose(){
    return m_odometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose){
    m_odometry.resetPosition(pose, getAngle());
  }

  private double gyroReversed = 1;

  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(gyroReversed*m_gyro.getAngle());
  }

  //robot's heading in degrees from -180 to 180
  public double getHeading(){
    return Math.IEEEremainder(gyroReversed*m_gyro.getAngle(), 360) ;
  }

  public void zeroHeading(){
    m_gyro.reset();
  }

  public void resetEncoders(){
    RobotContainer.l1.getEncoder().setPosition(0);
    RobotContainer.l2.getEncoder().setPosition(0);
    RobotContainer.r1.getEncoder().setPosition(0);
    RobotContainer.r2.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), 
    ((-RobotContainer.l1.getEncoder().getPosition()/ Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters
    + (-RobotContainer.l2.getEncoder().getPosition()/ Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters)/2, 
    ((RobotContainer.r1.getEncoder().getPosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters
    + (RobotContainer.r2.getEncoder().getPosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters)/2);
  }
}