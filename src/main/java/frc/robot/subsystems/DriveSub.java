package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
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
    return ((-Constants.backLeft.getSensorCollection().getQuadraturePosition()/ Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters
    + (Constants.backRight.getSensorCollection().getQuadraturePosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters) / 2.0;
  }

  public double getTurnRate(){
    return m_gyro.getRate();
  }

  public void teleDrive(){
    // if(RobotContainer.dx.get()){
    //   tankDriveVolts(2, 2);
    // }
    if(RobotContainer.db.get()){
      Constants.DiffDrive.arcadeDrive(RobotContainer.driver.getRawAxis(1) * 0.5, (-RobotContainer.driver.getRawAxis(4) * 0.5));
    }else {
      Constants.DiffDrive.arcadeDrive(RobotContainer.driver.getRawAxis(1), -RobotContainer.driver.getRawAxis(4));
    }
  }

  public void drive(double xSpeed, double rotation){
    Constants.DiffDrive.arcadeDrive(xSpeed, rotation);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      (-Constants.backLeft.getSensorCollection().getQuadratureVelocity()*10 / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters, 
      (Constants.backRight.getSensorCollection().getQuadratureVelocity()*10 / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters);

  }

  double maxVolt = 100; //5
  public void tankDriveVolts(double leftVolts, double rightVolts) {
        Constants.left.setVoltage(MathUtil.clamp(-leftVolts, -maxVolt, maxVolt));
    Constants.right.setVoltage(MathUtil.clamp(rightVolts, -maxVolt, maxVolt));
    // System.out.println("L Volt:     " + leftVolts + "     R Volt      " + rightVolts);
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
    (-Constants.backLeft.getSensorCollection().getQuadraturePosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters, 
    (Constants.backRight.getSensorCollection().getQuadraturePosition() / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters);

    // / Constants.ticksPerRevolution) * Constants.wheelCircumferenceMeters
  }
}
