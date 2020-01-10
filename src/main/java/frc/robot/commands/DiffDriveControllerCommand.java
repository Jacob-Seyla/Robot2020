package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

/**
 * A command that uses two PID controllers ({@link PIDController}) and a
 * ProfiledPIDController ({@link ProfiledPIDController}) to follow a trajectory
 * {@link Trajectory} with a DifferentialDrive drive.
 *
 * <p>This command outputs the raw desired DifferentialDrive Module States ({@link DifferentialDriveModuleState})
 * in an array. The desired wheel and module rotation velocities should be taken
 * from those and used in velocity PIDs.
 *
 * <p>The robot angle controller does not follow the angle given by
 * the trajectory but rather goes to the angle given in the final state of the trajectory.
 */

@SuppressWarnings("MemberName")
public class DiffDriveControllerCommand extends CommandBase {
  private final Timer m_timer = new Timer();
  private Pose2d m_finalPose;

  private final Trajectory m_trajectory;
  private final Supplier<Pose2d> m_pose;
  private final DifferentialDriveKinematics m_kinematics;
  private final PIDController m_yController;
  private final ProfiledPIDController m_thetaController;

  /**
   * Constructs a new DifferentialDriveControllerCommand that when executed will follow the
   * provided trajectory. This command will not return output voltages but rather
   * raw module states from the position controllers which need to be put into a
   * velocity PID.
   *
   * <p>
   * Note: The controllers will *not* set the outputVolts to zero upon completion
   * of the path- this is left to the user, since it is not appropriate for paths
   * with nonstationary endstates.
   *
   * <p>
   * Note 2: The rotation controller will calculate the rotation based on the
   * final pose in the trajectory, not the poses at each time step.
   *
   * @param trajectory         The trajectory to follow.
   * @param pose               A function that supplies the robot pose - use one
   *                           of the odometry classes to provide this.
   * @param kinematics         The kinematics for the robot drivetrain.
   * @param yController        The Trajectory Tracker PID controller for the
   *                           robot's y position.
   * @param thetaController    The Trajectory Tracker PID controller for angle for
   *                           the robot.
   * @param requirements       The subsystems to require.
   */

  @SuppressWarnings("ParameterName")
  public DiffDriveControllerCommand(Trajectory trajectory,
                               Supplier<Pose2d> pose,
                               DifferentialDriveKinematics kinematics,
                               PIDController yController,
                               ProfiledPIDController thetaController,
                               Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "DifferentialDriveControllerCommand");
    m_pose = requireNonNullParam(pose, "pose", "DifferentialDriveControllerCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "DifferentialDriveControllerCommand");

    m_yController = requireNonNullParam(yController,
      "xController", "DifferentialDriveControllerCommand");
    m_thetaController = requireNonNullParam(thetaController,
      "thetaController", "DifferentialDriveControllerCommand");

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    // Sample final pose to get robot rotation
    m_finalPose = m_trajectory.sample(m_trajectory.getTotalTimeSeconds()).poseMeters;

    m_timer.reset();
    m_timer.start();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void execute() {
    double curTime = m_timer.get();

    var desiredState = m_trajectory.sample(curTime);
    var desiredPose = desiredState.poseMeters;

    var poseError = desiredPose.relativeTo(m_pose.get());

    double targetYVel = m_yController.calculate(
        m_pose.get().getTranslation().getY(),
        desiredPose.getTranslation().getY());

    // The robot will go to the desired rotation of the final pose in the trajectory,
    // not following the poses at individual states.
    double targetAngularVel = m_thetaController.calculate(
        m_pose.get().getRotation().getRadians(),
        m_finalPose.getRotation().getRadians());

    double vRef = desiredState.velocityMetersPerSecond;

    targetYVel += vRef * poseError.getRotation().getSin();

    // var targetChassisSpeeds = new ChassisSpeeds(0, targetYVel, targetAngularVel);
    // var targetModuleStates = m_kinematics.toDifferentialDriveModuleStates(targetChassisSpeeds);
    // m_outputModuleStates.accept(targetModuleStates);

    Robot.driveSub.drive(targetYVel, -targetAngularVel);

  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasPeriodPassed(m_trajectory.getTotalTimeSeconds());
  }
}
