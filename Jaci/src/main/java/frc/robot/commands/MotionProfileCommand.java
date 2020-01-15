package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class MotionProfileCommand extends CommandBase {
	Timer t;
	Trajectory trajectory;
	EncoderFollower follower;

    public MotionProfileCommand(Trajectory trajectory) {
    	t = new Timer();
    	// Use requires() here to declare subsystem dependencies
        requires(Robot.DriveSubsystem);
    	
    	// Wheelbase Width = 2ft
        // Do something with the new Trajectories...
        this.trajectory = trajectory;
        follower = new EncoderFollower(trajectory);
    }

    // Called just before this Command runs the first time
    public void initialize() {
    	Robot.m_gyro.reset();
    	t.reset();
    	t.start();
    	follower.reset();
    	Robot.m_right_b.getSensorCollection().setQuadraturePosition(0, 10);
    	Robot.m_left_b.getSensorCollection().setQuadraturePosition(0, 10);
    	// max velocity 8.65 ft/s ? and kv = 1/max
    	
    	follower.configurePIDVA(0.07, 0, 0.001, 1.0/10, 0.001); // real bot pidva
//    	follower.configurePIDVA(1, 0, 0.01, 1.0/8.5, 0);
//    	follower.configurePIDVA(0.01, 0, 0.015, 1.0/8.5, 0);
    	follower.configureEncoder(0, 3413, 1.0/3.0);
    	
    }

    // Called repeatedly when this Command is scheduled to run
    public void execute() {
    	double power = follower.calculate(Constants.Right2.getSensorCollection().getQuadraturePosition());
    	Robot.DriveSubsystem.DiffDrive(-power, Constants.Gyro1.getAngle()*0.1);
    	SmartDashboard.putNumber("timer motion profile", t.get());
    	t.reset();
    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
    	if(follower.isFinished()){
    		return true;
    	}
        return false;
    }

    // Called once after isFinished returns true
    public void end() {
    	Robot.DriveSubsystem.DiffDrive(0,0);
    	Constants.Right2.getSensorCollection().setQuadraturePosition(0, 10);
    	Constants.Left2.getSensorCollection().setQuadraturePosition(0, 10);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    public void interrupted() {
//    	Constants.Right1.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
//    	Constants.Right2.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
//    	Constants.Left1.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
//    	Constants.Left2.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
    	this.end();
//    	this.cancel();
    }
}
