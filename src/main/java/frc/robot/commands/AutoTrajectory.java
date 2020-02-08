/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTrajectory extends CommandBase {
  DriveSubsystem subsystem;
  RamseteCommand ramseteCommand;

  /*
   Creates a new AutoTrajectory.
   */
  public AutoTrajectory(DriveSubsystem subsystem, String trajectoryJSON){
    this.subsystem = subsystem;
    addRequirements(this.subsystem);

    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond,
                             Constants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);
    
            
            

    // An example trajectory to follow.  All units in meters.
    
    //trajectoryJSON is defined in each automode and is defined there so that the trajectoryJSON has no value here.
    Trajectory AMtrajectory = null; //
     
    try { /*tries to get the .json file from the trajectoryJSON string, then converts it to the computer system path
            then gets the trajectory and puts it together from the x,y points from the .json files*/
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);// Trajectory path converted from trajectoryJSON
       AMtrajectory= TrajectoryUtil.fromPathweaverJson(trajectoryPath); // Creates the trajectory from the trajectoryPath
    } catch (IOException ex) { /*  catches error if trajectory cannot be found, generated, etc.
      writes error message on driver station if caught*/
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    } 

      /* This below Transforms the starting position of the trajectory to match the starting position of the actual 
      roboto. Prevents robot from moving itself to the starting point of the trajectory first, and then following the
      correct Auto Mode. (Basically starts trajectory from where the robot currently is, not first XY point in file) */
      Transform2d transform = subsystem.getPose().minus(AMtrajectory.getInitialPose());
      Trajectory transformedTrajectory = AMtrajectory.transformBy(transform);
      


    /* This is a method used to calculate the trajectory from given points and follows it using robot characterization
      that is defined above*/
    ramseteCommand = new RamseteCommand(
        transformedTrajectory, /* This had been changed to be the transformed trajecotry so that it calculates trajectory 
                                from final (transformed) trajectory*/
        subsystem::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        subsystem::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        subsystem::tankDriveVolts,
        subsystem
        
    );
  }




  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Run path following command, then stop at the end.
    ramseteCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run ramsete trajectory
    ramseteCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ramseteCommand.isFinished();
  }

}
