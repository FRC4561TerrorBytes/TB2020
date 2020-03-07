/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
//import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoTrajectory {
  RamseteCommand ramseteCommand;
  Trajectory AMtrajectory;

  /*
   Creates a new AutoTrajectory.
   */
  public AutoTrajectory(DriveSubsystem subsystem, String trajectoryJSON) {
   
   // Each Auto Mode is created in RobotContainer and has its own String for trajectoryJSON which it gets
   //from AutoModeConstants
    try { /*tries to get the PathWeaver .json file from the directory in the code that has been
      deployed onto the RoboRIO.*/
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);// Trajectory path converted from trajectoryJSON
      AMtrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath); // Creates the trajectory from the trajectoryPath
    } catch (Exception ex) { /*  catches error if trajectory cannot be found, generated, etc.
      writes error message on driver station if caught*/
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    } 
  
    /* This Transforms the starting position of the trajectory to match the starting position of the actual 
    roboto. Prevents robot from moving to first X,Y of trajectory and then following the path.
    Changes the first point(s) of the trajectory to the X,Y point of where the robot currently is*/
    Transform2d transform = subsystem.getPose().minus(AMtrajectory.getInitialPose());
    Trajectory transformedTrajectory =  AMtrajectory.transformBy(transform);

    /* This is a method used to get the desired trajectory, put it into the command, have the command calculate the 
    actual route relative to one plotted in Pathweaver, and then follow it the best it can, based on characterization given to it.*/
    this.ramseteCommand = new RamseteCommand(
        transformedTrajectory, /* This had been changed to be the transformed trajecotry so that it calculates trajectory 
                                from final (transformed) trajectory*/
        subsystem::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        subsystem::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
        new PIDController(Constants.kPDriveVel, 0, Constants.kDDriveVel),
        // RamseteCommand passes volts to the callback
        subsystem::tankDriveVolts,
        subsystem 
    );
  }

  public Command getCommand() {
    return this.ramseteCommand;
  }

}
