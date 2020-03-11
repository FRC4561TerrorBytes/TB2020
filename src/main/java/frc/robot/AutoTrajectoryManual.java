/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Add your docs here.
 */
public class AutoTrajectoryManual {
  // Ramsete Command values
  final double MAX_VELOCITY_METERS_PER_SECOND = 3.75;
  final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.75;
  final double TRACK_WIDTH = 0.53975;
  final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(TRACK_WIDTH);
  final double VOLTS_kS = 0.240;
  final double VOLT_SECONDS_PER_METER_kV = 2.39;
  final double VOLT_SECONDS_SQUARD_PER_METER_kA = 0.140;
  final double kP = 2.6e-5;
  final double kD = 2.21e-5;
  final double kRamseteB = 0.75;
  final double kRamseteZeta = 0.0;

  DriveSubsystem subsystem;
  RamseteCommand ramseteCommand;
  
  /**
   * Create new path trajectory using JSON file containing path
   * @param subsystem DriveSubsystem to drive the robot
   * @param trajectoryJSON JSON file containing path
   */
  public AutoTrajectoryManual(DriveSubsystem subsystem, Pose2d startPoint, Pose2d endPoint, boolean isReversed) {
    this.subsystem = subsystem;
    List<Pose2d> waypoints = new ArrayList<Pose2d>() {
      {
        add(startPoint);
        add(endPoint);
      }
    };
    
    TrajectoryConfig config = new TrajectoryConfig(MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    config.setReversed(isReversed);
    
    // This transforms the starting position of the trajectory to match the starting position of the actual 
    // roboto. Prevents robot from moving to first X,Y of trajectory and then following the path.
    // Changes the first point(s) of the trajectory to the X,Y point of where the robot currently is
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(waypoints, config);
    Transform2d transform = subsystem.getPose().minus(trajectory.getInitialPose());
    Trajectory transformedTrajectory =  trajectory.transformBy(transform);

    // This is a method used to get the desired trajectory, put it into the command, have the command calculate the 
    // actual route relative to one plotted in Pathweaver, and then follow it the best it can, based on characterization given to it.
    this.ramseteCommand = new RamseteCommand(
        transformedTrajectory,  // This had been changed to be the transformed trajecotry so that it calculates trajectory 
                                // from final (transformed) trajectory
        subsystem::getPose,
        new RamseteController(this.kRamseteB, this.kRamseteZeta),
        new SimpleMotorFeedforward(this.VOLTS_kS,
                                  this.VOLT_SECONDS_PER_METER_kV,
                                  this.VOLT_SECONDS_SQUARD_PER_METER_kA),
        this.DRIVE_KINEMATICS,
        subsystem::getWheelSpeeds,
        new PIDController(this.kP, 0, this.kD),
        new PIDController(this.kP, 0, this.kD),
        // RamseteCommand passes volts to the callback
        subsystem::tankDriveVolts,
        subsystem 
    );
  }

  /**
   * Get Ramsete command to run
   * @return Ramsete command that will stop when complete
   */
  public Command getCommand() {
    return this.ramseteCommand.andThen(() -> this.subsystem.stop());
  }
}
