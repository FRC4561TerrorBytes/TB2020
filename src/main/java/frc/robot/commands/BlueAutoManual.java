/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import frc.robot.subsystems.DriveSubsystem;
//import edu.wpi.first.wpilibj.geometry.Pose2d;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

  
public class BlueAutoManual extends SequentialCommandGroup {
  //DriveSubsystem subsystem;
  //Pose2d startPosition;
  //Pose2d endPosition;
  
  /**
   * Creates a new BlueAutoManual. THIS IS A FAILSAFE AUTO MODE TEMPLATE ONLY!! USE ONLY IF WE HAVE TO PUT XY POINTS FROM PATHWEAVER IN MANUALLY
   */
  public BlueAutoManual() {
    /*The X,Y, and the rotation angle in radians
    Pose2d start_position = new Pose2d(1.0, 1.0, new Rotation2d(0.0)); //TODO:Change these values
    Pose2d end_position = new Pose2d(2.0, 1.0, new Rotation2d(0.0)); //TODO:Change these values

    //TODO: Change these values
    
    /*List<Translation2d> waypoints = new ArrayList<Translation2d>(); //Creates an array of 2dTranslations with the interior waypoints included
    waypoints.add(new Translation2d(1.0, 1.0)); //Creating the waypoints and setting X,Y, and Translation2d values as their parameters
    waypoints.add(new Translation2d(1.0, 1.0));
    waypoints.add(new Translation2d(1.0, 1.0));*/

    //trajectory = new AutoTrajectory(subsystem, start_position, end_position /*, waypoints if needed*/); 
    /*Auto generates a new trajectory
    from the subsystem, the start and end position coordinates and rotation, and from the interior waypoints of the path*/
    
  }
}
