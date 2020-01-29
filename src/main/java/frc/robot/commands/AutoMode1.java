/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands;

import java.util.List; //May not be used, but keep if are
import java.util.ArrayList;//Same with here

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d; //These may not be used depending on use for interior waypoints
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.AutoTrajectory;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html


/*This automode is to just test basic position movement and only the drivetrain motors. Right now, it is just 
* a simple change in rotation. Later, it will be changed into slight change in forward movement, then side to side,
* then a curve, etc. I don't know if waypoints will be used or not, but they are in the comments if needed.
*/
public class AutoMode1 extends InstantCommand { //TODO: Change name to something relevant to path or function

  AutoTrajectory trajectory; 
  public AutoMode1(DriveSubsystem subsystem) { 
    
    //The X,Y, and the rotation angle in radians
    Pose2d start_position = new Pose2d(1.0, 1.0, new Rotation2d(3.0)); //TODO:Change these values
    Pose2d end_position = new Pose2d(1.0, 1.0, new Rotation2d(6.0)); //TODO:Change these values

    //TODO: Change these values
    
    /*List<Translation2d> waypoints = new ArrayList<Translation2d>(); //Creates an array of 2dTranslations with the interior waypoints included
    waypoints.add(new Translation2d(1.0, 1.0)); //Creating the waypoints and setting X,Y, and Translation2d values as their parameters
    waypoints.add(new Translation2d(1.0, 1.0));
    waypoints.add(new Translation2d(1.0, 1.0));*/

    trajectory = new AutoTrajectory(subsystem, start_position, end_position /*, waypoints if needed*/); /*Auto generates a new trajectory
    *from the subsystem, the start and end position coordinates and rotation, and from the interior waypoints of the path*/
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}

