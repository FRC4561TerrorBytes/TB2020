/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoModeConstants.AM_PATH1;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

/*This automode is to just test basic position movement and only the drivetrain motors. Right now, it is just 
* a simple change in rotation. Later, it will be changed into slight change in forward movement, then side to side,
* then a curve, etc. I don't know if waypoints will be used or not, but they are in the comments if needed.
*/
public class AutoMode1 extends InstantCommand { // TODO: Change name to something relevant to path or function

  
  public AutoMode1(DriveSubsystem subsystem) {

    new AutoTrajectory(subsystem, AM_PATH1.trajectoryJSON); /*Auto generates a new trajectory
    *from the DriveSubsystem, and from AutoTrajectory, but with a assigned path */
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}

