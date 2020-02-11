/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoModeConstants.MoveBackAfterShooting3Balls;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

//Moves the robot back after shooting the 3 balls preloaded in robot into the Power  Port..
public class MoveBackShootingAutoBalls extends InstantCommand {
  public MoveBackShootingAutoBalls(DriveSubsystem subsystem) throws IOException{
    new AutoTrajectory(subsystem, MoveBackAfterShooting3Balls.trajectoryJSON);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
