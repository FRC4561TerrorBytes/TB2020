/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.AutoModeConstants.RedTrenchBall3;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

// Gets the third ball in the trench on the Red Alliance's side of the field.
public class RedSideTrenchBall3 extends InstantCommand {
  public RedSideTrenchBall3(DriveSubsystem subsystem) throws IOException{
    new AutoTrajectory(subsystem, RedTrenchBall3.trajectoryJSON);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
}
