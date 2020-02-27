/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.AutoModeConstants;
//import frc.robot.subsystems.DriveSubsystem;
//import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

  
public class TwoPathBasicAuto extends SequentialCommandGroup {
  
  public TwoPathBasicAuto(DriveSubsystem subsystem) {
    addCommands(
    new AutoTrajectory(subsystem, AutoModeConstants.BlueA.trajectoryJSON),
    new WaitCommand(2), // Waits for 2 seconds inbetween Commands
    new PrintCommand("Blue A Finished"), // Prints that the Command is finished in the Terminal Logs
     new AutoTrajectory(subsystem, AutoModeConstants.BlueAMovetoTrench.trajectoryJSON),
     new WaitCommand(2),
     new PrintCommand("Blue A Move To Trench Finished"),
    new AutoTrajectory(subsystem, AutoModeConstants.BlueSideTrenchBalls.trajectoryJSON),
    new WaitCommand(2),
    new PrintCommand("Blue Side Trench Balls Finished"),
    new AutoTrajectory(subsystem, AutoModeConstants.BlueFroTrenchReverse.trajectoryJSON),
    new WaitCommand(2),
    new PrintCommand("Blue From Trench Reverse Finsihed")
    );
 
    
  }
}
