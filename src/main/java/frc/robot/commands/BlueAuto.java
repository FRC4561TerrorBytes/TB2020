/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class BlueAuto extends SequentialCommandGroup {


  /**
   * Creates a new Blue Side Auto Auto. This will be a SequentialCommandGroup (a list) of commands that will run 
   * one after another in the order it is put into the syntax. This is what an auto mode will likely consist of.
   * 

   */
  public BlueAuto(DriveSubsystem subsystem){
  
    addCommands(new SequentialCommandGroup(/*new AutoTrajectory(subsystem, AutoModeConstants.Example)*/)); //TODO: Add Sequential Commands here to run in Autonomous when on blue side of field
   
  }
}
