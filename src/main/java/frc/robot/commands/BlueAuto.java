/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class BlueAuto extends SequentialCommandGroup {

  /**
   * Creates a new Blue Side Auto Auto. This will be a SequentialCommandGroup (a
   * list) of commands that will run one after another in the order it is put into
   * the syntax. This is what an auto mode will likely consist of.
   */

  public BlueAuto(DriveSubsystem subsystem, ShooterSubsystem SHOOTER_SUBSYSTEM) {

    addCommands(
     //Aim towards target using vision
      new RunCommand(() -> 
      SHOOTER_SUBSYSTEM.turretVisionPID(),
       SHOOTER_SUBSYSTEM),
    
      //Set flywheel speed to -1000 or counterclockwise rotations in RPM
       new RunCommand(() -> {
      SHOOTER_SUBSYSTEM.setFlywheelSpeed(-1000);
      try 
        {
        Thread.sleep(2000); //Keeps the flywheel speed at -1000 RPM for 2000ms/2s (or however long it will take to fire the 3 balls)
        } 
      catch (InterruptedException e) 
        {
        e.printStackTrace(); // If catches an error, will set flywheel speed to 0 before it can finish closing error
        }
      finally
        {
        SHOOTER_SUBSYSTEM.setFlywheelSpeed(0); // Sets flywheel speed to 0 RPM after 3 preloaded balls have been shot.
        }
    }, SHOOTER_SUBSYSTEM)
    
    
        
        );
      }
  }
