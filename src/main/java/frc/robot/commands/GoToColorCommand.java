/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SpinnerSubsystem;

public class GoToColorCommand extends CommandBase {

  SpinnerSubsystem subsystem;
  String targetColor;
  int motorDirection;
  /**
   * Creates a new GoToColorCommand.
   */
  public GoToColorCommand(SpinnerSubsystem subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.subsystem = subsystem;
    addRequirements(this.subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.targetColor = DriverStation.getInstance().getGameSpecificMessage();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.motorDirection = subsystem.goToColor(this.targetColor);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(this.motorDirection == 1) {
      subsystem.manualSpin(0.1);
      try { wait(100, 0); }
      catch (Exception e) { e.printStackTrace(); }
      subsystem.stopSpinner();
    } else if (this.motorDirection == -1) {
      subsystem.manualSpin(-0.1);
      try { wait(100, 0); }
      catch (Exception e) { e.printStackTrace(); }
      subsystem.stopSpinner();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.checkCorrectColor(this.targetColor);
  }
}
