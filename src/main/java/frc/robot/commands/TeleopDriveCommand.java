/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopDriveCommand extends CommandBase {
  private DriveSubsystem driveSubsystem;
  private DoubleSupplier speed;
  private DoubleSupplier turn_request;

  /**
   * Creates a new TeleopDriveCommand.
   */
  public TeleopDriveCommand(DriveSubsystem driveSubsystem, DoubleSupplier speed, DoubleSupplier turn_request) {
    this.driveSubsystem = driveSubsystem;
    this.speed = speed;
    this.turn_request = turn_request;
    this.addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Enable/start DriveSubsystem PID loop
    this.driveSubsystem.enable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive PID using speed and turn input from controller for testing
    this.driveSubsystem.teleopPID(this.speed.getAsDouble(), this.turn_request.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
