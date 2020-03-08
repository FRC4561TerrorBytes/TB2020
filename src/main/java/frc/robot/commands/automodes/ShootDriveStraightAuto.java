/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.automodes;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoModeConstants;
import frc.robot.AutoTrajectory;
import frc.robot.Constants;
import frc.robot.commands.TurretSetpointCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootDriveStraightAuto extends SequentialCommandGroup {
  /**
   * Creates a new ShootDriveStraightAuto.
   */
  public ShootDriveStraightAuto(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem, MagazineSubsystem magazineSubsystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_BACK_POSITION, false),
          race(
            new RunCommand(() -> shooterSubsystem.setFlywheelSpeed(-17600), shooterSubsystem),
            new ConditionalCommand(new RunCommand(() -> magazineSubsystem.ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED), magazineSubsystem),
                                   new RunCommand(() -> magazineSubsystem.ballUptake(Constants.MOTOR_STOP), magazineSubsystem),
                                   () -> shooterSubsystem.isFlywheelAtSpeed())
          ).withTimeout(7),
          new AutoTrajectory(driveSubsystem, AutoModeConstants.ShootDriveStraight.trajectoryJSON).getCommand(),
          race(
          new RunCommand(() -> shooterSubsystem.setFlywheelSpeed(Constants.MOTOR_STOP), shooterSubsystem),
          new RunCommand(() -> magazineSubsystem.ballUptake(Constants.MOTOR_STOP), magazineSubsystem)).withTimeout(0),
          new TurretSetpointCommand(shooterSubsystem, Constants.TURRET_FRONT_LIMIT_POSITION, false)
    );
  }
}
