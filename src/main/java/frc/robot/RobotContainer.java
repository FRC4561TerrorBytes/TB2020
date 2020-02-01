/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...


  
  private final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(Constants.DRIVE_kP,
                                                                            Constants.DRIVE_kD, 
                                                                            Constants.DRIVE_PERIOD_SECONDS,
                                                                            Constants.DRIVE_TOLERANCE,
                                                                            Constants.DRIVE_TURN_SCALAR,
                                                                            Constants.DEADBAND);

  private static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();

  public static final MagazineSubsystem MAGAZINE_SUBSYSTEM = new MagazineSubsystem();
  
  private static final XboxController XBOX_CONTROLLER = new XboxController(Constants.XBOX_CONTROLLER_PORT);


  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Enable PID on drive subsytem
    DRIVE_SUBSYSTEM.enable();
    // Set default command
    DRIVE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> DRIVE_SUBSYSTEM.teleopPID(XBOX_CONTROLLER.getY(Hand.kLeft), XBOX_CONTROLLER.getX(Hand.kRight)), DRIVE_SUBSYSTEM));
    // Alternative way of setting default command using command class
    //DRIVE_SUBSYSTEM.setDefaultCommand(new TeleopDriveCommand(DRIVE_SUBSYSTEM, () -> XBOX_CONTROLLER.getY(Hand.kLeft), () -> XBOX_CONTROLLER.getY(Hand.kRight)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(XBOX_CONTROLLER, Button.kA.value)
        .whileHeld(new RunCommand(() -> DRIVE_SUBSYSTEM.setSetpoint(180), DRIVE_SUBSYSTEM));

    // Moves climber lift at 0.5 speed when X button is pressed
    new JoystickButton(XBOX_CONTROLLER, Button.kX.value)
        .whileHeld(new RunCommand(() -> CLIMBER_SUBSYSTEM.liftManual(Constants.CLIMBER_LIFT_CONSTANT), CLIMBER_SUBSYSTEM));
    
    // Moves climber hook at 0.5 speed when Y button is pressed
    new JoystickButton(XBOX_CONTROLLER, Button.kY.value)
        .whileHeld(new RunCommand(() -> CLIMBER_SUBSYSTEM.hookManual(Constants.CLIMBER_HOOK_CONSTANT), CLIMBER_SUBSYSTEM));

    // When pressed, detects ball and spins motor at 0.5 speed
    new Trigger(MAGAZINE_SUBSYSTEM::ballDetected)
        .whenActive(new InstantCommand(() ->  MAGAZINE_SUBSYSTEM.magazineMotorSpeed(), MAGAZINE_SUBSYSTEM));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }

  /**
   * Get DriveSubsystem
   * @return drivesubsystem
   */
  public DriveSubsystem getDriveSubsystem() {
    return DRIVE_SUBSYSTEM;
  }

  /**
   * Get MagazineSubsystem
   * @return magazinesubsystem
   */

   public static MagazineSubsystem getMagazineSubsystem() {
     return MAGAZINE_SUBSYSTEM;
   }
}
