/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoTrajectory;
//import frc.robot.commands.BlueAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //DriveSubsystem subsystem;
  
  private final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(Constants.DRIVE_kP,
                                                                            Constants.DRIVE_kD, 
                                                                            Constants.DRIVE_PERIOD_SECONDS,
                                                                            Constants.DRIVE_TOLERANCE,
                                                                            Constants.DRIVE_TURN_SCALAR,
                                                                            Constants.DEADBAND,
                                                                            Constants.AUTO_SPEED);
  
  private static final XboxController XBOX_CONTROLLER = new XboxController(Constants.XBOX_CONTROLLER_PORT);
  
  private static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem(Constants.FLYWHEEL_CONFIG, Constants.HOOD_CONFIG, Constants.TURRET_CONFIG);
  

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
   
     //Creates a new Shuffleboard option that displays the String as its name, and its option is a 
    // new AutoTrajectory(DRIVE_SUBSYSTEM, AutoModeConstants.Example.name()); with subsystem (like normal)
    //as its params, as well as its custom and corresponding trajectoryJSON String in AutoModeConstants
    chooser.setDefaultOption("Hyperion Test Auto", new AutoTrajectory(DRIVE_SUBSYSTEM, AutoModeConstants.BlueTestAuto.name()));
    //chooser.addOption("Blue Auto Mode 1", new BlueAuto(subsystem));

    //FOR MANUAL MODE ONLY!!!!
    // chooser.addOption("Blue Auto Manual", new BlueAutoManual(subsystem, Pose2d startPosition, Pose2d endPosition /*,List<Translation2d> interiorWaypoints*/ ));
  }

  private void configureButtonBindings() {
    // new JoystickButton(XBOX_CONTROLLER, Button.kA.value)
    //     .whenPressed(new InstantCommand(() -> DRIVE_SUBSYSTEM.setSetpoint(180), DRIVE_SUBSYSTEM));

    // // When pressed, moves hood to top position
    
    // new JoystickButton(XBOX_CONTROLLER, Button.kY.value)
    //     .whenPressed(new InstantCommand(() -> SHOOTER_SUBSYSTEM.moveHoodPID(Constants.HOOD_TOP_POSITION), SHOOTER_SUBSYSTEM));

        //When pressed, moves hood to bottom position

    // new JoystickButton(XBOX_CONTROLLER, Button.kY.value)
    //     .whenPressed(new InstantCommand(() -> SHOOTER_SUBSYSTEM.moveHoodPID(Constants.HOOD_BOTTOM_POSITION), SHOOTER_SUBSYSTEM));
    
    // Control the Flywheel at a set speed using the right bumber
    new JoystickButton(XBOX_CONTROLLER, Button.kBumperRight.value)
        .whenPressed(new RunCommand(() -> SHOOTER_SUBSYSTEM.flywheelManual(-0.2), SHOOTER_SUBSYSTEM))
        .whenReleased(new RunCommand(() -> SHOOTER_SUBSYSTEM.flywheelManual(0.0), SHOOTER_SUBSYSTEM));

    // Control the Flywheel using PID using the left bumber
    new JoystickButton(XBOX_CONTROLLER, Button.kBumperLeft.value)
        .whenPressed(new RunCommand(() -> SHOOTER_SUBSYSTEM.setFlywheelSpeed(-1000), SHOOTER_SUBSYSTEM))
        .whenReleased(new RunCommand(() -> SHOOTER_SUBSYSTEM.setFlywheelSpeed(0), SHOOTER_SUBSYSTEM));
    
    // Move the hood down using the A button
    new JoystickButton(XBOX_CONTROLLER, Button.kA.value)
        .whenPressed(new RunCommand(() -> SHOOTER_SUBSYSTEM.hoodManual(-0.3), SHOOTER_SUBSYSTEM))
        .whenReleased(new RunCommand(() -> SHOOTER_SUBSYSTEM.hoodManual(0.0), SHOOTER_SUBSYSTEM));

    // Move the hood up using the Y button
    new JoystickButton(XBOX_CONTROLLER, Button.kY.value)
        .whenPressed(new RunCommand(() -> SHOOTER_SUBSYSTEM.hoodManual(0.3), SHOOTER_SUBSYSTEM))
        .whenReleased(new RunCommand(() -> SHOOTER_SUBSYSTEM.hoodManual(0), SHOOTER_SUBSYSTEM));

    // Reset the turret encoder to the front position
    if (SHOOTER_SUBSYSTEM.turretLimitFront()) {
      SHOOTER_SUBSYSTEM.getTurretMotor().setSelectedSensorPosition(Constants.TURRET_FRONT_POSITION);
    }
    
    // Reset the turret encoder to the back position
    if (SHOOTER_SUBSYSTEM.turretLimitBack()) {
       SHOOTER_SUBSYSTEM.getTurretMotor().setSelectedSensorPosition(Constants.TURRET_BACK_POSITION);
    }

    // Reset the hood encoder to the back position
    if (SHOOTER_SUBSYSTEM.hoodLimit()) {
      SHOOTER_SUBSYSTEM.getHoodMotor().setSelectedSensorPosition(Constants.HOOD_BOTTOM_POSITION);
    }
  }

  static SendableChooser<Command> chooser = new SendableChooser<>();//Creates a SendableChooser for the 
  //SmartDashboard/Shuffleboard. 
   
  

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  /*private void configureButtonBindings() {
    new JoystickButton(XBOX_CONTROLLER, Button.kA.value)
        .whenPressed(new InstantCommand(() -> DRIVE_SUBSYSTEM.setSetpoint(180), DRIVE_SUBSYSTEM));
  }*/


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return new RunCommand(() -> DRIVE_SUBSYSTEM.driveStraight(30), DRIVE_SUBSYSTEM); LEAVE HERE! MAY BE USED?
    return chooser.getSelected();
  }

  

  /**
   * Get DriveSubsystem
   * @return drivesubsystem
   */
  public DriveSubsystem getDriveSubsystem() {
    return DRIVE_SUBSYSTEM;
  }
  



  
}
