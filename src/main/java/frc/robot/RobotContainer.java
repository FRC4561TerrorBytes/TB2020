/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoException;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoTrajectory;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveTurretManualCommand;
import frc.robot.commands.TurretSetpointCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MagazineSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final DriveSubsystem DRIVE_SUBSYSTEM = new DriveSubsystem(Constants.DRIVE_kP, Constants.DRIVE_kD,
      Constants.DRIVE_PERIOD_SECONDS, Constants.DRIVE_TOLERANCE, Constants.DRIVE_TURN_SCALAR, Constants.DEADBAND);

  private static final ClimberSubsystem CLIMBER_SUBSYSTEM = new ClimberSubsystem();

  public static final MagazineSubsystem MAGAZINE_SUBSYSTEM = new MagazineSubsystem(Constants.ARM_CONFIG);

  private static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem(Constants.FLYWHEEL_MASTER_CONFIG,
      Constants.HOOD_CONFIG, Constants.TURRET_CONFIG);

  public static final XboxController XBOX_CONTROLLER = new XboxController(Constants.XBOX_CONTROLLER_PORT);

  private static final Joystick LEFT_JOYSTICK = new Joystick(Constants.LEFT_JOYSTICK_PORT);
  private static final Joystick RIGHT_JOYSTICK = new Joystick(Constants.RIGHT_JOYSTICK_PORT);

  public static UsbCamera camera1;
  public static UsbCamera camera2;

  
  
  
  
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Start streaming driver camera
    initializeCamera();

    // Enable PID on drive subsytem
    //DRIVE_SUBSYSTEM.enable();

    // Set default commands for subsystems
    DRIVE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> DRIVE_SUBSYSTEM.teleop(LEFT_JOYSTICK.getY(), RIGHT_JOYSTICK.getX(), Constants.DRIVE_RESPONSE_EXPONENT), DRIVE_SUBSYSTEM));
    MAGAZINE_SUBSYSTEM.setDefaultCommand(new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(), MAGAZINE_SUBSYSTEM));

    // chooser.addOption("Auto test", new AutoTest(DRIVE_SUBSYSTEM));
    chooser.addOption("Auto test", new RunCommand(() -> new AutoTrajectory(DRIVE_SUBSYSTEM, AutoModeConstants.DriveStraightTest.trajectoryJSON), DRIVE_SUBSYSTEM));
  }

  //  static SendableChooser<SequentialCommandGroup> chooser = new SendableChooser<>();
  static SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // new JoystickButton(XBOX_CONTROLLER, Button.kB.value)
    //     .whileHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(-Constants.MAGAZINE_MOTOR_SPEED), MAGAZINE_SUBSYSTEM))
    //     .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.magazineMotorStop(), MAGAZINE_SUBSYSTEM));


    // // Moves climber lift at 0.5 speed when X button is pressed
    // new JoystickButton(XBOX_CONTROLLER, Button.kX.value)
    //     .whileHeld(new RunCommand(() -> CLIMBER_SUBSYSTEM.liftManual(Constants.CLIMBER_LIFT_CONSTANT), CLIMBER_SUBSYSTEM));
    
    // Moves climber hook at 0.5 speed when Y button is pressed
    // new JoystickButton(XBOX_CONTROLLER, Button.kY.value)
    //     .whileHeld(new RunCommand(() -> CLIMBER_SUBSYSTEM.hookManual(Constants.CLIMBER_HOOK_CONSTANT), CLIMBER_SUBSYSTEM));

    // new JoystickButton(XBOX_CONTROLLER, Button.kB.value)
    //     .whenPressed(new InstantCommand(() -> MAGAZINE_SUBSYSTEM.toggleArmPosition()));

    // new POVButton(XBOX_CONTROLLER, 90).whileHeld(new HoodCommand(SHOOTER_SUBSYSTEM, -0.3))
    //   .whenReleased(new HoodCommand(SHOOTER_SUBSYSTEM, Constants.MOTOR_STOP));

    // new POVButton(XBOX_CONTROLLER, 270).whileHeld(new HoodCommand(SHOOTER_SUBSYSTEM, 0.3))
    //   .whenReleased(new HoodCommand(SHOOTER_SUBSYSTEM, Constants.MOTOR_STOP));

    // new POVButton(XBOX_CONTROLLER, 0).whileHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.armManual(-.5), MAGAZINE_SUBSYSTEM))
    //   .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.armManual(Constants.MOTOR_STOP), MAGAZINE_SUBSYSTEM));
    // new POVButton(XBOX_CONTROLLER, 180).whileHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.armManual(.5), MAGAZINE_SUBSYSTEM))
    //   .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.armManual(Constants.MOTOR_STOP), MAGAZINE_SUBSYSTEM));


    // new Trigger(() -> (XBOX_CONTROLLER.getTriggerAxis(Hand.kRight) > .2))
    //             .whenActive(new RunCommand(() -> SHOOTER_SUBSYSTEM.relativeMoveTurretPID(XBOX_CONTROLLER.getTriggerAxis(Hand.kRight) * 10), SHOOTER_SUBSYSTEM));

    // new Trigger(() -> (XBOX_CONTROLLER.getTriggerAxis(Hand.kLeft) > .2))
    //             .whenActive(new RunCommand(() -> SHOOTER_SUBSYSTEM.relativeMoveTurretPID(XBOX_CONTROLLER.getTriggerAxis(Hand.kLeft) * 10), SHOOTER_SUBSYSTEM));
    
  
    // new POVButton(XBOX_CONTROLLER, 90).whenPressed(new RunCommand(() -> CLIMBER_SUBSYSTEM.hookManual(1), CLIMBER_SUBSYSTEM))
    //               .whenReleased(new RunCommand(() -> CLIMBER_SUBSYSTEM.hookManual(Constants.MOTOR_STOP), CLIMBER_SUBSYSTEM));
            
    // new POVButton(XBOX_CONTROLLER, 270).whenPressed(new RunCommand(() -> CLIMBER_SUBSYSTEM.hookManual(-1), CLIMBER_SUBSYSTEM))
    //               .whenReleased(new RunCommand(() -> CLIMBER_SUBSYSTEM.hookManual(Constants.MOTOR_STOP), CLIMBER_SUBSYSTEM));

    // Reset the arm encoder to the back position
    // new Trigger(() -> MAGAZINE_SUBSYSTEM.armLimit())
    //             .whileActiveOnce(new InstantCommand(() -> MAGAZINE_SUBSYSTEM.getArmMotor().setSelectedSensorPosition(Constants.ARM_BOTTOM_POSITION),
    //             MAGAZINE_SUBSYSTEM));
    

    // TODO: Driver buttons here!!!
//     
// 
// 
//  
    // Right joystick intake button 1
    // new JoystickButton(RIGHT_JOYSTICK, 1)
    //   .whileHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.intakeMotorSpeed(Constants.INTAKE_MOTOR_SPEED), MAGAZINE_SUBSYSTEM))
    //   .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.intakeMotorSpeed(Constants.MOTOR_STOP), MAGAZINE_SUBSYSTEM));

    new JoystickButton(RIGHT_JOYSTICK, 1)
      .whileHeld(new IntakeCommand(MAGAZINE_SUBSYSTEM, Constants.INTAKE_MOTOR_SPEED))
      .whenReleased(new IntakeCommand(MAGAZINE_SUBSYSTEM, Constants.MOTOR_STOP));

    // Right joystick Hood Toggle button 2
    new JoystickButton(RIGHT_JOYSTICK, 2)
      .whenPressed(new InstantCommand(() -> SHOOTER_SUBSYSTEM.toggleHoodPosition(), SHOOTER_SUBSYSTEM));

    // Right joystick shoot button 3
    new JoystickButton(RIGHT_JOYSTICK, 3)
      .whileHeld(new RunCommand(() -> SHOOTER_SUBSYSTEM.flywheelManual(-1), SHOOTER_SUBSYSTEM));

    new JoystickButton(RIGHT_JOYSTICK, 3)
      .whenReleased(new RunCommand(() -> SHOOTER_SUBSYSTEM.setFlywheelSpeed(0), SHOOTER_SUBSYSTEM));
      // .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(0), MAGAZINE_SUBSYSTEM));

    // Right joystick shoot button 4
    // new JoystickButton(RIGHT_JOYSTICK, 4)
    //   .whileHeld(new RunCommand(() -> SHOOTER_SUBSYSTEM.setFlywheelSpeed(-17600), SHOOTER_SUBSYSTEM).alongWith(
    //     new ConditionalCommand(
    //       new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(Constants.MAGAZINE_MOTOR_SPEED)), 
    //       new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(Constants.MOTOR_STOP)), 
    //       () -> SHOOTER_SUBSYSTEM.isFlywheelAtSpeed())));
      
    // new JoystickButton(RIGHT_JOYSTICK, 4)
    //   .whenReleased(new RunCommand(() -> SHOOTER_SUBSYSTEM.setFlywheelSpeed(0), SHOOTER_SUBSYSTEM))
    //   .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(0), MAGAZINE_SUBSYSTEM));

    // Left joystick intake button 1
    new JoystickButton(LEFT_JOYSTICK, 1)
      .whileHeld(new IntakeCommand(MAGAZINE_SUBSYSTEM, -Constants.OUTTAKE_MOTOR_SPEED))
      .whenReleased(new IntakeCommand(MAGAZINE_SUBSYSTEM, Constants.MOTOR_STOP));

    // Auto drive button 1
    // new JoystickButton(LEFT_JOYSTICK, 1)
    //   .whileHeld(new AutoDriveCommand(DRIVE_SUBSYSTEM, SHOOTER_SUBSYSTEM));
    
    // Right joystick Arm Toggle button 2
    new JoystickButton(LEFT_JOYSTICK, 2)
      .whenPressed(new InstantCommand(() -> MAGAZINE_SUBSYSTEM.toggleArmPosition(), MAGAZINE_SUBSYSTEM));

    // Right joystick turret to 180 button 3
    new JoystickButton(LEFT_JOYSTICK, 3)
      .whenPressed(new TurretSetpointCommand(SHOOTER_SUBSYSTEM, Constants.TURRET_BACK_POSITION));

    // Right joystick turret to 0 button 4
    new JoystickButton(LEFT_JOYSTICK, 4)
      .whenPressed(new TurretSetpointCommand(SHOOTER_SUBSYSTEM, Constants.TURRET_STRAIGHT_POSITION));

    // Controller turret to 180 button 1
    new JoystickButton(XBOX_CONTROLLER, 1)
      .whenPressed(new TurretSetpointCommand(SHOOTER_SUBSYSTEM, Constants.TURRET_BACK_POSITION));
    // Controller turret to 90 button 2
    new JoystickButton(XBOX_CONTROLLER, 2)
      .whenPressed(new TurretSetpointCommand(SHOOTER_SUBSYSTEM, Constants.TURRET_MIDDLE_POSITION));

    // Controller turret to front limit button 3
    new JoystickButton(XBOX_CONTROLLER, 3)
      .whenPressed(new TurretSetpointCommand(SHOOTER_SUBSYSTEM, Constants.TURRET_FRONT_LIMIT_POSITION));

    // Controller turret to 0 button 4
    new JoystickButton(XBOX_CONTROLLER, 4)
      .whenPressed(new TurretSetpointCommand(SHOOTER_SUBSYSTEM, Constants.TURRET_STRAIGHT_POSITION));

    new JoystickButton(XBOX_CONTROLLER, Button.kBumperLeft.value)
      .whileHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.armPositionRelative(-Constants.ARM_MANUAL_INCREMENT), MAGAZINE_SUBSYSTEM));

    new JoystickButton(XBOX_CONTROLLER, Button.kBumperRight.value)
      .whileHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.armPositionRelative(Constants.ARM_MANUAL_INCREMENT), MAGAZINE_SUBSYSTEM));

    // Vision toggle
    VisionData.setEnabled(false);
    new JoystickButton(XBOX_CONTROLLER, Button.kStart.value)
      .whenPressed(new InstantCommand(() -> VisionData.toggle()));

    new JoystickButton(XBOX_CONTROLLER, Button.kBack.value)
      .whenHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.armManual(.6)))
      .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.armPositionRelative(0)));

    // Uptake
    new POVButton(XBOX_CONTROLLER, 0).whileHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED), MAGAZINE_SUBSYSTEM))
      .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(Constants.MOTOR_STOP)));

    new POVButton(XBOX_CONTROLLER, 180).whileHeld(new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(-Constants.MAGAZINE_DOWN_MOTOR_SPEED), MAGAZINE_SUBSYSTEM))
      .whenReleased(new RunCommand(() -> MAGAZINE_SUBSYSTEM.ballUptake(Constants.MOTOR_STOP)));;

      /*
    // Controller mouse droid left/right button 5 // TODO: change to left or right
    // new JoystickButton(XBOX_CONTROLLER, 5)
    //   .whenHeld(new RunCommand(() -> CLIMBER_SUBSYSTEM.mouseDroidManual(Constants.MOUSE_DROID_SPEED), CLIMBER_SUBSYSTEM))
    //   .whenReleased(new RunCommand(() -> CLIMBER_SUBSYSTEM.mouseDroidManual(Constants.MOTOR_STOP), CLIMBER_SUBSYSTEM));

    // Controller mouse droid left/right button 6 // TODO: change to left or right
    new JoystickButton(XBOX_CONTROLLER, 6)
      .whenHeld(new RunCommand(() -> CLIMBER_SUBSYSTEM.mouseDroidManual(-Constants.MOUSE_DROID_SPEED), CLIMBER_SUBSYSTEM))
      .whenReleased(new RunCommand(() -> CLIMBER_SUBSYSTEM.mouseDroidManual(Constants.MOTOR_STOP), CLIMBER_SUBSYSTEM));;

    */

    // Controller turret left, trigger left
    new Trigger(() -> (XBOX_CONTROLLER.getTriggerAxis(Hand.kLeft) > Constants.DEADBAND))
      .whenActive(new MoveTurretManualCommand(SHOOTER_SUBSYSTEM, () -> -XBOX_CONTROLLER.getTriggerAxis(Hand.kLeft) * 25));
    
    // Controller turret right, trigger right
    new Trigger(() -> (XBOX_CONTROLLER.getTriggerAxis(Hand.kRight) > Constants.DEADBAND))
      .whenActive(new MoveTurretManualCommand(SHOOTER_SUBSYSTEM, () -> XBOX_CONTROLLER.getTriggerAxis(Hand.kRight) * 25));

    // new POVButton(XBOX_CONTROLLER, 90).whileHeld(new HoodCommand(SHOOTER_SUBSYSTEM, -0.3))
    //   .whenReleased(new HoodCommand(SHOOTER_SUBSYSTEM, Constants.MOTOR_STOP));

    // new POVButton(XBOX_CONTROLLER, 270).whileHeld(new HoodCommand(SHOOTER_SUBSYSTEM, 0.3))
    //   .whenReleased(new HoodCommand(SHOOTER_SUBSYSTEM, Constants.MOTOR_STOP));

    // Controller Lift right stick
    // new Trigger(() -> (Math.abs(XBOX_CONTROLLER.getY(Hand.kRight)) > Constants.DEADBAND))
    //   .whenActive(new WinchCommand(CLIMBER_SUBSYSTEM, () -> -Math.abs(XBOX_CONTROLLER.getY(Hand.kRight))))
    //   .whenInactive(new WinchCommand(CLIMBER_SUBSYSTEM, () -> Constants.MOTOR_STOP));

      // Controller Hook left stick
    // new Trigger(() -> (Math.abs(XBOX_CONTROLLER.getY(Hand.kLeft)) > Constants.DEADBAND))
    //   .whenActive(new HookCommand(CLIMBER_SUBSYSTEM, () -> XBOX_CONTROLLER.getY(Hand.kLeft)))
    //   .whenInactive(new HookCommand(CLIMBER_SUBSYSTEM, () -> Constants.MOTOR_STOP));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
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
   public MagazineSubsystem getMagazineSubsystem() {
     return MAGAZINE_SUBSYSTEM;
   }

   /**
    * Get ShooterSubsystem
    * @return shootersubsystem
    */
   public ShooterSubsystem getShooterSubsystem() {
     return SHOOTER_SUBSYSTEM;
   }

   /**
    * Get ClimberSubsystem
    * @return climbersubsystem
    */
    public ClimberSubsystem getClimberSubsystem() {
      return CLIMBER_SUBSYSTEM;
    }

   /**
    * Initializes the camera(s)
    */
   public void initializeCamera() {
    try {

      // Intake
      camera1 = CameraServer.getInstance().startAutomaticCapture();
      camera1.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
      camera1.setResolution(176, 144);
      camera1.setFPS(15); // Can go up to 30
      camera1.setBrightness(25);
      camera1.setExposureManual(10);
      camera1.setWhiteBalanceManual(10);
      
      // Shooter
      camera2 = CameraServer.getInstance().startAutomaticCapture();
      camera2.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen);
      camera2.setResolution(176, 144);
      camera2.setFPS(15); // Can go up to 30
      camera2.setBrightness(25);
      camera2.setExposureManual(10);
      camera2.setWhiteBalanceManual(10);

    } catch (VideoException e) {
      e.printStackTrace();
    }
   }
   
}
