/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  
  private Command autonomousCommand;

  private RobotContainer robotContainer;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Reset DriveSubsystem PID
    //robotContainer.getDriveSubsystem().resetAngle();
    //robotContainer.getDriveSubsystem().setSetpoint(0);

    robotContainer.getDriveSubsystem().stop();

    // Reset Shooter Turret to front limit switch
    robotContainer.getShooterSubsystem().reset();

    autonomousCommand = robotContainer.getAutonomousCommand();

    // this.autoID = (int)SmartDashboard.getNumber("Choose Auto", 0);
    // this.autoVision = SmartDashboard.getBoolean("Auto Vision?", false);
    
   

    
    // new AutoTrajectory(robotContainer.getDriveSubsystem(), AutoModeConstants.BlueA.trajectoryJSON).initialize();


    // switch (this.autoID) {
    //   case 0:
    //   auto0();
    //   break;
    // }
      /*
      case 0:
      auto1(this.autoVision);
      robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_FRONT_LIMIT_POSITION);
      break;

      case 1:
      auto1(this.autoVision);
      robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_FRONT_LIMIT_POSITION);
      break;

      case 2:
      auto1(this.autoVision);
      robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_FRONT_LIMIT_POSITION);
      break;

      case 3:
      auto1(this.autoVision);
      robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_FRONT_LIMIT_POSITION);
      break;
      
    } */

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    
    
    
  }

  @Override
  public void teleopInit() {
    // Reset DriveSubsystem PID
    robotContainer.getDriveSubsystem().resetAngle();
    robotContainer.getDriveSubsystem().setSetpoint(0);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  

  /**
   * Default auto that moves robot off line.
   */
  // private void auto0() {
  //   long start = System.currentTimeMillis();
  //   while (System.currentTimeMillis() - start < 3000) {
  //     robotContainer.getDriveSubsystem().teleop(-.2, 0, 1);
  //   }
  //   robotContainer.getDriveSubsystem().teleop(0, 0, 1);
  // }

  /**
   * Robot starts on the line, intake facing away, shoots 3 balls and drives straight off line.
   * @param useVision input if wanting to use vision
   */
  private void auto1(boolean useVision) {
    robotContainer.getShooterSubsystem().toggleHoodPosition();
    robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_BACK_POSITION);
    robotContainer.getShooterSubsystem().flywheelManual(-.72);
    try {Thread.sleep(2000);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    long start = System.currentTimeMillis();
    while (System.currentTimeMillis() - start < 3000) {
      robotContainer.getDriveSubsystem().teleop(.2, 0, 1);
    }
    robotContainer.getDriveSubsystem().teleop(0, 0, 1);
    robotContainer.getShooterSubsystem().toggleHoodPosition();
    robotContainer.getShooterSubsystem().flywheelManual(Constants.MOTOR_STOP);
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
  }

  /**
   * Robot starts on the line, intake facing away, shoots 3 balls and drives straight off line.
   * @param useVision input if wanting to use vision
   */
  private void auto2(boolean useVision) {
    robotContainer.getShooterSubsystem().toggleHoodPosition();
    robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_BACK_POSITION);
    robotContainer.getShooterSubsystem().flywheelManual(-.72);
    try {Thread.sleep(2000);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    long start = System.currentTimeMillis();
    while (System.currentTimeMillis() - start < 1500) {
      robotContainer.getDriveSubsystem().teleop(-.2, 0, 1);
    }
    robotContainer.getDriveSubsystem().teleop(0, 0, 1);
    robotContainer.getShooterSubsystem().toggleHoodPosition();
    robotContainer.getShooterSubsystem().flywheelManual(Constants.MOTOR_STOP);
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
  }

  /**
   * Robot starts on the line, intake facing away, shoots 3 balls and drives straight off line.
   * @param useVision input if wanting to use vision
   */
  private void auto3(boolean useVision) {
    robotContainer.getShooterSubsystem().toggleHoodPosition();
    robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_BACK_POSITION);
    robotContainer.getShooterSubsystem().flywheelManual(-1);
    try {Thread.sleep(2000);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
    try {Thread.sleep(500);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    long start = System.currentTimeMillis();
    while (System.currentTimeMillis() - start < 1500) {
      robotContainer.getDriveSubsystem().teleop(-.2, 0, 1);
    }
    robotContainer.getDriveSubsystem().teleop(0, 0, 1);
    robotContainer.getShooterSubsystem().toggleHoodPosition();
    robotContainer.getShooterSubsystem().flywheelManual(Constants.MOTOR_STOP);
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MOTOR_STOP);
  }

  // private void auto1(boolean useVision) {
  //   robotContainer.getShooterSubsystem().toggleHoodPosition();
  //   robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_BACK_POSITION);
  //   try {Thread.sleep(1000);} catch (Exception e){}
  //   VisionData.setEnabled(useVision);
  //   try {Thread.sleep(3000);} catch (Exception e){}
  //   VisionData.setEnabled(false);
  //   robotContainer.getShooterSubsystem().flywheelManual(-1);
  //   try {Thread.sleep(1000);} catch (Exception e){}
  //   robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
  //   try {Thread.sleep(5000);} catch (Exception e){}
  //   long start = System.currentTimeMillis();
  //   while (System.currentTimeMillis() - start < 3000) {
  //     robotContainer.getDriveSubsystem().teleop(-.2, 0, 1);
  //   }
  //   robotContainer.getDriveSubsystem().teleop(0, 0, 1);
  //   robotContainer.getShooterSubsystem().toggleHoodPosition();
  //   robotContainer.getShooterSubsystem().flywheelManual(0);
  //   robotContainer.getMagazineSubsystem().ballUptake(0);
  // }

  /**
   * Robot starts on the left side, intake facing away, shoots 3 balls and drives slightly to the right off the line
   * @param useVision
   */
  private void auto4(boolean useVision) {
    robotContainer.getShooterSubsystem().toggleHoodPosition();
    robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_BACK_POSITION);
    try {Thread.sleep(500);} catch (Exception e){}
    VisionData.setEnabled(useVision);
    try {Thread.sleep(1000);} catch (Exception e){}
    VisionData.setEnabled(false);
    robotContainer.getShooterSubsystem().flywheelManual(-1);
    try {Thread.sleep(1000);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(5000);} catch (Exception e){}
    long start = System.currentTimeMillis();
    while (System.currentTimeMillis() - start < 3000) {
      robotContainer.getDriveSubsystem().teleop(-.2, 0.3, 1);
    }
    robotContainer.getDriveSubsystem().teleop(0, 0, 1);
    robotContainer.getShooterSubsystem().flywheelManual(0);
    robotContainer.getMagazineSubsystem().ballUptake(0);
    robotContainer.getShooterSubsystem().toggleHoodPosition();
  }

  /**
   * Robot starts on the right side, intake facing away, shoots 3 balls and drives slightly to the left off the line
   * @param useVision
   */
  private void auto5(boolean useVision) {
    robotContainer.getShooterSubsystem().toggleHoodPosition();
    robotContainer.getShooterSubsystem().moveTurretPID(Constants.TURRET_BACK_POSITION);
    try {Thread.sleep(500);} catch (Exception e){}
    VisionData.setEnabled(useVision);
    try {Thread.sleep(1000);} catch (Exception e){}
    VisionData.setEnabled(false);
    robotContainer.getShooterSubsystem().flywheelManual(-1);
    try {Thread.sleep(1000);} catch (Exception e){}
    robotContainer.getMagazineSubsystem().ballUptake(Constants.MAGAZINE_UP_MOTOR_SPEED);
    try {Thread.sleep(5000);} catch (Exception e){}
    long start = System.currentTimeMillis();
    while (System.currentTimeMillis() - start < 3000) {
      robotContainer.getDriveSubsystem().teleop(-.2, -0.3, 1);
    }
    robotContainer.getDriveSubsystem().teleop(0, 0, 1);
    robotContainer.getShooterSubsystem().flywheelManual(0);
    robotContainer.getMagazineSubsystem().ballUptake(0);
    robotContainer.getShooterSubsystem().toggleHoodPosition();
  }
}
