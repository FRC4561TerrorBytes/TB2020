/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class DriveSubsystem extends PIDSubsystem {

  private DifferentialDrive drivetrain;

  private final WPI_TalonFX LEFT_MASTER_MOTOR = new WPI_TalonFX(Constants.FRONT_LEFT_MOTOR_PORT);
  private final WPI_TalonFX LEFT_REAR_SLAVE = new WPI_TalonFX(Constants.REAR_LEFT_MOTOR_PORT);

  private final WPI_TalonFX RIGHT_MASTER_MOTOR = new WPI_TalonFX(Constants.FRONT_RIGHT_MOTOR_PORT);
  private final WPI_TalonFX RIGHT_REAR_SLAVE = new WPI_TalonFX(Constants.REAR_RIGHT_MOTOR_PORT);

  private final double MIN_TOLERANCE = 1.0;

  private final AHRS NAVX = new AHRS(SPI.Port.kMXP);

  private double speed = 0.0;
  private double turn_scalar = 1.0;
  private double deadband = 0.0; 

  private boolean was_turning = false;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param period Time (in seconds) between PID calculations
   * @param tolerance Allowed closed loop error (degrees)
   * @param turn_scalar Turn sensitivity
   * @param deadband Deadzone for joystick
   */
  public DriveSubsystem(double kP, double kD, double period, double tolerance, double turn_scalar, double deadband) {
      // The PIDController used by the subsystem
      super(new PIDController(kP, 0, kD, period));

      // Make mid and rear left motor controllers follow left master
      LEFT_REAR_SLAVE.set(ControlMode.Follower, LEFT_MASTER_MOTOR.getDeviceID());

      // Make mid and rear right motor controllers follow right master
      RIGHT_REAR_SLAVE.set(ControlMode.Follower, RIGHT_MASTER_MOTOR.getDeviceID());

      // Wait for Robot init before finishing DriveSubsystem init
      try { Thread.sleep(10000); }
      catch (Exception e) { e.printStackTrace(); }

      // Initialise PID subsystem setpoint and input
      this.resetAngle();
      this.setSetpoint(0);

      // Set drive PID tolerance, minimum is 1 degree
      if (tolerance < this.MIN_TOLERANCE) tolerance = this.MIN_TOLERANCE;
      this.getController().setTolerance(tolerance);

      // Instantiate drive object
      this.drivetrain = new DifferentialDrive(LEFT_MASTER_MOTOR, RIGHT_MASTER_MOTOR);

      // Disable built in deadband application
      this.drivetrain.setDeadband(0);

      this.turn_scalar = turn_scalar;
      this.deadband = deadband;
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    this.drivetrain.arcadeDrive(this.speed, -output, false);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return this.getAngle();
  }

  /**
   * Call this repeatedly to drive using PID during teleoperation
   * @param speed Desired speed from -1.0 to 1.0
   * @param turn_request Turn input from -1.0 to 1.0
   */
  public void teleopPID(double speed, double turn_request) {
    // Set drive speed if it is more than the deadband
    if (Math.abs(speed) >= this.deadband) this.setSpeed(speed);
    else this.stop();
    
    // Start turning if input is greater than deadband
    if (Math.abs(turn_request) >= this.deadband) {
      // Add delta to setpoint scaled by factor
      this.setSetpoint(this.getController().getSetpoint() + (turn_request * this.turn_scalar));
      this.was_turning = true;
    } else { 
      // When turning is complete, set setpoint to current angle
      if (this.was_turning) {
        this.setSetpoint(this.getAngle());
        this.was_turning = false;
      }
    }
  }

  /**
   * Set DriveSubsystem speed
   * @param speed Desired speed from -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  /**
   * Stop drivetrain
   */
  public void stop() {
    this.setSpeed(0);
  }

  /**
   * Get DriveSubsystem speed
   * @return Speed
   */
  public double getSpeed() {
    return this.speed;
  }

  /**
   * Get DriveSubsystem angle as detected by the navX MXP
   * @return Total accumulated yaw angle
   */
  public double getAngle() {
    // Get accumulated yaw angle
    double raw_angle = NAVX.getAngle();

    // Get fused heading and convert from [0, 360] to [-180, 180]
    double normalized_heading = ((NAVX.getFusedHeading() - 180) % 360) - 180;

    // Get number of whole rotations in degrees
    double num_rotations_degrees = Math.copySign(Math.abs(raw_angle) - (Math.abs(raw_angle) % 360), raw_angle);

    // Return the sum of whole rotations and heading
    return num_rotations_degrees + normalized_heading;
  }

  /**
   * Get DriveSubsystem turn scalar
   * @return Turn scalar
   */
  public double getTurnScalar() {
    return this.turn_scalar;
  }

  /**
   * Reset DriveSubsystem navX MXP yaw angle
   */
  public void resetAngle() {
    NAVX.reset();
  }

}
