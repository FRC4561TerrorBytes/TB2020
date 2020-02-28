/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.TalonPIDConfig;
import frc.robot.VisionData;

public class ShooterSubsystem extends SubsystemBase {

  private final String SUBSYSTEM_NAME = "Shooter Subsystem";

  private static class Flywheel {
    private static double kF;
    private static final int TICKS_PER_ROTATION = 2048;
    // public static final double GEAR_RATIO = 3 / 4; The real thing
    public static final double GEAR_RATIO = 1.0;
    private static final WPI_TalonFX MASTER_MOTOR = new WPI_TalonFX(Constants.FLYWHEEL_MASTER_MOTOR_PORT);
    private static final WPI_TalonFX SLAVE_MOTOR = new WPI_TalonFX(Constants.FLYWHEEL_SLAVE_MOTOR_PORT);
    private static TalonPIDConfig masterConfig;
    private static double speed = 0;
  }

  private static class Hood {
    private static boolean needsReset = true;
    private static int topPosition = Constants.HOOD_TOP_POSITION;
    private static double bottomPosition = Constants.HOOD_BOTTOM_POSITION;
    private static final int TICKS_PER_ROTATION = 4096;
    public static final double GEAR_RATIO = 298 / 25;
    private static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(Constants.HOOD_MOTOR_PORT);
    private static TalonPIDConfig config;
  }

  private static class Turret {
    private static boolean needsReset = true;
    private static int leftPosition = Constants.TURRET_FRONT_LIMIT_POSITION;
    private static int straightPosition = Constants.TURRET_STRAIGHT_POSITION;
    private static int rightPosition = Constants.TURRET_BACK_LIMIT_POSITION;
    private static final int TICKS_PER_ROTATION = 4096;
    public static final int GEAR_RATIO = 94 / 15;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_ROTATION * GEAR_RATIO) / 360;
    private static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(Constants.TURRET_MOTOR_PORT);
    private static TalonPIDConfig config;

  }

  /**
  * Create an instance of ShooterSubsystem
  * <p>
  * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
  * <p>
  * NOTE: ASSUMES HOOD STARTS IN LOWEST POSITION!
  * <p>
  * NOTE: ASSUMES TURRET STARTS IN RIGHTMOST POSITION!
  * <p>
   * @param flywheelConfig PID config for Flywheel
   * @param hoodConfig PID config for Hood
   * @param turretConfig PID config for Turret
   */
  public ShooterSubsystem(TalonPIDConfig flywheelMasterConfig, TalonPIDConfig hoodConfig, TalonPIDConfig turretConfig) {

    Flywheel.masterConfig = flywheelMasterConfig;
    Hood.config = hoodConfig;
    Turret.config = turretConfig;

    

    Flywheel.masterConfig.initializeTalonPID(Flywheel.MASTER_MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, false, false);
    Flywheel.SLAVE_MOTOR.set(ControlMode.Follower, Flywheel.MASTER_MOTOR.getDeviceID());
    Flywheel.SLAVE_MOTOR.setInverted(false);

    Hood.config.initializeTalonPID(Hood.MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, true, false);
    Turret.config.initializeTalonPID(Turret.MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, true, true);

    // Reset encoders to 0 on initialisation
    this.resetEncoder(Flywheel.MASTER_MOTOR);
    this.resetEncoder(Hood.MOTOR);
    
    // Initialize Turret to 0
    this.moveTurretPID(Constants.TURRET_FRONT_LIMIT_POSITION);

     // Prints debug statements on SmartDashboard
     if (Constants.SHOOTER_DEBUG) {
      ShuffleboardTab tab = Shuffleboard.getTab(this.SUBSYSTEM_NAME);
      tab.addNumber("Flywheel Motor Output", () -> Flywheel.MASTER_MOTOR.getMotorOutputPercent());
      tab.addNumber("Flywheel Motor Velocity", () -> Flywheel.MASTER_MOTOR.getSensorCollection().getIntegratedSensorVelocity());
      tab.addNumber("Flywheel Motor Setpoint", () -> Flywheel.speed);
      tab.addNumber("Flywheel Error", () -> flywheelError());
      tab.addBoolean("Flywheel at Speed?", () -> isFlywheelAtSpeed());

      tab.addNumber("Hood Motor Output", () -> Hood.MOTOR.getMotorOutputPercent());
      tab.addNumber("Hood Encoder Position", () -> Hood.MOTOR.getSelectedSensorPosition());
      tab.addNumber("Hood Error", () -> Hood.MOTOR.getClosedLoopError());
      
      tab.addNumber("Turret Motor Output", () -> Turret.MOTOR.getMotorOutputPercent());
      tab.addNumber("Turret Encoder Position", () -> Turret.MOTOR.getSelectedSensorPosition());
      tab.addNumber("Turret Encoder Setpoint", () -> Turret.MOTOR.getClosedLoopTarget());
      tab.addNumber("Turret Error", () -> Turret.MOTOR.getClosedLoopError());
    }
  }
  
  /**
   * Reset the Turret and Hood setpoints
   */
  public void reset() {
    this.moveTurretPID(Turret.leftPosition);
    this.moveHoodPID(Hood.bottomPosition);
  }

  /**
   * Moves hood to position
   * @param setpoint input position to move to (in ticks)
   */
  public void moveHoodPID(double setpoint) {
    // Normalise setpoint
    if (setpoint < Hood.topPosition) setpoint = Hood.topPosition;
    else if (setpoint > Hood.bottomPosition) setpoint = Hood.bottomPosition;

    // Move hood toward setpoint
    Hood.MOTOR.set(ControlMode.MotionMagic, setpoint);
  }

  /**
   * Toggles Hood between top and bottom positions
   */
  public void toggleHoodPosition() {
    if (Hood.MOTOR.getClosedLoopTarget() == Constants.HOOD_TOP_POSITION) {
      moveHoodPID(Constants.HOOD_BOTTOM_POSITION);
    } else if (Hood.MOTOR.getClosedLoopTarget() == Constants.HOOD_BOTTOM_POSITION) {
      moveHoodPID(Constants.HOOD_TOP_POSITION);
    }
  }

  /**
   * Moves turret to position
   * @param setpoint inputs position to move to (in ticks)
   */
  public void moveTurretPID(double setpoint) {
    // Normalise setpoint
    if (setpoint < Turret.leftPosition) setpoint = Turret.leftPosition;
    else if (setpoint > Turret.rightPosition) setpoint = Turret.rightPosition;

    // Move turret toward setpoint
    Turret.MOTOR.set(ControlMode.MotionMagic, setpoint);
  }

  /**
   * Automatically aim at vision target
   */
  public void turretVisionPID() {
    if (VisionData.isReady() && VisionData.isDetected()) {
      double ticks = convertTurretDegreesToTicks(Constants.TURRET_TURN_DAMPER * VisionData.getXAngle());
      relativeMoveTurretPID(ticks);
    }
  }

  /**
   * Adds angle to current setpoint
   * @param setpoint inputs position to move to (in ticks)
   */
  public void relativeMoveTurretPID(double setpoint) {
    // Normalise setpoint
    if (Turret.MOTOR.getSelectedSensorPosition() + setpoint > Turret.config.getUpperLimit()) setpoint = 0;
    else if (Turret.MOTOR.getSelectedSensorPosition() + setpoint < Turret.config.getLowerLimit()) setpoint = 0;

    // Move turret toward setpoint
    this.moveTurretPID(Turret.MOTOR.getClosedLoopTarget() + setpoint);
  }

  /**
   * Checks if turret PID motion is complete
   * @return True if motion is completed else false
   */
  public boolean turretCheckIfMotionComplete() {
    return Math.abs(Turret.MOTOR.getSelectedSensorPosition() - Turret.MOTOR.getClosedLoopTarget()) < Turret.config.getTolerance();
  }

  /**
   * Moves flywheel to a speed
   * @param speed input speed to keep the motor at (ticks per 100 ms)
   */
  public void setFlywheelSpeed(double speed) {
    Flywheel.speed = speed;
    Flywheel.MASTER_MOTOR.set(ControlMode.Velocity, speed, DemandType.ArbitraryFeedForward, Flywheel.kF);
  }

  // TODO: Delete this once moveMotorManual works
  public void flywheelManual(double speed) {
    Flywheel.MASTER_MOTOR.set(speed);
    //Flywheel.SLAVE_MOTOR.set(-speed);
  }

  /**
   * Checks if flywheel is at set speed
   * @return True if flywheel is at speed else false
   */
  public boolean isFlywheelAtSpeed() {
    boolean atSpeed = (Math.abs(flywheelError()) > Flywheel.masterConfig.getTolerance());
    return atSpeed;
  }

  public double flywheelError() {
    return Flywheel.speed - Flywheel.MASTER_MOTOR.getSensorCollection().getIntegratedSensorVelocity();
  }

    // TODO: Delete this once moveMotorManual works
  public void hoodManual(double speed) {
    Hood.MOTOR.set(speed);
  }

  /**
   * Converts RPM to ticks per 100ms
   * @param RPM
   * @return ticks per 100ms
   */
  public double convertRPMToTicks(double RPM) {
    return Flywheel.TICKS_PER_ROTATION * RPM / (Flywheel.GEAR_RATIO * 600);
  }

  /**
   * Converts ticks per 100ms to RPM
   * @param ticks
   * @return RPM
   */
  public double convertTicksToRPM(double ticks) {
    return (ticks * (Flywheel.GEAR_RATIO * 600)) / Flywheel.TICKS_PER_ROTATION;
  }

  /**
   * Converts degrees to ticks for Turret
   * @param degrees input degrees to convert
   * @return position in ticks
   */
  private double convertTurretDegreesToTicks(double degrees) {
    return (Turret.TICKS_PER_DEGREE * degrees);
  }

  /**
   * @return converts ticks to degrees on the turret
   */
  private double convertTurretTicksToDegrees(double ticks) {
    return (ticks / Turret.TICKS_PER_DEGREE);
  }

  /**
   * @return current angle of the turret relative to the robot
   */
  public double getAngle() {
    return convertTurretTicksToDegrees(Turret.MOTOR.getSelectedSensorPosition());
  }
  
  /**
   * Moves subsystem motors manually
   * TODO: Test if this works, then delete other manual methods
   * @param motor Input motor
   * @param value Percentage power to motor [-1.0, 1.0]
   */
  public void moveMotorManual(BaseTalon motor, double value) {
    motor.set(ControlMode.PercentOutput, value);
  }

  /**
   * Reset encoder to 0 to keep it in sync
   * @param motor resets input encoder
   */
  public void resetEncoder(BaseTalon motor) {
    motor.setSelectedSensorPosition(0);
  }

  /**
   * @return if the front limit switch is pressed
   */
  public boolean turretLimitFront() {
    return Turret.MOTOR.getSensorCollection().isRevLimitSwitchClosed(); // TODO: Figure out if this is right
  }

  /**
   * @return if the back limit switch is pressed
   */
  public boolean turretLimitBack() {
    return Turret.MOTOR.getSensorCollection().isFwdLimitSwitchClosed(); // TODO: Figure out if this is right
  }

  /**
   * @return if the back limit switch is pressed
   */
  public boolean hoodLimit() {
    return Hood.MOTOR.getSensorCollection().isFwdLimitSwitchClosed(); // TODO: Figure out if this is right
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Automatically move turret to vision target
    if (RobotContainer.doVision) turretVisionPID();

    // Reset the turret encoder to the front position
    if (this.turretLimitFront() && Turret.needsReset) {
      Turret.needsReset = false;
      Turret.MOTOR.setSelectedSensorPosition(Constants.TURRET_FRONT_LIMIT_POSITION);
    } else if (!this.turretLimitFront() && !this.turretLimitBack()) Turret.needsReset = true;

    // Reset the turret encoder to the back position
    if (this.turretLimitBack() && Turret.needsReset) {
      Turret.needsReset = false;
      Turret.MOTOR.setSelectedSensorPosition(Constants.TURRET_BACK_LIMIT_POSITION);
    } else if (!this.turretLimitBack() && !this.turretLimitFront()) Turret.needsReset = true;

    // Reset the hood encoder to the back position
    if (this.hoodLimit() && Hood.needsReset) {
      Hood.needsReset = false;
      Hood.MOTOR.setSelectedSensorPosition(Constants.HOOD_BOTTOM_POSITION);
    } else if (!this.hoodLimit()) Hood.needsReset = true;
  }
}
