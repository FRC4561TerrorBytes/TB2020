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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonPIDConfig;

public class ShooterSubsystem extends SubsystemBase {
  private static class Flywheel {
    private static double kF;
    private static final int TICKS_PER_ROTATION = 2048;
    // public static final double GEAR_RATIO = 3 / 4; The real thing
    public static final double GEAR_RATIO = 1.0;
    private static final WPI_TalonFX MASTER_MOTOR = new WPI_TalonFX(Constants.FLYWHEEL_MASTER_MOTOR_PORT);
    private static final WPI_TalonFX SLAVE_MOTOR = new WPI_TalonFX(Constants.FLYWHEEL_SLAVE_MOTOR_PORT);
    private static TalonPIDConfig masterConfig;
    private static TalonPIDConfig slaveConfig;
    private static double speed = 0;
  }

  private static class Hood {
    private static int topPosition;
    private static double bottomPosition = 0;
    private static final int TICKS_PER_ROTATION = 4096;
    public static final double GEAR_RATIO = 298 / 25;
    private static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(Constants.HOOD_MOTOR_PORT);
    private static TalonPIDConfig config;
  }

  private static class Turret {
    private static int leftPosition;
    private static int middlePosition;
    private static int rightPosition = 0;
    private static final int TICKS_PER_ROTATION = 4096;
    public static final int GEAR_RATIO = 94 / 15;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_ROTATION * GEAR_RATIO) / 360;
    private static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(Constants.TURRET_MOTOR_PORT);
    private static TalonPIDConfig config;

  }

  // Creates new NetworkTable object
  public static NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard");

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
  public ShooterSubsystem(TalonPIDConfig flywheelMasterConfig, TalonPIDConfig flywheelSlaveConfig, TalonPIDConfig hoodConfig, TalonPIDConfig turretConfig) {

    Flywheel.masterConfig = flywheelMasterConfig;
    Flywheel.slaveConfig = flywheelSlaveConfig;
    Hood.config = hoodConfig;
    Turret.config = turretConfig;

    

    Flywheel.masterConfig.initializeTalonPID(Flywheel.MASTER_MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, false, false);
    Flywheel.slaveConfig.initializeTalonPID(Flywheel.SLAVE_MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, false, false);
    Hood.config.initializeTalonPID(Hood.MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, true, false);
    Turret.config.initializeTalonPID(Turret.MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, true, true);

    // Reset encoders to 0 on initialisation
    this.resetEncoder(Flywheel.MASTER_MOTOR);
    this.resetEncoder(Hood.MOTOR);
    
    // TODO: Comment this
    Turret.middlePosition = Turret.MOTOR.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		Turret.middlePosition &= 0xFFF;
		if (Turret.config.getSensorPhase())  Turret.middlePosition *= -1;
		if (Turret.config.getInvertMotor()) Turret.middlePosition *= -1;
		
		/* Set the quadrature (relative) sensor to match absolute */
		Turret.MOTOR.setSelectedSensorPosition(Turret.middlePosition);

  }

  /**
   * Moves hood to position
   * @param setpoint input position to move to (in ticks)
   */
  public void moveHoodPID(double setpoint) {
    // Normalise setpoint
    if (setpoint > Hood.topPosition) setpoint = Hood.topPosition;
    else if (setpoint < Hood.bottomPosition) setpoint = Hood.bottomPosition;

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
   * @param setpoint inputs position to move to (in degrees)
   */
  public void moveTurretPID(double setpoint) {
    // Normalise setpoint
    if (setpoint > Turret.leftPosition) setpoint = Turret.leftPosition;
    else if (setpoint < Turret.rightPosition) setpoint = Turret.rightPosition;

    // Move turret toward setpoint
    Turret.MOTOR.set(ControlMode.MotionMagic, convertTurretDegreesToTicks(setpoint) + Turret.middlePosition);
  }

  /**
   * Automatically aim at vision target
   */
  public void turretVisionPID() {
    if (table.getEntry("detected?").getBoolean(false)) {
      relativeMoveTurretPID(convertTurretTicksToDegrees(table.getEntry("xAngle").getDouble(0.0)));
    }
  }

  /**
   * Is the vision target detected
   * @return True if target is detected else false
   */
  public boolean isDetected() {
    return table.getEntry("detected?").getBoolean(false);
  }

  /**
   * Adds angle to current setpoint
   * @param setpoint inputs position to move to (in degrees)
   */
  public void relativeMoveTurretPID(double setpoint) {
    // Normalise setpoint
    if (Turret.MOTOR.getSelectedSensorPosition() + convertTurretDegreesToTicks(setpoint) > Turret.config.getUpperLimit()) setpoint = 0;
    else if (Turret.MOTOR.getSelectedSensorPosition() + convertTurretDegreesToTicks(setpoint) < Turret.config.getUpperLimit()) setpoint = 0;

    // Move turret toward setpoint
    Turret.MOTOR.set(ControlMode.MotionMagic, Turret.MOTOR.getClosedLoopTarget() + convertTurretDegreesToTicks(setpoint));
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
    Flywheel.MASTER_MOTOR.set(ControlMode.Velocity, -speed, DemandType.ArbitraryFeedForward, Flywheel.kF);
    Flywheel.SLAVE_MOTOR.set(ControlMode.Velocity, speed, DemandType.ArbitraryFeedForward, Flywheel.kF);
  }

  // TODO: Delete this once moveMotorManual works
  public void flywheelManual(double speed) {
    //Flywheel.MASTER_MOTOR.set(speed);
    Flywheel.SLAVE_MOTOR.set(speed);
  }

  /**
   * Checks if flywheel is at set speed
   * @return True if flywheel is at speed else false
   */
  public boolean isFlywheelAtSpeed() {
    return (Math.abs(Flywheel.MASTER_MOTOR.getSelectedSensorVelocity() - Flywheel.speed) < Flywheel.masterConfig.getTolerance() &&
            Math.abs(Flywheel.SLAVE_MOTOR.getSelectedSensorVelocity() - Flywheel.speed) < Flywheel.slaveConfig.getTolerance());
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
   * Gets flywheel motor
   * @return WPI_TalonFX flywheel motor
   */
  public WPI_TalonFX getFlywheelMotor() {
    return Flywheel.MASTER_MOTOR;
  }

  /**
   * Gets hood motor
   * @return WPI_TalonSRX hood motor
   */
  public WPI_TalonSRX getHoodMotor() {
    return Hood.MOTOR;
  }

  /**
   * Gets turret motor
   * @return WPI_TalonSRX turret motor
   */
  public  WPI_TalonSRX getTurretMotor() {
    return Turret.MOTOR;
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
    return Turret.MOTOR.getSensorCollection().isFwdLimitSwitchClosed(); // TODO: Figure out if this is right
  }

  /**
   * @return if the back limit switch is pressed
   */
  public boolean turretLimitBack() {
    return Turret.MOTOR.getSensorCollection().isRevLimitSwitchClosed(); // TODO: Figure out if this is right
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
    turretVisionPID();

    // Reset the turret encoder to the front position
    if (this.turretLimitFront()) {
      this.getTurretMotor().setSelectedSensorPosition(Constants.TURRET_FRONT_LIMIT_POSITION);
    }
    
    // Reset the turret encoder to the back position
    if (this.turretLimitBack()) {
       this.getTurretMotor().setSelectedSensorPosition(Constants.TURRET_BACK_LIMIT_POSITION);
    }

    // Reset the hood encoder to the back position
    if (this.hoodLimit()) {
      this.getHoodMotor().setSelectedSensorPosition(Constants.HOOD_BOTTOM_POSITION);
    }

    // Prints debug statements on SmartDashboard
    if (Constants.SHOOTER_DEBUG) {
      SmartDashboard.putNumber("Flywheel Motor Output", Flywheel.MASTER_MOTOR.getMotorOutputPercent());
      SmartDashboard.putNumber("Flywheel Motor Velocity", Flywheel.MASTER_MOTOR.getSensorCollection().getIntegratedSensorVelocity());
      SmartDashboard.putNumber("Flywheel Error", Flywheel.MASTER_MOTOR.getClosedLoopError());

      SmartDashboard.putNumber("Hood Motor Output", Hood.MOTOR.getMotorOutputPercent());
      SmartDashboard.putNumber("Hood Encoder Position", Hood.MOTOR.getSensorCollection().getQuadraturePosition());
      SmartDashboard.putNumber("Hood Error", Hood.MOTOR.getClosedLoopError());
      
      SmartDashboard.putNumber("Turret Motor Output", Turret.MOTOR.getMotorOutputPercent());
      SmartDashboard.putNumber("Turret Encoder Position", convertTicksToRPM(Turret.MOTOR.getSensorCollection().getQuadraturePosition()));
      SmartDashboard.putNumber("Turret Error", Turret.MOTOR.getClosedLoopError());
    }
  }
}
