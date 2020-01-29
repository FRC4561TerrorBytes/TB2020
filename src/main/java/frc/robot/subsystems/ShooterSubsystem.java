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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private static class Flywheel {
    private static double kF;
    private static double tolerance;
    private static final int TICKS_PER_ROTATION = 2048;
    public static final double GEAR_RATIO = 3 / 4;
    private static final WPI_TalonFX MASTER_MOTOR = new WPI_TalonFX(Constants.FLYWHEEL_MASTER_MOTOR_PORT);
    private static final WPI_TalonFX SLAVE_MOTOR = new WPI_TalonFX(Constants.FLYWHEEL_SLAVE_MOTOR_PORT);
    private static final int PID_SLOT = 0;
  }

  private static class Hood {
    private static double tolerance;
    private static int topPosition;
    private static double bottomPosition = 0;
    private static final int TICKS_PER_ROTATION = 4096;
    public static final double GEAR_RATIO = 298 / 25;
    private static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(Constants.HOOD_MOTOR_PORT);
    private static final int PID_SLOT = 0;
  }

  private static class Turret {
    private static double tolerance;
    private static int leftPosition;
    private static int middlePosition;
    private static int rightPosition = 0;
    private static final int TICKS_PER_ROTATION = 4096;
    public static final int GEAR_RATIO = 94 / 15;
    private static final double TICKS_PER_DEGREE = (TICKS_PER_ROTATION * GEAR_RATIO) / 360;
    private static final WPI_TalonSRX MOTOR = new WPI_TalonSRX(Constants.TURRET_MOTOR_PORT);
    private static final int PID_SLOT = 0;
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
   * @param flywheel_kP proportional gain for flywheel
   * @param flywheel_kD derivative gain for flywheel
   * @param flywheel_kF feed forward gain for flywheel
   * @param flywheel_tolerance tolerance in encoder ticks for flywheel
   * @param hood_kP proportional gain for hood
   * @param hood_kD derivative gain for hood
   * @param hood_tolerance tolerance in encoder ticks for hood
   * @param hood_topPosition topmost position in encoder ticks for hood
   * @param turret_kP proportional gain for turret
   * @param turret_kD derivative gain for turret
   * @param turret_tolerance tolerance in encoder ticks for turret
   * @param turret_leftPosition leftmost position in encoder ticks for turret
   */
  public ShooterSubsystem(double flywheel_kP, double flywheel_kD, double flywheel_kF,
                          double flywheel_tolerance, double hood_kP, double hood_kD,
                          double hood_tolerance, int hood_topPosition, double turret_kP,
                          double turret_kD, double turret_tolerance, int turret_leftPosition) {
    Flywheel.kF = flywheel_kF;
    Flywheel.tolerance = flywheel_tolerance;
    Hood.topPosition = hood_topPosition;
    Hood.tolerance = hood_tolerance;
    Turret.tolerance = turret_tolerance;
    Turret.middlePosition = turret_leftPosition;
    Turret.leftPosition = turret_leftPosition;

    // Make a falcon motor controller follow master
    Flywheel.SLAVE_MOTOR.set(ControlMode.Follower, Flywheel.MASTER_MOTOR.getDeviceID());


    /******************************************************
     * PID for Flywheel
     ******************************************************/
    
    // Reset arm Talon SRX to factory defaults to prevent unexpected behavior
    Flywheel.MASTER_MOTOR.configFactoryDefault();

    // Set encoder as feedback sensor and set sensor phase
    Flywheel.MASTER_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    Flywheel.MASTER_MOTOR.setSensorPhase(Constants.FLYWHEEL_ENCODER_SENSOR_PHASE);

    // Invert motor to stay in phase with encoder
    Flywheel.MASTER_MOTOR.setInverted(Constants.FLYWHEEL_MOTOR_INVERTED);

    // Set minimum and maxiumum motor output values
    Flywheel.MASTER_MOTOR.configClosedLoopPeakOutput(Flywheel.PID_SLOT, 1);

    // Set PID values in profile slot 0, PID slot 0.
    Flywheel.MASTER_MOTOR.config_kP(Flywheel.PID_SLOT, flywheel_kP);
    Flywheel.MASTER_MOTOR.config_kI(Flywheel.PID_SLOT, 0); // Do not use I
    Flywheel.MASTER_MOTOR.config_kD(Flywheel.PID_SLOT, flywheel_kD);
    Flywheel.MASTER_MOTOR.config_kF(Flywheel.PID_SLOT, flywheel_kF);
    Flywheel.MASTER_MOTOR.configAllowableClosedloopError(Flywheel.PID_SLOT, (int)Flywheel.tolerance);


    /******************************************************
     * PID for Hood
     ******************************************************/

    // Reset arm Talon SRX to factory defaults to prevent unexpected behavior
    Hood.MOTOR.configFactoryDefault();

    // Set encoder as feedback sensor and set sensor phase
    Hood.MOTOR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    Hood.MOTOR.setSensorPhase(Constants.HOOD_ENCODER_SENSOR_PHASE);

    // Invert motor to stay in phase with encoder
    Hood.MOTOR.setInverted(Constants.HOOD_MOTOR_INVERTED);

    // Configure and enable soft limits
    Hood.MOTOR.configForwardSoftLimitThreshold(Hood.topPosition);
    Hood.MOTOR.configForwardSoftLimitEnable(true);
    Hood.MOTOR.configReverseSoftLimitThreshold(0); // Bottom limit is 0
    Hood.MOTOR.configReverseSoftLimitEnable(true);

    // Set minimum and maxiumum motor output values
    Hood.MOTOR.configClosedLoopPeakOutput(Hood.PID_SLOT, 1);

    // Set PID values in profile slot 0, PID slot 0.
    Hood.MOTOR.selectProfileSlot(Hood.PID_SLOT, Hood.PID_SLOT);
    Hood.MOTOR.config_kP(Hood.PID_SLOT, hood_kP);
    Hood.MOTOR.config_kI(Hood.PID_SLOT, 0); // Do not use I
    Hood.MOTOR.config_kD(Hood.PID_SLOT, hood_kD);
    Hood.MOTOR.configAllowableClosedloopError(Hood.PID_SLOT, (int)Hood.tolerance);


    /******************************************************
     * PID for Turret
     ******************************************************/

    // Reset arm Talon SRX to factory defaults to prevent unexpected behavior
    Turret.MOTOR.configFactoryDefault();

    // Set encoder as feedback sensor and set sensor phase
    Turret.MOTOR.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    Turret.MOTOR.setSensorPhase(Constants.TURRET_ENCODER_SENSOR_PHASE);

    // Invert motor to stay in phase with encoder
    Turret.MOTOR.setInverted(Constants.TURRET_MOTOR_INVERTED);

    // Configure and enable soft limits
    Turret.MOTOR.configForwardSoftLimitThreshold(Turret.leftPosition);
    Turret.MOTOR.configForwardSoftLimitEnable(true);
    Turret.MOTOR.configReverseSoftLimitThreshold(0); // Bottom limit is 0
    Turret.MOTOR.configReverseSoftLimitEnable(true);

    // Set minimum and maxiumum motor output values
    Turret.MOTOR.configClosedLoopPeakOutput(Turret.PID_SLOT, 1);

    // Set PID values in profile slot 0, PID slot 0. F is calculated dynamically
    Turret.MOTOR.selectProfileSlot(Turret.PID_SLOT, Turret.PID_SLOT);
    Turret.MOTOR.config_kP(Turret.PID_SLOT, turret_kP);
    Turret.MOTOR.config_kI(Turret.PID_SLOT, 0); // Do not use I
    Turret.MOTOR.config_kD(Turret.PID_SLOT, turret_kD);
    Turret.MOTOR.configAllowableClosedloopError(Turret.PID_SLOT, (int)Turret.tolerance);



    // Reset encoders to 0 on initialisation
    this.resetEncoder(Flywheel.MASTER_MOTOR);
    this.resetEncoder(Hood.MOTOR);
    
    Turret.middlePosition = Turret.MOTOR.getSensorCollection().getPulseWidthPosition();

		/* Mask out overflows, keep bottom 12 bits */
		Turret.middlePosition &= 0xFFF;
		if (Constants.TURRET_ENCODER_SENSOR_PHASE)  Turret.middlePosition *= -1;
		if (Constants.TURRET_MOTOR_INVERTED) Turret.middlePosition *= -1;
		
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
    Hood.MOTOR.set(ControlMode.Position, setpoint);
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
    Turret.MOTOR.set(ControlMode.Position, convertTurretDegreesToTicks(setpoint));
  }

  /**
   * Moves flywheel to a speed
   * @param speed input RPM to keep the motor at
   */
  public void setFlywheelSpeed(double speed) {
    Flywheel.MASTER_MOTOR.set(ControlMode.Velocity, speed, DemandType.ArbitraryFeedForward, Flywheel.kF);
  }

  /**
   * Converts RPM to ticks per 100ms
   * @param RPM
   * @return ticks per 100ms
   */
  public double convertRPMtoTicks(double RPM) {
    return Flywheel.TICKS_PER_ROTATION * RPM / (Flywheel.GEAR_RATIO * 600);
  }

  /**
   * Converts degrees to ticks for Turret
   * @param degrees input degrees to convert
   * @return position in ticks
   */
  private double convertTurretDegreesToTicks(double degrees) {
    return (Turret.TICKS_PER_DEGREE * degrees) + Turret.middlePosition;
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
  private void resetEncoder(BaseTalon motor) {
    motor.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
