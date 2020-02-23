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
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonPIDConfig;

public class MagazineSubsystem extends SubsystemBase {

  // Declaration of motor
  private final WPI_TalonSRX MAGAZINE_MOTOR = new WPI_TalonSRX(Constants.MAGAZINE_MOTOR_PORT);
  private final CANSparkMax INTAKE_MOTOR = new CANSparkMax(Constants.INTAKE_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
  private final WPI_TalonSRX ARM_MOTOR = new WPI_TalonSRX(Constants.ARM_MOTOR_PORT);

  private final AnalogInput MAGAZINE_SENSOR_BOT = new AnalogInput(Constants.MAGAZINE_ULTRASONIC_BOT);
  private final AnalogInput MAGAZINE_SENSOR_TOP = new AnalogInput(Constants.MAGAZINE_ULTRASONIC_TOP);

  private final double TICKS_PER_ROTATION = 4096;
  private final double GEAR_RATIO = 2;
  private final double TICKS_PER_DEGREE = (this.TICKS_PER_DEGREE * this. GEAR_RATIO) / 360;
  private TalonPIDConfig config;

  private final double DISTANCE_PER_VOLT = 1.0; //TODO: Find value
  private final double ULTRASONIC_THRESHOLD = 5; //TODO: Find value

  /**
   * Creates a new MagazineSubsystem.
   */
  public MagazineSubsystem(TalonPIDConfig config) {
    this.config = config;
    this.config.initializeTalonPID(ARM_MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, true, false);
  }

  /**
   * Run uptake when ball is detected at the bottom & not at the top
   */
  public void ballUptake() {
    if (ballDetectedBottom() && !ballDetectedTop()) {
      this.ballUptake(Constants.MAGAZINE_MOTOR_SPEED);
    }
  }
 
  /**
   * Makes the motor for the magazine to spin
   * @param speed (double) how fast you want the motor to spin [-1, 1]
   */
  public void ballUptake(double speed) {
    MAGAZINE_MOTOR.set(speed);
  }

  /**
   * Stops magazine motor
   */
  public void magazineMotorStop() {
    MAGAZINE_MOTOR.set(0);
  }

  /**
   * Moves arm to set position
   * @param setpoint Position where arm goes (ticks)
   */
  public void armSetPosition(double setpoint) {
    double feedForward = this.config.getkF() * Math.cos(Math.toRadians(encoderPositionToDegrees(ARM_MOTOR.getSelectedSensorPosition())));
    ARM_MOTOR.set(ControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, feedForward);
  }

  /**
   * Moves arm relative to current setpoint
   * @param setpoint increments to move arm by
   */
  public void armPositionRelative(double setpoint) {
    this.armSetPosition(ARM_MOTOR.getClosedLoopTarget() + setpoint);
  }

  /**
   * Moves arm at set speed
   * @param speed (double) how fast you want the motor to spin [-1, 1]
   */
  public void armManual(double speed) {
    ARM_MOTOR.set(speed);
  }

  /**
   * Toggles arm between top and bottom positions
   */
  public void toggleArmPosition() {
    if (ARM_MOTOR.getClosedLoopTarget() == Constants.ARM_TOP_POSITION) {
      armSetPosition(Constants.ARM_BOTTOM_POSITION);
    } else if (ARM_MOTOR.getClosedLoopTarget() == Constants.ARM_BOTTOM_POSITION) {
      armSetPosition(Constants.ARM_TOP_POSITION);
    }
  }

  /**
   * Converts position (ticks) to degrees
   * @param position measured in ticks
   * @return degrees
   */
  public double encoderPositionToDegrees(double position) {
    return (position - Constants.ARM_BOTTOM_POSITION) / this.TICKS_PER_DEGREE;
  }

  /**
   * Makes the intake motor spin
   * @param speed (double) how fast you want the motor to spin [-1, 1]
   */
  public void intakeMotorSpeed(double speed) {
    INTAKE_MOTOR.set(speed);
  }

  /**
   * Stops the motor
   */
  public void intakeMotorStop() {
    INTAKE_MOTOR.set(0);
  }

  /**
   * @return bottom ultrasonic distance
   */
  private double magazineSensorBottom() {
    return voltageToDistance(MAGAZINE_SENSOR_BOT.getVoltage());
  }

  /**
   * @return top ultrasonic distance
   */
  private double magazineSensorTop() {
    return voltageToDistance(MAGAZINE_SENSOR_TOP.getVoltage());
  }

  /**
   * @return if a ball is detected at the bottom position
   */
  public boolean ballDetectedBottom() {
    return magazineSensorBottom() < ULTRASONIC_THRESHOLD;
  }

  /**
   * @return if a ball is detected at the bottom position
   */
  public boolean ballDetectedTop() {
    return magazineSensorTop() < ULTRASONIC_THRESHOLD;
  }

  /**
   * Reset encoder to 0 to keep it in sync
   * @param motor resets input encoder
   */
  public void resetEncoder(WPI_TalonSRX motor) {
    motor.setSelectedSensorPosition(0);
  }

  /**
   * @return True if the limit switch is pressed else false
   */
  public boolean armLimit() {
    return ARM_MOTOR.getSensorCollection().isFwdLimitSwitchClosed(); // TODO: Figure out if this is right
  }

  /**
   * Gets arm motor
   * @return WPI_TalonSRX arm motor
   */
  public WPI_TalonSRX getArmMotor() {
    return ARM_MOTOR;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (ARM_MOTOR.getSensorCollection().isFwdLimitSwitchClosed()) ARM_MOTOR.setSelectedSensorPosition(0);

    if (Constants.MAGAZINE_DEBUG) {
      SmartDashboard.putNumber("Arm Motor Output", ARM_MOTOR.getMotorOutputPercent());
      SmartDashboard.putNumber("Arm Motor Position", ARM_MOTOR.getSelectedSensorPosition());
      SmartDashboard.putNumber("Arm Motor Setpoint", ARM_MOTOR.getClosedLoopTarget());
      SmartDashboard.putNumber("Intake speed", INTAKE_MOTOR.get());
      SmartDashboard.putNumber("Top sensor", this.magazineSensorTop());
      SmartDashboard.putNumber("Bottom sensor", this.magazineSensorBottom());
    }
  }

  /**
   * converts raw voltage to distance from ultrasonic sensor
   * @param voltage input ultrasonic sensor voltage
   * @return distance in inches
   */
  private double voltageToDistance(double voltage) {
    return voltage * DISTANCE_PER_VOLT;
  }
}

