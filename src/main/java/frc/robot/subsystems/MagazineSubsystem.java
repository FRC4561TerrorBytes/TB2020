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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.TalonPIDConfig;

public class MagazineSubsystem extends SubsystemBase {

  // Declaration of motor
  private final WPI_TalonSRX MAGAZINE_MOTOR = new WPI_TalonSRX(Constants.MAGAZINE_MOTOR_PORT);
  private final WPI_TalonSRX INTAKE_MOTOR = new WPI_TalonSRX(Constants.INTAKE_MOTOR_PORT);
  private final WPI_TalonSRX ARM_MOTOR = new WPI_TalonSRX(Constants.ARM_MOTOR_PORT);

  private final double TICKS_PER_ROTATION = 4096;
  private final double GEAR_RATIO = 2;
  private final double TICKS_PER_DEGREE = (this.TICKS_PER_DEGREE * this. GEAR_RATIO) / 360;
  private TalonPIDConfig config;

  /**
   * Creates a new MagazineSubsystem.
   */
  public MagazineSubsystem(TalonPIDConfig config) {
    this.config = config;
    this.config.initializeTalonPID(ARM_MOTOR, FeedbackDevice.CTRE_MagEncoder_Relative, true, false);
  }
 
  /**
   * Makes the motor for the magazine to spin
   */
  public void magazineMotorSpeed() {
    MAGAZINE_MOTOR.set(Constants.MagazineMotorSpeed);
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
    ARM_MOTOR.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, feedForward);
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
   * Detects ball 
   * @return True if limit switch is hit else False
   */
  public boolean ballDetected() {
    boolean DetectorBall = this.MAGAZINE_MOTOR.getSensorCollection().isFwdLimitSwitchClosed();
    return DetectorBall;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

