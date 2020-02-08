/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ClimberSubsystem extends PIDSubsystem {
  
  // Declaration of motors
  private final WPI_TalonSRX CLIMBER_LIFT_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_LIFT_MOTOR_PORT);
  private final WPI_TalonSRX CLIMBER_HOOK_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_HOOK_MOTOR_PORT);
  private final WPI_TalonSRX CLIMBER_BALANCE_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_BALANCE_MOTOR_PORT);
  private final I2C gyro = new I2C(I2C.Port.kOnboard, Constants.CLIMBER_GYRO_PORT);
  private double levelTarget = 0.0;
  private static int gyroAddress = 0x1c;
  private static double zAxisPosition = 0;


  public ClimberSubsystem(double kP, double kD) {
    
    super(
        // The PIDController used by the subsystem
        new PIDController(kP, 0, kD));

    byte[] zAxis = new byte[2];
    if(!gyro.read(0x2C, 2, zAxis)) {
        this.levelTarget = zAxis[1] * 256 + zAxis[0];
        setSetpoint(this.levelTarget);
    }

  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    CLIMBER_BALANCE_MOTOR.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    byte[] zAxisVelocityBytes = new byte[2];
    double zAxisVelocity = 0;
    if (!gyro.read(gyroAddress, 2, zAxisVelocityBytes)) {
      zAxisVelocity = zAxisVelocityBytes[1] << 8 | zAxisVelocityBytes[0];
      return this.zAxisPosition += zAxisVelocity / 60 * Constants.ROBOT_TICK_RATE;
    } else return zAxisPosition;
  }

  /**
   * Resets mousedroid gyro
   */
  public void resetGyro() {
    this.zAxisPosition = 0;
  }
  
  /**
   * Move climber lift
   * @param liftSpeed speed at which motor lifts at [-1, 1]
   */
  public void liftManual(double liftSpeed) {
    CLIMBER_LIFT_MOTOR.set(liftSpeed);
  }

  /**
   * Moves the climber hook with controller button
   * @param hookspeed speed at which the hook moves at [-1, 1]
   */
  public void hookManual(double hookspeed) {
    CLIMBER_HOOK_MOTOR.set(hookspeed);
  }

  /**
   * Stops the lift motor
   */
  public void stopLift() {
    CLIMBER_LIFT_MOTOR.set(0);
  }

  /**
   * Stops the balance motor
   */
  public void stopBalance() {
    CLIMBER_BALANCE_MOTOR.set(0);
  }

  /**
   * Stops the hook motor
   */
  public void stopHook() {
    CLIMBER_HOOK_MOTOR.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if(Constants.CLIMBER_SUBSYSTEM_DEBUG) {
      SmartDashboard.putNumber("Mouse Droid Angle", zAxisPosition);
    }
  }
}
