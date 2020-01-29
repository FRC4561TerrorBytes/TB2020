/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ClimberSubsystem extends PIDSubsystem {
  
  // Declaration of motors
  private final WPI_TalonSRX CLIMBER_LIFT_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_LIFT_MOTOR_PORT);
  private final WPI_TalonSRX CLIMBER_HOOK_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_HOOK_MOTOR_PORT);
  private final WPI_TalonSRX CLIMBER_BALANCE_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_BALANCE_MOTOR_PORT);

  public ClimberSubsystem() {
    
    super(
        // The PIDController used by the subsystem
        new PIDController(0, 0, 0));
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
  
  // Move climber lift with controller button
  public void liftManual(double liftSpeed) {
    CLIMBER_LIFT_MOTOR.set(liftSpeed);
  }
  // Moves the climber hook with controller button
  public void hookManual(double hookspeed) {
    CLIMBER_HOOK_MOTOR.set(hookspeed);
  }

  // Stops the lift motor
  public void stopLift() {
    CLIMBER_LIFT_MOTOR.set(0);
  }
  // Stops the balance motor
  public void stopBalance() {
    CLIMBER_BALANCE_MOTOR.set(0);
  }
  // Stops the hook motor
  public void stopHook() {
    CLIMBER_HOOK_MOTOR.set(0);
  }
}