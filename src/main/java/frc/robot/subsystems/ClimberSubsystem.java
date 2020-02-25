/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;

public class ClimberSubsystem extends PIDSubsystem {

  private final String SUBSYSTEM_NAME = "Climber Subsystem";
  private boolean hasToRun = true;
  
  // Declaration of motors
  private final WPI_TalonSRX CLIMBER_LIFT_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_LIFT_MOTOR_PORT);
  private final WPI_TalonSRX CLIMBER_HOOK_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_HOOK_MOTOR_PORT);
  private final WPI_TalonSRX CLIMBER_BALANCE_MOTOR = new WPI_TalonSRX(Constants.CLIMBER_BALANCE_MOTOR_PORT);
  //private final I2C gyro = new I2C(I2C.Port.kOnboard, Constants.CLIMBER_GYRO_PORT);
  private final double GYRO_SENSITIVITY = 0.00875;
  private final int ENABLE_GYRO_REG = 0x10;
  private final int ENABLE_GYRO_VALUE = 0x20;
  private final int GYRO_ADDRESS = 0x1C;
  private long previousTime = 0;
  private double zAxisVelocity = 0;
  private double zAxisPosition = 0;
  private byte[] zAxisVelocityBytes = new byte[2];


  public ClimberSubsystem(double kP, double kD) {
    super(new PIDController(kP, 0, kD));

    //gyro.write(ENABLE_GYRO_REG, ENABLE_GYRO_VALUE);

    this.setSetpoint(0);

    if (Constants.CLIMBER_DEBUG) {
      ShuffleboardTab tab = Shuffleboard.getTab(this.SUBSYSTEM_NAME);
      tab.addNumber("Hook Percent Output", () -> CLIMBER_HOOK_MOTOR.getMotorOutputPercent());
      tab.addNumber("Lift Percent Output", () -> CLIMBER_LIFT_MOTOR.getMotorOutputPercent());
      tab.addNumber("Mouse Droid position", () -> CLIMBER_BALANCE_MOTOR.getSensorCollection().getQuadraturePosition());
      tab.addNumber("Mouse Droid Angle", () -> this.zAxisPosition);
    }
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    CLIMBER_BALANCE_MOTOR.set(output);
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here //TODO: rip gyro
    /*
    if (!gyro.read(this.GYRO_ADDRESS, 2, this.zAxisVelocityBytes)) {
      long currentTime = System.currentTimeMillis();
      zAxisVelocity = this.zAxisVelocityBytes[1] << 8 | this.zAxisVelocityBytes[0];
      zAxisVelocity *= this.GYRO_SENSITIVITY;
      if (Math.abs(zAxisVelocity) < 0.25) zAxisVelocity = 0;
      System.out.println(zAxisVelocity);
      long timeDelta = (this.previousTime == 0) ? 0 : currentTime - this.previousTime;
      this.previousTime = currentTime;
      return this.zAxisPosition += zAxisVelocity * (timeDelta / 1000);
    } else return zAxisPosition;
    */
    return 0;
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
   * Moves the climber hook
   * @param hookspeed speed at which the hook moves at [-1, 1]
   */
  public void hookManual(double hookspeed) {
    CLIMBER_HOOK_MOTOR.set(hookspeed);
  }

  /**
   * Moves the climber mouse droid
   * @param speed speed at which the mouse droid moves at [-1, 1]
   */
  public void mouseDroidManual(double speed) {
    CLIMBER_BALANCE_MOTOR.set(speed);
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
  }
}
