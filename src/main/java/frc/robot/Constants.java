/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    // Drive PID values
    public static final double DRIVE_kP = 0.010;
    public static final double DRIVE_kD = 0.0005;
    public static final double DRIVE_PERIOD_SECONDS = 0.01667; // 60Hz
    public static final double DRIVE_TURN_SCALAR = 10.0;
    public static final double DRIVE_TOLERANCE = 1.0;

    // Intake Arm PID config
    public static final double ARM_kP = 0.0;
    public static final double ARM_kD = 0.0;
    public static final double ARM_kF = 0.0;
    public static final double ARM_TOLERANCE = 0.0;
    public static final double ARM_LOWER_LIMIT = 0.0;
    public static final double ARM_UPPER_LIMIT = 0.0;
    public static final double ARM_VELOCITY_RPM = 0.0;
    public static final double ARM_ACCELERATION_RPM_PER_SEC = 0.0;
    public static final int ARM_MOTION_SMOOTHING = 0;
    public static final boolean ARM_SENSOR_PHASE = false;
    public static final boolean ARM_INVERT_MOTOR = false;

    // Intake Arm Positions
    //TODO: Set these
    public static final double ARM_TOP_POSITION = 0;
    public static final double ARM_BOTTOM_POSITION = 1000;

    // Arm Config
    public static final TalonPIDConfig ARM_CONFIG = new TalonPIDConfig(ARM_SENSOR_PHASE,
                                                                        ARM_INVERT_MOTOR,
                                                                        ARM_kP,
                                                                        0.0,
                                                                        ARM_kD,
                                                                        ARM_kF,
                                                                        ARM_TOLERANCE,
                                                                        ARM_LOWER_LIMIT,
                                                                        ARM_UPPER_LIMIT,
                                                                        ARM_VELOCITY_RPM,
                                                                        ARM_ACCELERATION_RPM_PER_SEC,
                                                                        ARM_MOTION_SMOOTHING);

    // Analog stick deadband value
    public static final double DEADBAND = 0.005;
    
    // Joystick Ports
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final int LEFT_JOYSTICK_PORT = 0;

    // Xbox controller
    public static final int XBOX_CONTROLLER_PORT = 2;    

    // Drive Motor Ports 
    public static final int FRONT_LEFT_MOTOR_PORT = 0;
    public static final int REAR_LEFT_MOTOR_PORT = 1;

    public static final int FRONT_RIGHT_MOTOR_PORT = 2;
    public static final int REAR_RIGHT_MOTOR_PORT = 3;

    // Climber Motor Ports
    public static final int CLIMBER_LIFT_MOTOR_PORT = 12;
    public static final int CLIMBER_HOOK_MOTOR_PORT = 11;
    public static final int CLIMBER_BALANCE_MOTOR_PORT = 10;

    // Climber Movement Constants
    public static final double CLIMBER_LIFT_CONSTANT = 0.5;
    public static final double CLIMBER_HOOK_CONSTANT = 0.69;

    //Magazine Motor Port
    public static final int MAGAZINE_MOTOR_PORT = 13;

    //Intake Motor Port
    public static final int INTAKE_MOTOR_PORT = 14;
    public static final int ARM_MOTOR_PORT = 15;
      
    //set Magazine Motor Speed
    public static final double MagazineMotorSpeed = 0.5;
}
