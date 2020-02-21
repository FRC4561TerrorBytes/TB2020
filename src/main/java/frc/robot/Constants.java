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
    
    // Robot tick rate in seconds
    public static final double ROBOT_TICK_RATE = 0.01667;

    // Dubug constants
    public static final boolean CLIMBER_SUBSYSTEM_DEBUG = true;

    // Drive PID values
    public static final double DRIVE_kP = 0.010;
    public static final double DRIVE_kD = 0.0005;
    public static final double DRIVE_PERIOD_SECONDS = 0.01667; // 60Hz
    public static final double DRIVE_TURN_SCALAR = 10.0;
    public static final double DRIVE_TOLERANCE = 1.0;

    // Mouse Droid PID values
    public static final double MOUSE_kP = 0.0;
    public static final double MOUSE_kD = 0.0;

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
    public static final int CLIMBER_LIFT_MOTOR_PORT = 6;
    public static final int CLIMBER_HOOK_MOTOR_PORT = 11;
    public static final int CLIMBER_BALANCE_MOTOR_PORT = 7;

    // Climber Movement Constants
    public static final double CLIMBER_LIFT_CONSTANT = 0.5;
    public static final double CLIMBER_HOOK_CONSTANT = 0.69;

    // Climber I2C port
    //public static final int CLIMBER_GYRO_PORT = 0x6B;

    // Speed limiting [0.0, 1.0]
    public static final double CLIMBER_SPEED_LIMIT = 1.0; 

}
