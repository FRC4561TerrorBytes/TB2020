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

    // Show Debug
    public static final boolean SHOOTER_DEBUG = true;
    
    // Drive PID values
    public static final double DRIVE_kP = 0.010;
    public static final double DRIVE_kD = 0.0005;
    public static final double DRIVE_PERIOD_SECONDS = 0.01667; // 60Hz
    public static final double DRIVE_TURN_SCALAR = 10.0;
    public static final double DRIVE_TOLERANCE = 1.0;

    // Shooter PID Values
    //TODO: Set these
    private static final double FLYWHEEL_kP = 0.3;
    private static final double FLYWHEEL_kD = 0.0;
    private static final double FLYWHEEL_kF = 0.01;
    private static final double FLYWHEEL_TOLERANCE = 0.0;
    private static final boolean FLYWHEEL_ENCODER_SENSOR_PHASE = true;
    private static final boolean FLYWHEEL_MOTOR_INVERTED = true;
    private static final double HOOD_kP = 0.0;
    private static final double HOOD_kD = 0.0;
    private static final double HOOD_TOLERANCE = 0.0;
    private static final double HOOD_VELOCITY_RPM = 0.0;
    private static final double HOOD_ACCELERATION_RPM_PER_SEC = 0.0;
    private static final int HOOD_MOTION_SMOOTHING = 0; // between [0, 7]
    private static final boolean HOOD_ENCODER_SENSOR_PHASE = true;
    private static final boolean HOOD_MOTOR_INVERTED = true;
    private static final double TURRET_kP = 0.0;
    private static final double TURRET_kD = 0.0;
    private static final double TURRET_TOLERANCE = 0.0;
    private static final double TURRET_VELOCITY_RPM = 0.0;
    private static final double TURRET_ACCELERATION_RPM_PER_SEC = 0.0;
    private static final int TURRET_MOTION_SMOOTHING = 0; // between [0, 7]
    private static final boolean TURRET_ENCODER_SENSOR_PHASE = true;
    private static final boolean TURRET_MOTOR_INVERTED = true;
    
    // Shooter Positions
    public static final int HOOD_BOTTOM_POSITION = 0;
    public static final int HOOD_TOP_POSITION = 1000; //TODO: set this
    public static final int TURRET_FRONT_POSITION = 0;
    public static final int TURRET_MIDDLE_POSITION = 9626; //TODO: set this
    public static final int TURRET_BACK_POSITION = 19251; //TODO: set this

    // Set PID for Flywheel
    public static final TalonPIDConfig FLYWHEEL_CONFIG = new TalonPIDConfig(FLYWHEEL_ENCODER_SENSOR_PHASE,
                                                        FLYWHEEL_MOTOR_INVERTED,
                                                        FLYWHEEL_kP,
                                                        0,
                                                        FLYWHEEL_kD,
                                                        FLYWHEEL_kF,
                                                        FLYWHEEL_TOLERANCE);
    // Set PID for Hood
    public static final TalonPIDConfig HOOD_CONFIG = new TalonPIDConfig(HOOD_ENCODER_SENSOR_PHASE,
                                                    HOOD_MOTOR_INVERTED,
                                                    HOOD_kP,
                                                    0,
                                                    HOOD_kD,
                                                    0,
                                                    HOOD_TOLERANCE,
                                                    HOOD_BOTTOM_POSITION,
                                                    HOOD_TOP_POSITION,
                                                    HOOD_VELOCITY_RPM,
                                                    HOOD_ACCELERATION_RPM_PER_SEC,
                                                    HOOD_MOTION_SMOOTHING);

    // Set PID for Turret
    public static final TalonPIDConfig TURRET_CONFIG = new TalonPIDConfig(TURRET_ENCODER_SENSOR_PHASE,
                                                        TURRET_MOTOR_INVERTED,
                                                        TURRET_kP,
                                                        0,
                                                        TURRET_kD,
                                                        0,
                                                        TURRET_TOLERANCE,
                                                        TURRET_FRONT_POSITION,
                                                        TURRET_BACK_POSITION,
                                                        TURRET_VELOCITY_RPM,
                                                        TURRET_ACCELERATION_RPM_PER_SEC,
                                                        TURRET_MOTION_SMOOTHING);

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

    // Shooter Motor Ports
    public static final int FLYWHEEL_MASTER_MOTOR_PORT = 4;
    public static final int FLYWHEEL_SLAVE_MOTOR_PORT = 5;
    public static final int HOOD_MOTOR_PORT = 6;
    public static final int TURRET_MOTOR_PORT = 7;


}
