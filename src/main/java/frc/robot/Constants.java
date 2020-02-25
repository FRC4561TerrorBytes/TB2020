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
    public static final boolean MAGAZINE_DEBUG = true;
    
    // Robot tick rate in seconds
    public static final double ROBOT_TICK_RATE = 0.01667;

    // Dubug constants
    public static final boolean CLIMBER_SUBSYSTEM_DEBUG = true;

    // Drive PID values
    public static final double DRIVE_kP = 0.015;
    public static final double DRIVE_kD = 0.001;
    public static final double DRIVE_PERIOD_SECONDS = 0.004166; // 240Hz
    public static final double DRIVE_TURN_SCALAR = 10.0;
    public static final double DRIVE_TOLERANCE = 1.0;

    // Intake Arm PID config
    private static final double ARM_kP = 1.2;
    private static final double ARM_kD = 0.000;
    private static final double ARM_kF = 0.000;
    private static final double ARM_TOLERANCE = 10.0;
    private static final double ARM_LOWER_LIMIT = -3500;
    private static final double ARM_UPPER_LIMIT = 0;
    private static final boolean ARM_SOFT_LIMITS = true;
    private static final boolean ARM_SENSOR_PHASE = true;
    private static final boolean ARM_INVERT_MOTOR = false;
    public static final double ARM_MANUAL_INCREMENT = 150;

    // Intake Arm Positions
    //TODO: Set these
    public static final int ARM_TOP_POSITION = -3080;
    public static final int ARM_BOTTOM_POSITION = 0;


    // Shooter PID Values
    //TODO: Set these
    private static final double FLYWHEEL_kP = 0.05;
    private static final double FLYWHEEL_kD = 0.0;
    private static final double FLYWHEEL_kF = 2;
    private static final double FLYWHEEL_TOLERANCE = 100;
    private static final boolean FLYWHEEL_MASTER_ENCODER_SENSOR_PHASE = true;
    private static final boolean FLYWHEEL_MASTER_MOTOR_INVERTED = true;

    private static final double HOOD_kP = 2.0;
    private static final double HOOD_kD = 0.01;
    private static final double HOOD_TOLERANCE = 1.0;
    private static final boolean HOOD_SOFT_LIMITS = false;
    private static final double HOOD_VELOCITY = 100;
    private static final double HOOD_ACCELERATION = 50;
    private static final int HOOD_MOTION_SMOOTHING = 1;
    private static final boolean HOOD_ENCODER_SENSOR_PHASE = false;
    private static final boolean HOOD_MOTOR_INVERTED = false;
    public static final int HOOD_BOTTOM_POSITION = 0;
    public static final int HOOD_TOP_POSITION = -4570;

    private static final double TURRET_kP = 0.5;
    private static final double TURRET_kD = 0.001;
    private static final double TURRET_TOLERANCE = 2;
    private static final boolean TURRET_SOFT_LIMITS = true;
    private static final double TURRET_VELOCITY = 150;
    private static final double TURRET_ACCELERATION = 30;
    private static final int TURRET_MOTION_SMOOTHING = 7; // between [0, 7]
    private static final boolean TURRET_ENCODER_SENSOR_PHASE = false;
    private static final boolean TURRET_MOTOR_INVERTED = false;
    
    // Shooter Positions
    public static final int TURRET_FRONT_LIMIT_POSITION = 0;
    public static final int TURRET_STRAIGHT_POSITION = 3300;
    public static final int TURRET_MIDDLE_POSITION = 9700; //TODO: set this
    public static final int TURRET_BACK_POSITION = 16100;
    public static final int TURRET_BACK_LIMIT_POSITION = 19700; //TODO: set this

    // Set PID for Flywheel
    public static final TalonPIDConfig FLYWHEEL_MASTER_CONFIG = new TalonPIDConfig(FLYWHEEL_MASTER_ENCODER_SENSOR_PHASE,
                                                        FLYWHEEL_MASTER_MOTOR_INVERTED,
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
                                                    HOOD_TOP_POSITION,
                                                    HOOD_BOTTOM_POSITION,
                                                    HOOD_SOFT_LIMITS,
                                                    HOOD_VELOCITY,
                                                    HOOD_ACCELERATION,
                                                    HOOD_MOTION_SMOOTHING);

    // Set PID for Turret
    public static final TalonPIDConfig TURRET_CONFIG = new TalonPIDConfig(TURRET_ENCODER_SENSOR_PHASE,
                                                        TURRET_MOTOR_INVERTED,
                                                        TURRET_kP,
                                                        0,
                                                        TURRET_kD,
                                                        0,
                                                        TURRET_TOLERANCE,
                                                        TURRET_FRONT_LIMIT_POSITION,
                                                        TURRET_BACK_LIMIT_POSITION,
                                                        TURRET_SOFT_LIMITS,
                                                        TURRET_VELOCITY,
                                                        TURRET_ACCELERATION,
                                                        TURRET_MOTION_SMOOTHING);
    // Mouse Droid PID values
    public static final double MOUSE_kP = 0.0;
    public static final double MOUSE_kD = 0.0;

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
                                                                        ARM_SOFT_LIMITS);

    //public static final TalonPIDConfig MAGAZINE_CONFIG = new TalonPIDConfig(); //TODO: f

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

    //Magazine Motor Port
    public static final int MAGAZINE_MOTOR_PORT = 10;

    //Intake Motor Port
    public static final int INTAKE_MOTOR_PORT = 13;
    public static final int ARM_MOTOR_PORT = 12;

    //Magazine proximity sensor ports
    public static final int MAGAZINE_ULTRASONIC_BOT = 0;
    public static final int MAGAZINE_ULTRASONIC_TOP = 1;

    // Shooter Motor Ports
    public static final int FLYWHEEL_MASTER_MOTOR_PORT = 5;
    public static final int FLYWHEEL_SLAVE_MOTOR_PORT = 4;
    public static final int HOOD_MOTOR_PORT = 9;
    public static final int TURRET_MOTOR_PORT = 8;

    // Motor Speeds - @author utkarsh
    public static final double MOTOR_STOP = 0;
    public static final double INTAKE_MOTOR_SPEED = 0.4;
    public static final double LIFT_MOTOR_SPEED = 0.6;
    public static final double MAGAZINE_MOTOR_SPEED = 0.45;
    public static final double MOUSE_DROID_SPEED = .5;

    // Ultrasonic Sensor ports
    public static final int ANALOG_PORT = 0;

}
