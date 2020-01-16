/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    
    // Drive PID values
    public static final double DRIVE_kP = 0.15;
    public static final double DRIVE_kD = 0.0025;
    public static final double DRIVE_PERIOD_SECONDS = 0.0041675;
    public static final double DRIVE_TURN_SCALAR = 10.0;
    public static final double DRIVE_TOLERANCE = 1.0;

    public static final double kMetersToTicks = 0.000116889336037;
    public static final double kTicksToMeters = 8555.10035308;
    public static final double AUTO_SPEED = 0.5;
    
    // Analog stick deadband value
    public static final double DEADBAND = 0.02;
    
    // Joystick Ports
    public static final int RIGHT_JOYSTICK_PORT = 1;
    public static final int LEFT_JOYSTICK_PORT = 0;

    // Xbox controller
    public static final int XBOX_CONTROLLER_PORT = 2;    

    // Drive Motor Ports 
    public static final int FRONT_LEFT_MOTOR_PORT = 3;
    public static final int REAR_LEFT_MOTOR_PORT = 5;

    public static final int FRONT_RIGHT_MOTOR_PORT = 0;
    public static final int REAR_RIGHT_MOTOR_PORT = 8;

    /**
     * TODO: ADD COMMENTS ONCE FINISHED
     */

    public static final double kTrackwidthMeters = 0.53975;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final int kEncoderTPR = 4096;
    public static final double kWheelDiameterMeters = 0.1524;
    public static final double kEncoderDistancePerPulse = (kWheelDiameterMeters * Math.PI) / (double) kEncoderTPR;

    public static final boolean kGyroReversed = true;

    /**
     * TODO: CHANGE TO ACTUAL VALUES
     */

    public static final double ksVolts = 0.0;
    public static final double kvVoltSecondsPerMeter = 0.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0;
    public static final double kPDriveVel = 0.0;
	public static final double kMaxSpeedMetersPerSecond = 0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0;
    
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

}
