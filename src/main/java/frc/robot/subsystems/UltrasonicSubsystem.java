package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class UltrasonicSubsystem extends Subsystem {
	
	@Override
	protected void initDefaultCommand() {
	}
	
	  // (pins 3, 6 and 7 from sensor to analog input 0)
	  private static final AnalogInput ultrasonic = new AnalogInput(Constants.ANALOG_PORT);
	  
	  // TODO - You will need to determine how to convert voltage to distance
	  // (use information from the data sheet, or your own measurements)
	  private static final double VOLTS_TO_DIST = 1.0;

	  public static double getVoltage() {
	    return ultrasonic.getVoltage();
	  }
	  
	  public static void updateDashboard() {
	    SmartDashboard.putNumber("Voltage: ", getVoltage());
	  }

	}