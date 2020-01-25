/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SpinnerSubsystem extends SubsystemBase {

  private final WPI_TalonSRX SPINNER_MOTOR = new WPI_TalonSRX(Constants.SPINNER_MOTOR_PORT);

  private final ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

  private final ColorMatch colorMatcher = new ColorMatch();

  private final HashMap<Color, String> colorMap = new HashMap<Color, String>() {
    {
      put(redTarget, "Red");
      put(yellowTarget, "Yellow");
      put(greenTarget, "Green");
      put(blueTarget, "Blue");
    }
  };

  private final HashMap<String, Color> controlPanelMap = new HashMap<String, Color>() {
    {
      put("red", blueTarget);
      put("green", yellowTarget);
      put("blue", redTarget);
      put("yellow", greenTarget);
    }
  };

  private final HashMap<Color, Color> colorMapMinusOne = new HashMap<Color, Color>() {
    {
      put(redTarget, greenTarget);
      put(greenTarget, blueTarget);
      put(blueTarget, yellowTarget);
      put(yellowTarget, redTarget);
    }
  };

  private final Color redTarget = ColorMatch.makeColor(0.5, 0.36, 0.14);
  private final Color yellowTarget = ColorMatch.makeColor(0.32, 0.56, 0.13);
  private final Color greenTarget = ColorMatch.makeColor(0.17, 0.58, 0.25);
  private final Color blueTarget = ColorMatch.makeColor(0.12, 0.43, 0.44);

  private double threshold;

  
  /**
   * Creates a new SpinnerSubsytem.
   */
  public SpinnerSubsystem(double threshold) {
    this.threshold = threshold;

    // Add colors to color match.
    colorMatcher.addColorMatch(redTarget);
    colorMatcher.addColorMatch(yellowTarget);
    colorMatcher.addColorMatch(greenTarget);
    colorMatcher.addColorMatch(blueTarget);
  }

  /**
   * Makes the spinner motor spin.
   * @param power spin speed [-1.0, 1.0]
   */
  public void manualSpin(double power) {
    SPINNER_MOTOR.set(power);
  }

  /**
   * Stops spinner motor.
   */
  public void stopSpinner() {
    SPINNER_MOTOR.set(0);
  }

  /**
   * Does nothing. (for now)
   */
  public void encoderSpin() {

  }

  /**
   * Moves spinner to correct color.
   * @param targetColorString
   * @return the direction that the motor is spinning -1 or 1 and 0 if motor doesn't spin.
   */
  public int goToColor(String targetColorString) {
    Color targetColor = controlPanelMap.get(targetColorString.toLowerCase());
    if (!getColorMatch(getDetectedColor()).color.equals(targetColor)){
      if(targetColor.equals(colorMapMinusOne.get(targetColor))) {
        SPINNER_MOTOR.set(-0.25);
        return -1;
      } else {
        SPINNER_MOTOR.set(0.25);
        return 1;
      }
    } else return 0;
  }

  public boolean checkCorrectColor(String targetColorString) {
    Color targetColor = controlPanelMap.get(targetColorString.toLowerCase());
    if (getColorMatch(getDetectedColor()).color.equals(targetColor)) {
      return true;
    }
    else return false;
  }
  
  /**
   * Gets the color that the sensor detects. 
   * @return detected color.
   */
  public Color getDetectedColor() {
    return colorSensor.getColor();
  }

  /**
   * Gets the color sensor.
   * @return color sensor object.
   */
  public ColorSensorV3 getSensor() {
    return colorSensor;
  }

  /**
   * Takes in a color and returns the closest matching color.
   * @param color The color you want to match.
   * @return The closest color in the color map.
   */
  public ColorMatchResult getColorMatch(Color color) {
    return colorMatcher.matchClosestColor(color);
  }

/**
 * Takes in color and returns the confidence of the closest matching color.
 * @param color The color you want get confidence.
 * @return The confidence of the closest color in the color map.
 */
  public double getConfidence(Color color) {
    return this.getColorMatch(color).confidence;
  }

  /**
   * Takes in color and returns true if confidence is above a threshold.
   * @param color The color you want to check confidence.
   * @return True if confidence is above threshold else false.
   */
  public boolean checkConfidence(Color color) {
    if (this.getConfidence(color) > this.threshold) {
      return true;
    }
    return false;
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Color detectColor = getDetectedColor();

    // Creates a hashmap that finds colors (String) based on color input (Color)

    String colorString = colorMap.get(getColorMatch(detectColor));

    SmartDashboard.putNumber("Red", detectColor.red);
    SmartDashboard.putNumber("Green", detectColor.green);
    SmartDashboard.putNumber("Blue", detectColor.blue);
    SmartDashboard.putNumber("Confidence", getConfidence(detectColor));
    SmartDashboard.putString("Detected Color", colorString);


  }
}

