/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



//This is a list of Strings of AutoMode Trajectories (which are strings of .json files for each trajectory)
//Auto Mode folder has been made in Deploy (to RoboRIO) folder for Auto Mode .json files
// ALL PATHS WILL BE IN FORMAT OF "src/main/deploy/output"

/**
 * Add your docs here.
 */

 //An enum of the .json files for the Auto Mode Trajectories. 
 public enum AutoModeConstants {
  DriveStraightTest("DriveStraightTest"),
  ShootDriveStraight("InFrontGoalDriveStraight"),
  ShootDriveBack("InFrontGoalDriveBack"),
  SixBallTrench("TrenchThreeBall"),
  SixBallTrenchPt2("TrenchThreeBallPt2");
  
    public final String trajectoryJSON;
    AutoModeConstants(String name) {
      trajectoryJSON = "output/" + name + ".wpilib.json"; // Sets the Strings above to be part of the relative path.
    }
  }
