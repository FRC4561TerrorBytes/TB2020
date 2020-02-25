/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;



//This is a list of Strings of AutoMode Trajectories (which are strings of .json files for each trajectory)
//Auto Mode folder has been made in Deploy (to RoboRIO) folder for Auto Mode .json files
// ALL PATHS WILL BE IN FORMAT OF "src/main/deploy/paths/AutoPaths/"
//ALL PATHS WILL ALSO HAVE TO BE MOVED INTO THE AUTO MODE FOLDER IN DEPLOY
/**
 * Add your docs here.
 */

 //An enum of the .json files for the Auto Mode Trajectories. Is like a list, and has an object that has a 
 //String attributed to it.
public enum AutoModeConstants {
    backForSpace("backForSpace"),
    BlueSGball1("BlueFroShieldBall1"),
    BlueSGball2("BlueFroShieldBall2"),
    BlueSGDriveBack("BlueFroShieldDriveback"),
    BlueSideTrenchBall1("BlueTrenchBall1"),
    BlueSideTrenchBall2("BlueTrenchBall2"),
    BlueSideTrenchBall3("BlueTrenchBall3"),
    DriveBackTrench("driveBack"),
    MoveBackShootingAutoBalls("MoveBackAfterShooting3Balls"),
    RedSideTrenchBall1("RedTrenchBall1"),
    RedSideTrenchBall2("RedTrenchBall2"),
    RedSideTrenchBall3("RedTrenchBall3"),
    KongoAutoMode1("KongoTestAuto"),
    ShootBallsAuto("Shoot");
  
    public final String trajectoryJSON;
    AutoModeConstants(String name) {
      trajectoryJSON = "autopaths/" + name + ".wpilib.json"; // Sets the Strings above to be part of the 
      // relative path. 
    }
  }