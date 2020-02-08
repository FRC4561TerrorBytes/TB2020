/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

//This is a list of Strings of AutoMode Trajectories (which are strings of .json files for each trajectory)
//Auto Mode folder has been made in Deploy (to RIoboRIO) folder for Auto Mode .json files
// ALL PATHS WILL BE IN FORMAT OF "src/main/deploy/paths/AutoPaths/"
//ALL PATHS WILL ALSO HAVE TO BE MOVED INTO THE AUTO MODE FOLDER IN DEPLOY
/**
 * Add your docs here.
 */
public class AutoModeConstants {

    public  static class AM_PATH1 { //TODO: Change this String to an actual path
         public static String trajectoryJSON = "src/main/deploy/paths/AutoPaths/testpath.wpilib.json"; 
        
}

}

