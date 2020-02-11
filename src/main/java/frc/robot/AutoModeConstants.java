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
         public static String trajectoryJSON = "src/main/deploy/AutoPaths/testpath.wpilib.json"; 
        
}


    public static class backForSpace {
        public static String trajectoryJSON = "src/main/deploy/output/backForSpace.wpilib.json";
    }

    public static class BlueSGball1{
        public static String trajectoryJSON ="src/main/deploy/output/BlueFroShieldBall1.wpilib.json";
    }

    public static class BlueSGball2{
        public static String trajectoryJSON = "src/main/deploy/output/BlueFroShieldBall2.wpilib.json";
    }

    public static class BlueSGDriveback{ 
        public static String trajectoryJSON ="src/main/deploy/output/BlueFroShieldDriveback.wpilib.json";
    }

    public static class BlueTrenchBall1{ 
        public static String trajectoryJSON ="src/main/deploy/output/BlueTrenchBall1.wpilib.json";
    }

    public static class BlueTrenchBall2{ 
        public static String trajectoryJSON ="src/main/deploy/output/BlueTrenchBall2.wpilib.json";
    }

    public static class BlueTrenchBall3{ 
        public static String trajectoryJSON ="src/main/deploy/output/BlueTrenchBall3.wpilib.json";
    }

    public static class driveBack{ 
        public static String trajectoryJSON ="src/main/deploy/output/driveBack.wpilib.json";
    }

    public static class MoveBackAfterShooting3Balls{ 
        public static String trajectoryJSON ="src/main/deploy/output/MoveBackAfterShooting3Balls.wpilib.json";
    }


    public static class RedTrenchBall1{ 
        public static String trajectoryJSON ="src/main/deploy/output/RedTrenchBall1.wpilib.json";
    }
    
    public static class RedTrenchBall2{ 
        public static String trajectoryJSON ="src/main/deploy/output/RedTrenchBall2.wpilib.json";
    }

    public static class RedTrenchBall3{ 
        public static String trajectoryJSON ="src/main/deploy/output/RedTrenchBall3.wpilib.json";
    }

    public static class ShootBalls{ 
        public static String trajectoryJSON ="src/main/deploy/output/Shoot.wpilib.json";
    }

}

