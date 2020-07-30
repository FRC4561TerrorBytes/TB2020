/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

/**
 * Add your docs here.
 */
public class AutoModePaths {

    public static List<Pose2d> ShootDriveStraight = new ArrayList<Pose2d>(){
        {
            add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            add(new Pose2d(2, 0, Rotation2d.fromDegrees(0)));
        }
    };

    public static List<Pose2d> ShootDriveBack = new ArrayList<Pose2d>(){
        {
            add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            add(new Pose2d(3, 0, Rotation2d.fromDegrees(0)));
        }
    };

    public static List<Pose2d> TrenchSixBallPt1 = new ArrayList<Pose2d>(){
        {
            add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            add(new Pose2d(5, 0, Rotation2d.fromDegrees(0)));
        }
    };
    public static List<Pose2d> TrenchSixBallPt2 = new ArrayList<Pose2d>(){
        {
            add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            add(new Pose2d(-5, -2, Rotation2d.fromDegrees(20)));
        }
    };

    public static List<Pose2d> f = new ArrayList<Pose2d>(){
        {
            add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            add(new Pose2d(-5, -2, Rotation2d.fromDegrees(20)));
        }
    };

    // UNTESTED
    public static List<Pose2d> Shoot_Middle = new ArrayList<Pose2d>() {
        {
            add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            add(new Pose2d(5, -2, Rotation2d.fromDegrees(30)));
        }
    };

    // UNTESTED
    public static List<Pose2d> Shoot_LoadingZone = new ArrayList<Pose2d>() {
        {
            add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
            add(new Pose2d(5, -4, Rotation2d.fromDegrees(60)));
        }
    };
}
