/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package com.team2930.lib.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Team 2930 (Sonic Squirrels) Geometry utils.
 */
public class geometry {

    /**
     * Distance from robot to a given point on the field
     * 
     * @param pose     of the robot
     * @param location of the target
     * 
     * @return distance in meters
     **/
    public static double distance2Target(Pose2d robotPose, Translation2d target) {
        return robotPose.getTranslation().getDistance(target);
    }

    /**
     * Angle between robot pose and target
     * 
     * @param pose     of the robot
     * @param location of the target
     * 
     * @return angle to target in Radians
     **/
    public static double angle2TargetRadians(Pose2d robotPose, Translation2d target) {
        double robot_angle_to_field = robotPose.getRotation().getRadians();
        double angle_to_target = Math.atan2(robotPose.getTranslation().getX() - target.getX(),
                robotPose.getTranslation().getY() - target.getY());
        double theta = angle_to_target - robot_angle_to_field;

        // make sure resulting angle is between -pi and pi (or -180 and 180 degrees)
        if (theta < -Math.PI) {
            theta = theta + 2 * Math.PI;
        } else if (theta > Math.PI) {
            theta = theta - 2 * Math.PI;
        }

        return theta;
    }

    /**
     * Angle between robot pose and target
     * 
     * @param pose     of the robot
     * @param location of the target
     * 
     * @return angle to target in Degrees
     **/
    public static double angle2TargetDegrees(Pose2d robotPose, Translation2d target) {
        return Math.toDegrees(angle2TargetRadians(robotPose, target));
    }

    public static double inches2Meters(double i) {
        return i * 0.0254;
    }

    public static double feet2Meters(double feet) {
        return (feet * 0.3048);
    }

    public static double meters2Feet(double meters){
        return (meters / 0.3048);
    }

}