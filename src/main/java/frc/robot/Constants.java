// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 * 
 * For example:
 * 
 * import static frc.robot.Constants.canId;
 * 
 *  ...
 * 
 * private WPI_TalonFX leftLeadFalcon  = new WPI_TalonFX(canId.leftLeadFalcon);
 * 
 */
public final class Constants {

    // Allocate CAN Ids from here. 
    // This avoids accidentally assigning the same CAN id to two different devices.
    public static final class driveConstants {
        public static final int falcon1_leftLead = 12;
        public static final int falcon2_leftFollow = 13;
        public static final int falcon3_rightLead = 14;
        public static final int falcon4_rightFollow = 15;
        public static final int driveTimeout = 30;
        public static final int pigeonCANid = 15;
        public static final int driveController = 0;
        public static final int operatorController = 1;

        public static final boolean kLeftEncoderReversed = false;
        public static final boolean kRightEncoderReversed = false;

        public static final boolean kGyroReversed = true;

        //TODO: Look into if these values are still the same with New Comp Bot
        // Comp bot track width (center of wheel to center of wheel) is 0.627m
        public static final double kTrackwidthMeters = 0.627;
        public static final DifferentialDriveKinematics kDriveKinematics = 
            new DifferentialDriveKinematics(kTrackwidthMeters);

        // Determined using frc-characterization tool
        public static final double ksVolts = 0.05; // 0.0491;
        public static final double kvVoltSecondsPerMeter = 2.36; // 2.36;
        public static final double kaVoltSecondsSquaredPerMeter = 0.127;

        // Determined using frc-characterization
        public static final double kPDriveVel = 5.0; // frc-characterization 14.4
        public static final double kDDriveVel = 0.0; // frc-characterization 5.16

        // TalonFX encoders have 2048, Rev Robotics have 4096
        public static final int kEncoderCPR = 2048;

        // Aprox 6 inch (0.1524 meters) traction wheels, measured 0.15836 m
        // Measured circumference = 0.509 m
        public static final double kDistancePerWheelRevolutionMeters = 0.509;
        public static final double kWheelDiameterMeters =
                kDistancePerWheelRevolutionMeters / Math.PI;

        // gear reduction from Falcon Gearbox:
        // Two stages 11:60 then 16:31 for a total gear reduction of 11:120
        public static final double kGearReduction = 11.0 / 120.0;

        // Assumes the encoders are directly mounted on the motor shafts
        public static final double kEncoderDistancePerPulseMeters =
                (kDistancePerWheelRevolutionMeters * kGearReduction) / (double) kEncoderCPR;
    }
}
