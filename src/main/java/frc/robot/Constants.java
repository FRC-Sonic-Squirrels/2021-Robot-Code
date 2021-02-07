// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
 * private WPI_TalonFX leftLeadFalcon  = new WPI_TalonFX(canId.leftLeadFalcon);che
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

        // TalonFX encoders have 2048, Rev Robitics have 4096
        public static final int kEncoderCPR = 2048;

        //TODO: Look into these actual values
        // Aprox 6 inch (0.1524 meters) traction wheels, measured 0.15836 m
        // Measured circumference = 0.509 m
        public static final double kDistancePerWheelRevolutionMeters = 0.509;
        public static final double kWheelDiameterMeters =
                kDistancePerWheelRevolutionMeters / Math.PI;

        // gear reduction from Falcon Gearbox:
        // Two stages 11:60 then 16:31 for a total gear reduction of 8.68:1
        public static final double kGearReduction = (50 / 12) * (50 / 24);

        // Assumes the encoders are directly mounted on the motor shafts
        public static final double kEncoderDistancePerPulseMeters =
                (kDistancePerWheelRevolutionMeters * kGearReduction) / (double) kEncoderCPR;
    }
    public static final class turretConstants {

        public static final int turret = 5;
        public static final int kSoftMaxTurretAngle = 90;
        public static final int kSoftMinTurretAngle = -90;
        public static final int kEncoderCPR = 4096; // CTRE Quadrature?

        // BAG motor controlled by Talon
        // Turret inner teeth = 264
        // gear teeth = 20
        // gearbox = 35:1
        // Total motor to turret rotation ration: 1:462  (1:35 * 20:264)
        // POST gearbox encoder ratio is:
        // 20:264 or 1/13.2
        //TODO: Change this Gear Ratio
        public static final double kGearRation = 1 / 13.2; // turret rotations per encoder rotation
        public static final double kTurretRotationsPerTick = kGearRation / kEncoderCPR;
        public static final double kDegreesPerTick = 360 * kTurretRotationsPerTick;
        // TODO: test and increase max velocity and acceleration
        // Max velocity: 90 deg/s
        // Max acceleration: 45 deg/s^2
        public static final double kMaxDegreesPerSecond = 90;
        public static final double kMaxDegreesPerSecondSquared = 45;
        public static final int kTimeout = 30; // Talon command timeout
        public static final int kIndex = 0; // Talon PID index
    }

    public static final class shooterConstants {
        public static final int shooter1 = 16;
        public static final int shooter2 = 17;
        public static final int shooterTimeout = 30;
        public static final int shooterSlotIdx = 0;
        public static final int shooterHood = 6;
    }

    public static final class limeLightConstants {
        public static final double limeLightHeight_meters = .68394;
        public static final double targetHeight_meters = 2.5019;
        public static final double limeLightAngle_degrees = 30;
    }

    public static final class digitalIOConstants {
        // assign digital IO (DIO) ports 0-9
        //public static final int dio0_indexerSensor1 = 0;
        //public static final int dio1_indexerSensor2 = 1;
        //public static final int dio2_indexerSensor3 = 2;
        public static final int dio7_turretLimit = 7;
    }
}
