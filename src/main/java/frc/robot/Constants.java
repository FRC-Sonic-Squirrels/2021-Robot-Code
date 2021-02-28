/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

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
    public static final class canId {
        // CAN Id 0 is off limits. Typically unconfigured devices default to CAN id zero. This will 
        // create problems if you already have a device using CAN id 0 on the CAN bus.
        public static final int DoNotUse_canId0 = 0;

       // When assigning a CAN ID, rename the constant to something descriptive. Such as
       // when assigning CAN 1 rename "canId1" to "driveLeftLead" or "pigeonIMU"
        public static final int canId1 = 1;
        public static final int canId2 = 2;
        public static final int canId3 = 3;
        public static final int canId4_hopper_agitator = 4;
        public static final int canId5_turret = 5;
        public static final int canId6_hood = 6;
        public static final int canId7 = 7;
        public static final int canId8_indexo_intake_and_hopper = 8;
        public static final int canId9 = 9;
        public static final int canId10_indexo_belts = 10;
        public static final int canId11_indexo_kicker = 11;
        public static final int canId12_drive_left_lead = 12;
        public static final int canId13_drive_left_follow = 13;
        public static final int canId14_drive_right_follow = 14;
        public static final int canId15_drive_right_lead = 15;
        public static final int canId16_flywheel_right = 16;
        public static final int canId17_flywheel_left = 17;
        public static final int canId18_intake = 18;
        public static final int canId19 = 19;
        public static final int canId20_pigeon_imu = 20;
    }

    public static final class currentLimits {
        public static SupplyCurrentLimitConfiguration m_currentlimitMain = new SupplyCurrentLimitConfiguration(true, 35, 1, 1);
        public static SupplyCurrentLimitConfiguration m_currentlimitSecondary = new SupplyCurrentLimitConfiguration(true, 25, 1, 1);
    }

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

        // TalonFX encoders have 2048, Rev Robotics have 4096
        public static final int kEncoderCPR = 2048;

        //TODO: Look into these actual values
        // Aprox 6 inch (0.1524 meters) traction wheels, measured 0.15836 m
        // Measured circumference = 0.509 m
        public static final double kDistancePerWheelRevolutionMeters = 0.509;
        public static final double kWheelDiameterMeters =
                kDistancePerWheelRevolutionMeters / Math.PI;

        // gear reduction from Falcon Gearbox:
        // Two stages 11:60 then 16:32 for a total gear reduction of 11:120
        public static final double kGearReduction = 11.0 / 120.0;


        // Assumes the encoders are directly mounted on the motor shafts
        public static final double kEncoderDistancePerPulseMeters =
                (kDistancePerWheelRevolutionMeters * kGearReduction) / (double) kEncoderCPR;
    }

    public static final class AutoConstants {
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
  
    public static final class turretConstants {
        public static final double turretGearRatio = 132.0;
        public static final int turret = canId.canId5_turret;
        public static final int kSoftMaxTurretAngle = 90;
        public static final int kSoftMinTurretAngle = -90;
    }

    public static final class shooterConstants {
        public static final int shooter1 = 16;
        public static final int shooter2 = 17;
    }

    public static final class limeLightConstants {
        public static final double limeLightHeight_meters = .68394;
        public static final double targetHeight_meters = 2.5019;
        public static final double limeLightAngle_degrees = 30;
    }

    public static final class digitalIOConstants {
        // assign digital IO (DIO) ports 0-9
        public static final int dio0_indexerSensor1 = 0;
        public static final int dio1_indexerSensor2 = 1;
        public static final int dio2_indexerSensor3 = 2;
    }

    public static final class indexConstants {
        public static final int hopperAgitator = canId.canId4_hopper_agitator;
        public static final int indexIntake = canId.canId8_indexo_intake_and_hopper;
        public static final int indexBelts = canId.canId10_indexo_belts;
        public static final int indexKicker = CanID.canId11_indexo_kicker;
    }
  
    public static final class intakeConstants {
        public static final int intakeMotor = 9;
        //Geared up 16:24 
        public static final double intakeGearRatio = 16.0 / 24.0;
    }

    public static final class pwmConstants {
        public static final int blinkin = 0;
    }

    public static final class hoodConstants {
        public static final int hoodMotor = canId.canId6_hood;
    }
}
