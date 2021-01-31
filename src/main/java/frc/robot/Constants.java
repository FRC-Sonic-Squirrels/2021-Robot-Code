// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
        public static final int canId4 = 4;
        public static final int canId5 = 5;
        public static final int canId6 = 6;
        public static final int canId7 = 7;
        public static final int canId8 = 8;
        public static final int canId9 = 9;
        public static final int canId10 = 10;
        public static final int canId11 = 11;
        public static final int canId12 = 12;
        public static final int canId13 = 13;
        public static final int canId14 = 14;
        public static final int canId15 = 15;
        public static final int canId16 = 16;
        public static final int canId17 = 17;
        public static final int canId18 = 18;
        public static final int canId19 = 19;
    }
}
