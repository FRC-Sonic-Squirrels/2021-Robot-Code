/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Field location constants.
 */
public class FieldLocations {

    // coords come from:
    //  https://www.chiefdelphi.com/t/field-coordinates/374470/2
    //  https://github.com/frc1444/robo-sim/blob/master/api/src/main/java/com/first1444/sim/api/frc/implementations/infiniterecharge/Field2020.kt
    //  https://github.com/frc1444/robo-sim/blob/master/gdx/src/main/java/com/first1444/sim/gdx/implementations/infiniterecharge2020/FieldSetup2020.kt

    // Origin (0,0) is center of field
    public static final double WIDTH = inchesToMeters(26 * 12 + 11.25);
    // 5.26 inches comes from page 5 of https://firstfrc.blob.core.windows.net/frc2020/PlayingField/LayoutandMarkingDiagram.pdf
    public static final double LENGTH = inchesToMeters(52 * 12 + 5.26);

    // General form: WIDTH / 2 - inchesToMeters(distance from right guardrail (relative to our Power Port) to point on field of coordinate), LENGTH / 2 - inchesToMeters(distance from our alliance's Alliance Station 2 to point on field of coordinate)
    // coordinates of our Power Port
    public static final Translation2d powerPort = new Translation2d(WIDTH / 2 - inchesToMeters(94.66), 0.0);
    // coordinates of leftmost Power Cell relative to our Power Port
    public static final Translation2d powerCell1 = new Translation2d(WIDTH / 2 - inchesToMeters(94.66 + 191.43 + 1 * 12 + 7.5), LENGTH / 2 - inchesToMeters(10 * 12 + 130.36));
    // coordinates of 2nd leftmost Power Cell relative to our Power Port
    public static final Translation2d powerCell2 = new Translation2d(WIDTH / 2 - inchesToMeters(94.66 + 191.43), LENGTH / 2 - inchesToMeters(10 * 12 + 130.36));
    // coordinates of 7th extra Power Cell relative to our Power Port
    // public static final Translation2d extraPowerCell7 = new Translation2d(WIDTH / 2 - inchesToMeters(), LENGTH / 2 - inchesToMeters());


   public static double inchesToMeters(double i) {
       return(i * 0.0254);
   }

}
