/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.controlPanelConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class controlPanelSubsystem extends SubsystemBase {
  // space for variables
  private WPI_TalonSRX controlPanelMotor = new WPI_TalonSRX(controlPanelConstants.motor);
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  String lastSeenColor = "Unknown";
  String gameData;
  private int count = 0;
  /*
   * Color Wheel Blue CMY: 100,0,0 RGB: #00FFFF Green CMY: 100,0,100 RGB: #00FF00
   * Red CMY: 0,100,100 RGB: #FF0000 Yellow CMY: 0,0,100 RGB: #FFFF00
   */
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

  public controlPanelSubsystem() {
    // Restoring the motors to default settings
    controlPanelMotor.configFactoryDefault();
    controlPanelMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, controlPanelConstants.PIDLoopIdx,
        controlPanelConstants.timeoutMs);
    controlPanelMotor.setSensorPhase(controlPanelConstants.sensorPhase);
    controlPanelMotor.setInverted(controlPanelConstants.motorInvert);
    controlPanelMotor.configVoltageCompSaturation(11);
    controlPanelMotor.enableVoltageCompensation(true);
    controlPanelMotor.configNominalOutputForward(0, controlPanelConstants.timeoutMs);
    controlPanelMotor.configNominalOutputReverse(0, controlPanelConstants.timeoutMs);
    controlPanelMotor.configPeakOutputForward(.2, controlPanelConstants.timeoutMs);
    controlPanelMotor.configPeakOutputReverse(-.2, controlPanelConstants.timeoutMs);
    controlPanelMotor.configAllowableClosedloopError(0, controlPanelConstants.PIDLoopIdx,
        controlPanelConstants.timeoutMs);
    controlPanelMotor.config_kF(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kF,
        controlPanelConstants.timeoutMs);
    controlPanelMotor.config_kP(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kP,
        controlPanelConstants.timeoutMs);
    controlPanelMotor.config_kI(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kI,
        controlPanelConstants.timeoutMs);
    controlPanelMotor.config_kD(controlPanelConstants.PIDLoopIdx, controlPanelConstants.gains.kD,
        controlPanelConstants.timeoutMs);
    controlPanelMotor.setNeutralMode(NeutralMode.Brake);

    controlPanelMotor.setSelectedSensorPosition(0, controlPanelConstants.PIDLoopIdx, controlPanelConstants.timeoutMs);

    // colors we want to match
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
  }

  public void setSpeed(double speed) {
    controlPanelMotor.set(ControlMode.Velocity, speed);
  }

  /**
   * stop - stop the motor.
   */
  public void stop() {
    setSpeed(0.0);
  }

  /**
   * senseColorWheelPos - detect current color, track the number of color
   * transitions. Eight (8) color transistions in one direction is a full rotation
   * of the wheel.
   */
  public void senseColorWheelPos() {

    String colorString = getColor();

    int proximity = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Proximity", proximity);

    /* Code for counting colors */

    if (lastSeenColor.equals("Red")) {
      if (colorString.equals("Green")) {
        count = count + 1;
      }
      if (colorString.equals("Yellow")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Green")) {
      if (colorString.equals("Blue")) {
        count = count + 1;
      }
      if (colorString.equals("Red")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Blue")) {
      if (colorString.equals("Yellow")) {
        count = count + 1;
      }
      if (colorString.equals("Green")) {
        count = count - 1;
      }
    } else if (lastSeenColor.equals("Yellow")) {
      if (colorString.equals("Red")) {
        count = count + 1;
      }
      if (colorString.equals("Blue")) {
        count = count - 1;
      }
    }
    // Color reset and count display on SmartDashboard
    lastSeenColor = colorString;
    SmartDashboard.putNumber("Count", count);

    // TODO: Detect errors and unknown colors
  }

  public String getColor() {
    // This method will be called once per scheduler run
    Color detectedColor = m_colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);

    return colorString;
  }

  /**
   * Given a String color name return the corresponding number. B -> 0 Y -> 1 R ->
   * 2 G -> 3 and return -1 for anything else.
   * 
   * @param colorString
   * @return int color number
   */
  public int colorNumbers(String colorString) {
    if (colorString.length() == 0) {
      // catch possible error if string is empty
      return -1;
    }

    char colorChar = colorString.charAt(0);

    if (colorChar == 'B') {
      return 0;
    }
    if (colorChar == 'Y') {
      return 1;
    }
    if (colorChar == 'R') {
      return 2;
    }
    if (colorChar == 'G') {
      return 3;
    }

    // Error: return an invalid integer
    return -1;
  }

  public int moveToGamePosition() {
    String currentColor = getColor();
    char currentColorChar = currentColor.charAt(0);
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    char stage2ColorChar = gameData.charAt(0);

    // TODO: I think maybe you should split this into two functions.
    // One to get the number of color transtions we need to move
    // from the start position to the desired position.
    // Second to track when we've turned far enough.
    // This will make controlPanelStage2Command look very similar to the
    // Stage1Command
    //
    // I like the comments like "move CCW 2" and "move CW 1" etc. very descriptive.

    // TODO:Figure out what actions to take if one of these options isn't the case

    if (stage2ColorChar == currentColorChar) {
      return 2;
      //move counterclockwise 2
    }
    if (stage2ColorChar == 'B' && currentColorChar == 'Y') {
      return 1;
      // move counterclockwise 1
    }
    if (stage2ColorChar == 'B' && currentColorChar == 'R') {
      return 0;
      // don't move
    }
    if (stage2ColorChar == 'B' && currentColorChar == 'G') {
      return -1;
      // move clockwise 1
    }
    if (stage2ColorChar == 'Y' && currentColorChar == 'B') {
      return 1;
      // move counterclockwise 1
    }
    if (stage2ColorChar == 'Y' && currentColorChar == 'R') {
      return -1;
      // move clockwise 1
    }
    if (stage2ColorChar == 'Y' && currentColorChar == 'G') {
      return 0;
      // don't move
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'Y') {
      return 1;
      // move counterclockwise 1
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'G') {
      return -1;
      // move clockwise 1
    }
    if (stage2ColorChar == 'R' && currentColorChar == 'B') {
      return 0;
      // don't move
    }
    if (stage2ColorChar == 'G' && currentColorChar == 'B') {
      return -1;
      // move clockwise 1
    }
    if (stage2ColorChar == 'G' && currentColorChar == 'Y') {
      return 0;
      // don't move
    }
    if (stage2ColorChar == 'G' && currentColorChar == 'R') {
      return 1;
      // move counterclockwise 1
    }
    return 6;
  }
  public int getMoveToGamePosition(){
    return moveToGamePosition();
  }

  public void setPosition(double position) {
    controlPanelMotor.set(ControlMode.Position, position);
  }

  /**
   * getColorCount - return the number of color transtions seen
   * 
   * @return int count of color transtions
   */
  public int getColorCount() {
    return count;
  }

  /**
   * resetColorCount - reset the number of color transitions seen to zero. Useful
   * if we have to start over or when we start Stage2 and need to zero out the
   * count from Stage0.
   * 
   */
  public void resetColorCount() {
    count = 0;
    lastSeenColor = "Unknown";
  }

  public double getRotationCount() {
    return (count / 8);
  }
}
