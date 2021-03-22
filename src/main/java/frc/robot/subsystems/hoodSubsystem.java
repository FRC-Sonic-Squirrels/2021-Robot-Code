// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.hoodConstants.hoodMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.team2930.lib.util.linearInterpolator;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANDigitalInput;

public class hoodSubsystem extends SubsystemBase {

  private CANSparkMax m_hood = new CANSparkMax(hoodMotor, MotorType.kBrushless);
  private CANEncoder m_encoder;
  private CANPIDController m_pidController;
  private double kP, kI, kD, kF, kIz, kMaxOutput, kMinOutput;
  private double hoodPosition = 0;
  private double minAngle = 46.13;
  private double maxAngle = 75.76;
  private double minPos = 0;
  private double maxPos = 38;  //  (50.0 * 28.0) / 36.0;
  // epsilon is how many rotations, we leave as a buffer at the top and bottom to avoid bottoming out on hard stops
  // 8 rotations is roughly 0.2 degrees.
  private double epsilon = 0.5;
  private linearInterpolator m_hoodAngle;
  private double m_hoodErrorRotations;
  private double m_currentHoodPosition;
  private boolean m_tune_PID = false;
  private CANDigitalInput m_forwardLimit;

  private int m_hoodZeroCount = 0;
  private boolean atForwardLimit = false;
  private boolean prevAtForwardLimit = false;
  private boolean zeroed = false;

  XboxController operatorController = RobotContainer.m_operatorController;

  //Sets Hood position in Degrees using Feet
 private double [][] hoodPos = {
    {4.0, 46.13},
    {5.0, 46.13},
    {6.6, 50.0},
    {7.0, 50}, 
    {11.0, 60},
    {12.7, 62},
    {15.0, 65}, 
    {20.0, 69},
    {25.0, 70}
  };

  /** Creates a new hoodSubsystem. */
  public hoodSubsystem() {
    
    m_hood.restoreFactoryDefaults();
    m_hood.setInverted(true);

    m_encoder = m_hood.getEncoder();
    m_pidController = m_hood.getPIDController();
    zeroHoodPos();
    m_hoodAngle = new linearInterpolator(hoodPos);

    // PID coefficients (currently default)
    kP = 0.15;
    kI = 1e-4;
    kD = 0;
    kF = 0;
    kIz = 10; 
    kMaxOutput = 0.3;
    kMinOutput = -0.3;

    // Set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);
    m_pidController.setIZone(kIz);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // Display initial set hood angle on SmartDashboard
    SmartDashboard.putNumber("Set Hood Angle", minAngle);
    SmartDashboard.putNumber("Hood Position Deg", rotationsToAngle(m_encoder.getPosition()));
    SmartDashboard.putNumber("Hood Position Error Pos", 0.0);
    SmartDashboard.putNumber("Hood Zero Count", m_hoodZeroCount);

    /**
     * A CANDigitalInput object is constructed using the getForwardLimitSwitch() or
     * getReverseLimitSwitch() method on an existing CANSparkMax object, depending
     * on which direction you would like to limit
     * 
     * Limit switches can be configured to one of two polarities:
     *  com.revrobotics.CANDigitalInput.LimitSwitchPolarity.kNormallyOpen
     *  com.revrobotics.CANDigitalInput.LimitSwitchPolarity.kNormallyClosed
     */
    m_forwardLimit = m_hood.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);

    /**
     * Limit switches are enabled by default when they are initialized. They can be disabled
     * by calling enableLimitSwitch(false) on a CANDigitalInput object
     * 
     * Limit switches can be reenabled by calling enableLimitSwitch(true)
     * 
     * The isLimitSwitchEnabled() method can be used to check if the limit switch is enabled
     */
    m_forwardLimit.enableLimitSwitch(false);

    prevAtForwardLimit = atForwardLimit = m_forwardLimit.get();

    if (atForwardLimit == false) {
      m_hood.set(-0.05);
    }

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (m_tune_PID) {
      // Retrieve PID values from SmartDashboard
      double p = SmartDashboard.getNumber("P Gain", 0);
      double i = SmartDashboard.getNumber("I Gain", 0);
      double d = SmartDashboard.getNumber("D Gain", 0);
      double f = SmartDashboard.getNumber("Feed Forward", 0);
      double iz = SmartDashboard.getNumber("I Zone", 0);
      double max = SmartDashboard.getNumber("Max Output", 0);
      double min = SmartDashboard.getNumber("Min Output", 0);

      // if PID coefficients on SmartDashboard have changed, write new values to
      // controller
      if (p != kP) {
        m_pidController.setP(p);
        kP = p;
      }
      if (i != kI) {
        m_pidController.setI(i);
        kI = i;
      }
      if (d != kD) {
        m_pidController.setD(d);
        kD = d;
      }
      if (f != kF) {
        m_pidController.setFF(f);
        kF = f;
      }
      if (iz != kIz) {
        m_pidController.setIZone(iz);
        kIz = iz;
      }
      if ((max != kMaxOutput) || (min != kMinOutput)) {
        m_pidController.setOutputRange(min, max);
        kMinOutput = min;
        kMaxOutput = max;
      }

      // Retrieve set hood angle from SmartDashboard and convert to motor rotations
      // double hoodRotations = SmartDashboard.getNumber("Set Hood Position", 0);
      double hoodRotations = angleToRotations(SmartDashboard.getNumber("Set Hood Angle", minAngle));

      if (hoodPosition != hoodRotations) {
        hoodPosition = hoodRotations;
        // m_pidController.setReference(hoodPosition, ControlType.kPosition);
        setPositionRotations(hoodPosition);
      }
    }  // end if (m_debug_PID)

    // Display current hood position on SmartDashboard
    m_currentHoodPosition = m_encoder.getPosition();
    m_hoodErrorRotations = m_currentHoodPosition - hoodPosition;

    // Display hood position error on SmartDashboard
    SmartDashboard.putNumber("Hood Position Deg",  rotationsToAngle(m_currentHoodPosition));
    SmartDashboard.putNumber("Hood Position Error Pos", m_hoodErrorRotations);
    SmartDashboard.putBoolean("Hood at Position", isAtPos());

    /**
     * The get() method can be used on a CANDigitalInput object to read the state of the switch.
     * 
     * In this example, the polarity of the switches are set to normally closed. In this case,
     * get() will return true if the switch is pressed. It will also return true if you do not 
     * have a switch connected. get() will return false when the switch is released.
     */
    atForwardLimit = m_forwardLimit.get();
    SmartDashboard.putBoolean("Forward Limit Switch", atForwardLimit);

    // If limit switch is triggered, zero the hood encoder.
    if (! zeroed && atForwardLimit && (atForwardLimit != prevAtForwardLimit)) {
      zeroHoodPos();
      zeroed = true;
    }

    prevAtForwardLimit = atForwardLimit;
  }

  /**
   * converts hood degrees (above horizontal) to motor rotations
   * 
   * @param angleDegrees
   * @return rotations
   */
  public double angleToRotations(double angleDegrees) {
    // 46.13  degrees ->  0 rotations
    // 75.76  degrees ->  38 rotations
    return  50.0 * (28.0/36.0) * (angleDegrees - minAngle)/(maxAngle - minAngle);
  }

  public double rotationsToAngle(double rotations){
    return (rotations * (maxAngle - minAngle) / 38) + minAngle;
  }

  /**
   * rest the zero hood position to the current hood location.
   */
  public void zeroHoodPos() {
    m_encoder.setPosition(0);
    m_pidController.setIAccum(0.0);
    m_hoodZeroCount++;
    SmartDashboard.putNumber("Hood Zero Count", m_hoodZeroCount);
  }

  /**
   * setPositionRotations()  - Sets hood to position in rotations
   * 
   * @param rotations, position of motor in rotations
   */
  public void setPositionRotations(double rotations){
      if (rotations < minPos) {
        rotations = minPos;
      }
      if (rotations > maxPos - epsilon) {
        rotations = maxPos - epsilon;
      }
      hoodPosition = rotations;
      m_pidController.setReference(rotations, ControlType.kPosition);
  }

  /**
   * setPercentOutput()  - override Hood Motors with percent output
   * 
   * @param percent, percent motor output -1.0 to 1.0
   */
  public void setPercentOutput(double percent){
      m_hood.set(percent);
  }

  /**
   * getAngleforDistanceFeet() - return Hood Angle based on distance to target in
   * FEET
   * 
   * @param distanceFeet distance in FEET to goal
   * @return Angle for Hood
   */
  public double getAngleforDistanceFeet(double distanceFeet) {
    return m_hoodAngle.getInterpolatedValue(distanceFeet);
  }

  /**
   * getAngleforDistanceMeter() - return Hood Angle based on distance to target in
   * METERS
   * 
   * @param distanceFeet distance in METERS to goal
   * @return Angle for Hood
   */
  public double getAngleforDistanceMeter(double distanceMeters) {
    return getAngleforDistanceFeet(distanceMeters * 3.28084);
  }

  /**
   * isAtPos() - returns true if hood is at desired hood angle.
   * 
   * @return boolean
   */
  public boolean isAtPos() {
    return Math.abs(m_hoodErrorRotations) < 1.0;
  }

  /**
   * isAtLowerLimit() - returns true if hood is triggering the low angle limit
   * switch.
   * 
   * @return boolean
   */
  public boolean isAtLowerLimit() {
    return atForwardLimit;
  }

  /**
   * retract the hood to the full down position
   */
  public void retractHood() {
    setPositionRotations(0);
  }

}
