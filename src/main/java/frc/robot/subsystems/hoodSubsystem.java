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

  XboxController operatorController = RobotContainer.m_operatorController;

  //Sets Hood position in Degrees using Feet
  private double [][] hoodPos = {
    {4.0, 46.13},
    {5.0, 46.13}, 
    {7.0, 50}, 
    {11.0, 60},
    {12.7, 60}, 
    {15.0, 65}, 
    {20.0, 70},
    {25.0, 70}
  };

  /** Creates a new hoodSubsystem. */
  public hoodSubsystem() {
    
    // TODO: Set current limit

    m_hood.restoreFactoryDefaults();
    m_hood.setInverted(true);

    m_encoder = m_hood.getEncoder();
    m_encoder.setPosition(0);
    
    m_pidController = m_hood.getPIDController();

    m_hoodAngle = new linearInterpolator(hoodPos);

    // TODO: tune PID values
    // PID coefficients (currently default)
    kP = 0.15;
    kI = 1e-4;
    kD = 0;
    kF = 0;
    kIz = 100; 
    kMaxOutput = 0.3;
    kMinOutput = -0.3;

    // Set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);
    m_pidController.setIZone(kIz);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // TODO: use limit switch!
    // TODO: configure limit switch to zero position


    // Display initial set hood angle on SmartDashboard
    SmartDashboard.putNumber("Set Hood Angle", 46);
    SmartDashboard.putNumber("Hood Position Deg", rotationsToAngle(m_encoder.getPosition()));
    SmartDashboard.putNumber("Hood Position Error Pos", 0.0);

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
      double hoodRotations = angleToRotations(SmartDashboard.getNumber("Set Hood Angle", 46));

      // Make sure to not set hood rotations beyond min or max position
      if (hoodRotations < minPos) {
        hoodRotations = minPos;
      }

      else if (hoodRotations > maxPos) {
        hoodRotations = maxPos;
      }

      if (hoodPosition != hoodRotations) {
        hoodPosition = hoodRotations;
        m_pidController.setReference(hoodPosition, ControlType.kPosition);
      }

  }  // end if (m_debug_PID)

    // Display current hood position on SmartDashboard
    m_currentHoodPosition = m_encoder.getPosition();
    m_hoodErrorRotations = m_currentHoodPosition - hoodPosition;

    // Display hood position error on SmartDashboard
    SmartDashboard.putNumber("Hood Position Deg",  rotationsToAngle(m_currentHoodPosition));
    SmartDashboard.putNumber("Hood Position Error Pos", m_hoodErrorRotations);
    SmartDashboard.putBoolean("Hood at Position", isAtPos());
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
  public void zeroHoodPos(){
    m_encoder.setPosition(0);
  }

  /**
   * setPositionRotations()  - Sets hood to position in rotations
   * 
   * @param rotations, position of motor in rotations
   */
  public void setPositionRotations(double rotations){
      if (rotations < minPos + epsilon) {
        rotations = minPos + epsilon;
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
   * getAngleforDistanceFeet() - return Hood Angle based on distance to target in FEET
   * 
   * @param distanceFeet distance in FEET to goal
   * @return Angle for Hood
   */
  public double getAngleforDistanceFeet(double distanceFeet) {
    return m_hoodAngle.getInterpolatedValue(distanceFeet);
  }

 /**
   * getAngleforDistanceMeter() - return Hood Angle based on distance to target in METERS
   * 
   * @param distanceFeet distance in METERS to goal
   * @return Angle for Hood
   */
  public double getAngleforDistanceMeter(double distanceMeters) {
    return getAngleforDistanceFeet(distanceMeters * 3.28084);
  }

  public boolean isAtPos(){
    return Math.abs(m_hoodErrorRotations) < 1.0;
  }

 /**
  * retract the hood to the full down position
  */
  public void retractHood(){
    setPositionRotations(0);
  }

}
