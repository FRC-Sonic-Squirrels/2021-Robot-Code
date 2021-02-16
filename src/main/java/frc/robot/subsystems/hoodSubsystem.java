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
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;

public class hoodSubsystem extends SubsystemBase {

  private CANSparkMax m_hood = new CANSparkMax(hoodMotor, MotorType.kBrushless);
  private CANEncoder m_encoder;
  private CANPIDController m_pidController;
  private double kP, kI, kD, kF, kIz, kMaxOutput, kMinOutput;
  private double hoodPosition = 0;
  private double minPos = 0;
  private double maxPos = 3.5;

  XboxController operatorController = RobotContainer.m_operatorController;

  /** Creates a new hoodSubsystem. */
  public hoodSubsystem() {
    
    // TODO: Set current limit

    m_hood.restoreFactoryDefaults();
    m_hood.setInverted(true);

    m_encoder = m_hood.getEncoder();
    m_encoder.setPosition(0);
    
    m_pidController = m_hood.getPIDController();

    // TODO: set and tune PID values
    // PID coefficients (currently default)
    kP = 0.2;
    kI = 1e-4;
    kD = 0;
    kF = 0;
    kIz = 100; 
    kMaxOutput = 1;
    kMinOutput = -1;

    // Set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setFF(kF);
    m_pidController.setIZone(kIz);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // Display PID values on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("Feed Forward", kF);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);

    SmartDashboard.putNumber("Hood output", m_hood.getAppliedOutput());

    // Display initial set hood angle on SmartDashboard
    // SmartDashboard.putNumber("Set Hood Position", 0);
    SmartDashboard.putNumber("Set Hood Angle", 46);
    SmartDashboard.putNumber("Hood Position", m_encoder.getPosition());

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Retrieve PID values from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double f = SmartDashboard.getNumber("Feed Forward", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if(p != kP) {
      m_pidController.setP(p);
      kP = p;
    }
    if(i != kI) {
      m_pidController.setI(i);
      kI = i;
    }
    if(d != kD) {
      m_pidController.setD(d);
      kD = d;
    }
    if(f != kF) {
      m_pidController.setFF(f);
      kF = f;
    }
    if(iz != kIz) {
      m_pidController.setIZone(iz);
      kIz = iz;
    }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min;
      kMaxOutput = max; 
    }

    // Retrieve set hood angle from SmartDashboard and convert to motor rotations
    // double hoodRotations = SmartDashboard.getNumber("Set Hood Position", 0);
    double hoodRotations = angleToRotations(SmartDashboard.getNumber("Set Hood Angle", 46));

    // Make sure to not set hood rotations beyond min or max position
    if(hoodRotations < 0) {
      hoodRotations = 0;
    }
    else if(hoodRotations > 3.5) {
      hoodRotations = 3.5;
    }

    // hoodRotations = (0.95 * (maxPos - minPos) * operatorController.getTriggerAxis(Hand.kRight)) - minPos;

    if (hoodPosition != hoodRotations) {
      hoodPosition = hoodRotations;
      m_pidController.setReference(hoodPosition, ControlType.kPosition);
    }

    // Display current hood position on SmartDashboard
    SmartDashboard.putNumber("Hood Position", m_encoder.getPosition());

    // Display hood position error on SmartDashboard
    SmartDashboard.putNumber("Hood Position Error", m_encoder.getPosition() - hoodPosition);
    SmartDashboard.putNumber("Hood output", m_hood.getAppliedOutput());

  }

  // converts hood degrees (above horizontal) to motor rotations
  private double angleToRotations(double angle) {

    return (3.5 / 29) * (angle - 46);

  }
}
