/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.turretConstants.kSoftMaxTurretAngle;
import static frc.robot.Constants.turretConstants.kSoftMinTurretAngle;
import static frc.robot.Constants.turretConstants.turret;

public class turretSubsystem extends SubsystemBase {

  //Neo 550 is the Motor Controlling the Turret
  private CANSparkMax turretDrive = new CANSparkMax(turret, MotorType.kBrushless);
  private CANEncoder m_encoder;
  private CANPIDController m_pidController;

  public turretSubsystem() {
    turretDrive.restoreFactoryDefaults();
    m_encoder = turretDrive.getEncoder();

    //Sets Turret Position as 0
    m_encoder.setPosition(0);

    turretDrive.setSoftLimit(SoftLimitDirection.kForward, (float)( kSoftMaxTurretAngle * Constants.turretConstants.turretGearRatio / 360));
    turretDrive.setSoftLimit(SoftLimitDirection.kReverse, (float)( kSoftMinTurretAngle * Constants.turretConstants.turretGearRatio / 360));
    
    turretDrive.enableSoftLimit(SoftLimitDirection.kForward, true);
    turretDrive.enableSoftLimit(SoftLimitDirection.kReverse, true);

    //turretDrive.configContinuousCurrentLimit(25);

    m_pidController = turretDrive.getPIDController();

    // Set PID coefficients. Currently default
    m_pidController.setP(0.4);
    m_pidController.setI(1e-4);
    m_pidController.setD(0);
    m_pidController.setFF(0);
    m_pidController.setIZone(100);
    m_pidController.setOutputRange(-0.5, 0.5);

  }

  /**
   * turretHome()  rotate turret to zero degrees
   */
  public void turretHome() {
     m_encoder.setPosition(0);
  }

  /**
   * setAngleDegrees - turn turret to a given angle relative to robot
   * 
   * @param angle in degrees
   */
  public void setAngleDegrees(double angleDeg) {
    if (angleDeg < kSoftMinTurretAngle) {
      angleDeg = kSoftMinTurretAngle;
    }
    if (angleDeg > kSoftMaxTurretAngle) {
      angleDeg = kSoftMaxTurretAngle;
    }

    m_encoder.setPosition(Constants.turretConstants.turretGearRatio * angleDeg / 360);
  }

  /**
   * setAngleRadians - turn turret to a given angle relative to robot
   * 
   * @param  angle in RADIANS
   */
  public void setAngleRadians(double angleRad) {
    setAngleDegrees(Math.toDegrees(angleRad));
  }

  /**
   * stop - stop the turret motor, disabling PID position control
   */
  public void stop() {
    setPercentOutput(0);
  }

   /**
   * setPercentOutput()  - override Turret Motor with percent output
   * 
   * @param percent, percent motor output -1.0 to 1.0
   */
  public void setPercentOutput(double percent) {
    turretDrive.set(-percent);
  }

  /**
   * getAngleDegrees()  return current turret angle in degrees
   * 
   * @return angle in degrees
   */
  public double getAngleDegrees() {
    return (m_encoder.getPosition() * 360 / Constants.turretConstants.turretGearRatio);
  }

  /**
   * getAngleRadians()  return current turret angle in RADIANS
   * 
   * @return angle in radians
   */
  public double getAngleRadians() {
    return(Math.toRadians(getAngleDegrees()));
  }

  @Override
  public void periodic() {

    double pos = m_encoder.getPosition();
    // SmartDashboard.putNumber("Turret Pos", pos);
    SmartDashboard.putNumber("Turret Angle", pos * 360 / Constants.turretConstants.turretGearRatio);
  }
}
