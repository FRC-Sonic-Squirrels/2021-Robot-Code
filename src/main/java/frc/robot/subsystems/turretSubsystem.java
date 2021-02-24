/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.turretConstants.kSoftMaxTurretAngle;
import static frc.robot.Constants.turretConstants.kSoftMinTurretAngle;
import static frc.robot.Constants.turretConstants.kDegreesPerTick;
import static frc.robot.Constants.turretConstants.kTimeout;
import static frc.robot.Constants.turretConstants.kIndex;
import static frc.robot.Constants.turretConstants.kMaxDegreesPerSecond;
import static frc.robot.Constants.turretConstants.kMaxDegreesPerSecondSquared;
import static frc.robot.Constants.turretConstants.turret;
import static frc.robot.Constants.digitalIOConstants.dio7_turretLimit;

public class turretSubsystem extends SubsystemBase {

  //Neo 550 is the Motor Controlling the Turret
  private CANSparkMax turretDrive = new CANSparkMax(turret, MotorType.kBrushless);
  //private TalonSRX turretDrive = new TalonSRX(turret);
  private CANEncoder m_encoder;
  private CANPIDController m_pidController;
  private DigitalInput limit = new DigitalInput(dio7_turretLimit);

  public turretSubsystem() {
    turretDrive.restoreFactoryDefaults();
    m_encoder = turretDrive.getEncoder();

    //Sets Turret Position as 0
    m_encoder.setPosition(0);

    turretDrive.configForwardSoftLimitThreshold(12308, kTimeout);
    turretDrive.configReverseSoftLimitThreshold(-11629, kTimeout);

    turretDrive.configForwardSoftLimitEnable(true);
    turretDrive.configReverseSoftLimitEnable(true);

    turretDrive.configContinuousCurrentLimit(25);

    // zero the position. start position becomes center
    turretDrive.setSelectedSensorPosition(0, kIndex, kTimeout);


     // Set PID coefficients. Currently default
     m_pidController.setP(0.2);
     m_pidController.setI(1e-4);
     m_pidController.setD(0);
     m_pidController.setFF(0);
     m_pidController.setIZone(100);
     m_pidController.setOutputRange(-1, 1);

    
     turretDrive.configAllowableClosedloopError(0, kIndex, kTimeout);

    // set Motion Magic max Cruise Velocity and max acceleration
    turretDrive.configMotionCruiseVelocity((int) (kMaxDegreesPerSecond / (kDegreesPerTick * 10)),
        kTimeout);
    turretDrive.configMotionAcceleration(
        (int) (kMaxDegreesPerSecondSquared / (kDegreesPerTick * 10)), kTimeout);

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

    m_encoder.setPosition();
    turretDrive.set(ControlMode.Position, angleDeg / kDegreesPerTick);
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
    turretDrive.set(percent);
  }

  /**
   * getAngleDegrees()  return current turret angle in degrees
   * 
   * @return angle in degrees
   */
  public double getAngleDegrees() {
    return (m_encoder.getPosition() * kDegreesPerTick);
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
    boolean turretLimit = !limit.get();

    double pos = m_encoder.getPosition();

    SmartDashboard.putBoolean("TurretLimit", turretLimit);
    SmartDashboard.putNumber("Turret Pos", pos);
    SmartDashboard.putNumber("Turret Angle", pos * kDegreesPerTick);

    if (turretLimit == true) {
      stop();
      if (pos < 0) {
        DriverStation.reportError("Min limit Reached on turret. motor stopped", false);
        // check angle and reset position to kSoftMinTurretAngle if off by more than 1 deg
        if (Math.abs(pos * kDegreesPerTick - kSoftMinTurretAngle) > 1.0) {
          // TODO: magnetic limits switch may be outside software min/max set accordingly
          turretDrive.setSelectedSensorPosition((int) (kSoftMinTurretAngle / kDegreesPerTick), kIndex, kTimeout);
        }
      }
      else {
        DriverStation.reportError("Max limit Reached on turret, motor stopped", false);
        // check angle and reset position to kSoftMaxTurretAngle if off by more than 1 deg
        if (Math.abs(pos * kDegreesPerTick - kSoftMaxTurretAngle) > 1.0) {
          // TODO: magnetic limits switch may be outside software min/max set accordingly
          turretDrive.setSelectedSensorPosition((int) (kSoftMaxTurretAngle / kDegreesPerTick), kIndex, kTimeout);
        }
      }
    }
  }
}