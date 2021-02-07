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
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
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
  private DigitalInput limit = new DigitalInput(dio7_turretLimit);

  public turretSubsystem() {
    turretDrive.restoreFactoryDefaults();
    turretDrive.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, kIndex, kTimeout);

    turretDrive.setInverted(false);
    turretDrive.setSensorPhase(false);

    turretDrive.configForwardSoftLimitThreshold(12308, kTimeout);
    turretDrive.configReverseSoftLimitThreshold(-11629, kTimeout);

    turretDrive.configForwardSoftLimitEnable(true);
    turretDrive.configReverseSoftLimitEnable(true);

    turretDrive.configContinuousCurrentLimit(25);

    // zero the position. start position becomes center
    turretDrive.setSelectedSensorPosition(0, kIndex, kTimeout);

    // TODO: tune PIDF parameters (these are only a guess)
    turretDrive.configAllowableClosedloopError(0, kIndex, kTimeout);
    turretDrive.config_kF(kIndex, 0.38, kTimeout);
    turretDrive.config_kP(kIndex, 0.1, kTimeout);
    turretDrive.config_kI(kIndex, 0, kTimeout);
    turretDrive.config_kD(kIndex, 0, kTimeout);

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
    turretDrive.set(ControlMode.Position, 0);
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

  public void setPercentOutput(double percent) {
    turretDrive.set(ControlMode.PercentOutput, percent);
  }

  /**
   * getAngleDegrees()  return current turret angle in degrees
   * 
   * @return angle in degrees
   */
  public double getAngleDegrees() {
    return (turretDrive.getSelectedSensorPosition() * kDegreesPerTick);
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

    // TODO: research if we need to use a double here. 
    int pos = (int) turretDrive.getSelectedSensorPosition();

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