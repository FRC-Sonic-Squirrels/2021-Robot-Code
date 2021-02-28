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

public class turretSubsystem extends SubsystemBase {

  private TalonSRX turretDrive = new TalonSRX(turret);

  public turretSubsystem() {
    turretDrive.configFactoryDefault();
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
    double pos = turretDrive.getSelectedSensorPosition();

    SmartDashboard.putNumber("Turret Pos", pos);
    SmartDashboard.putNumber("Turret Angle", pos * kDegreesPerTick);

  }
}
