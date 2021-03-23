/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.team2930.lib.util.linearInterpolator;

import frc.robot.Constants;
import frc.robot.RobotContainer;


public class shooterSubsystem extends SubsystemBase {

  private WPI_TalonFX talon_shooter1;
  private WPI_TalonFX talon_shooter2;

  private TalonFXSensorCollection m_encoder;
  private double kMaxOutput, kMinOutput;
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  private linearInterpolator m_lt_angle;
  private linearInterpolator m_lt_feet;
  private int m_idleRPM = 2000;
  private double m_currentRPM = 0;
  private double m_error = 0;
  private double m_max_RPM_error = 15;
  private final double RPMtoTicks = 2048 / 600;

  // lower number here, slows the rate of change and decreases the power spike 
  private double m_rate_RPMpersecond = 2500;
  private SlewRateLimiter m_rateLimiter;

  // based on the reported limelight angle

  private double shooterDistances[][] = {
    {4.0, 3700},  // 4 feet 
    {5.0, 3950},  // 5 feet
    {6.6, 3900},  // 6.6 Feet
    {11.0, 5000}, // 11 feet
    {15.0, 5400}, // 15 feet
    {16.2, 5500},
    {20.0, 6000},  // 20 feet
    {23.3, 6000},
    {25.0, 6200}
  };

  /**
   * shooterSubsystem() - constructor for shooterSubsytem class
   */
  public shooterSubsystem() {

    talon_shooter1 = new WPI_TalonFX(Constants.shooterConstants.shooter1);
    talon_shooter2 = new WPI_TalonFX(Constants.shooterConstants.shooter2);

    talon_shooter1.configFactoryDefault();
    talon_shooter2.configFactoryDefault();

    // set min time to go from neutral to full power
    // NOTE: closedloop ramp rate interacts poorly with closed loop control sometimes.
    // talon_shooter1.setClosedLoopRampRate(0.5);
    // talon_shooter2.setClosedLoopRampRate(0.5);
    
    // Set coast mode
    talon_shooter1.setNeutralMode(NeutralMode.Coast);
    talon_shooter2.setNeutralMode(NeutralMode.Coast);
    
    talon_shooter1.setInverted(false);

    talon_shooter2.follow(talon_shooter1);
    talon_shooter2.follow(talon_shooter1, FollowerType.PercentOutput);
    // always spin opposite of the lead motor
    talon_shooter2.setInverted(InvertType.OpposeMaster);
    
    m_encoder = talon_shooter1.getSensorCollection();

    kMaxOutput = 1.0; 
    kMinOutput = -0.05;
    talon_shooter1.configPeakOutputForward(kMaxOutput);
    talon_shooter1.configPeakOutputReverse(kMinOutput);

    //setShooterPID(0.12, 0.0005, 0.0, 0.048, 100);

    setShooterPID(0.14, 0.0005, 0.0, 0.048, 300);

    // Build the linear Interpolator
    m_lt_feet = new linearInterpolator(shooterDistances);

    m_desiredRPM = 0;
    
    /* Config sensor used for Primary PID [Velocity] */
    talon_shooter1.set(ControlMode.Velocity, 0);
    m_rateLimiter = new SlewRateLimiter(m_rate_RPMpersecond, m_desiredRPM);

    SmartDashboard.putNumber("RPM set point", m_desiredRPM);
    //SmartDashboard.putNumber("RPM", 0);
    //SmartDashboard.putNumber("RPM error", 0);
    //SmartDashboard.putNumber("Shooter Voltage", 0.0);

  }

  /**
   * periodic() - this function runs once every robot scheduler cycle.
   */
  @Override
  public void periodic() {

    m_currentRPM = m_encoder.getIntegratedSensorVelocity() / RPMtoTicks;
    m_error = m_currentRPM - m_desiredRPM;

    //if (Math.abs(m_error) < m_max_RPM_error) {
    if (((m_error >= 0) && (m_error < 50)) ||
        ((m_error < 0) && (m_error > -m_max_RPM_error))) {
      m_atSpeed = true;
    }
    else {
      m_atSpeed = false;  
    }

    double setPoint = m_rateLimiter.calculate(m_desiredRPM);
    if (m_desiredRPM < setPoint) {
      // we don't rate reduce slowing the robot
      setPoint = m_desiredRPM;
    }


    talon_shooter1.set(ControlMode.Velocity, setPoint * RPMtoTicks);

    SmartDashboard.putNumber("RPM", m_currentRPM);
    SmartDashboard.putNumber("RPM set point", setPoint);
    SmartDashboard.putNumber("RPM error", m_error);
    SmartDashboard.putBoolean("isAtSpeed", m_atSpeed);
    //SmartDashboard.putNumber("Shooter Voltage", talon_shooter1.getMotorOutputVoltage());
  }

  /**
    * add to the current shooter RPM
    */
  public void addToShooterRPM(double deltaRPM) {
    setShooterRPM(m_desiredRPM + deltaRPM);
  }

  /**
   * setShooterRPM - set desired flywheel RPM
   * 
   * @param desiredRPM
   */
  public void setShooterRPM (double desiredRPM) {
    m_desiredRPM = desiredRPM;
    isAtSpeed();

    if (m_desiredRPM <= 0) {
      m_desiredRPM = 0;
      setPercentOutput(0.0);

      // zero the RPM change rate limit. No rate limit to stop the flywheel
      m_rateLimiter = new SlewRateLimiter(m_rate_RPMpersecond, m_desiredRPM);
    }
  }

  /**
   * setShooterPID()   set flywheel PID parameters
   * 
   * @param kP
   * @param kI
   * @param kD
   * @param kF, feed forward constant
   * @param iZone, need to be this close to target to activate I
   */
  public void setShooterPID (double P, double I, double D, double F, double iZ) {
    talon_shooter1.config_kP(0, P);
    talon_shooter1.config_kI(0, I);
    talon_shooter1.config_kD(0, D);
    talon_shooter1.config_kF(0, F);
    talon_shooter1.config_IntegralZone(0, iZ);
  }

  /**
   * setPercentOutput() - override flywheel motor with percent output
   * 
   * @param percent, percent motor output -1.0 to 1.0
   */
  public void setPercentOutput(double percent) {
    talon_shooter1.set(ControlMode.PercentOutput, percent);
  }

  /**
   * isAtSpeed() - check if flywheel is at the desired RPM
   * 
   * @return true if at correct speed, else false
   */
  public boolean isAtSpeed() {
    m_error = m_currentRPM - m_desiredRPM;

    if (((m_error >= 0) && (m_error < 50)) ||
        ((m_error < 0) && (m_error > -m_max_RPM_error))) {
      m_atSpeed = true;
    }
    else {
      m_atSpeed = false;  
    }

    return m_atSpeed;
  }

  /**
   * getRPMforTY() - return RPM based on limelight TY value
   * 
   * @param TY limelight TY value
   * @return RPM for flywheel
   */
  public double getRPMforTY(double TY) {
    return m_lt_angle.getInterpolatedValue(TY);
  }

  /**
   * getRPMforDistanceFeet() - return RPM based on distance to target in FEET
   * 
   * @param distanceFeet distance in FEET to goal
   * @return RPM for flywheel
   */
  public double getRPMforDistanceFeet(double distanceFeet) {
    return m_lt_feet.getInterpolatedValue(distanceFeet);
  }

  /**
   * getRPMforDistanceFeet() - return RPM based on distance to target in METERS
   * 
   * @param distanceFeet distance in METERS to goal
   * @return RPM for flywheel
   */
  public double getRPMforDistanceMeter(double distanceMeters) {
    return getRPMforDistanceFeet(distanceMeters * 3.28084);
  }

  /**
   * getSetpoint() - return current target RPM
   */
  public double getSetPoint() {
    return m_desiredRPM;
  }

  /**
   * idle()  - run the flywheel at pre-determined idle speed
   */
  public void idle() {
    setShooterRPM(m_idleRPM);   
  }

  public void stop() {
    m_desiredRPM = 0;
    setPercentOutput(0.0);

    // zero the RPM change rate limit. No rate limit to stop the flywheel
    m_rateLimiter = new SlewRateLimiter(m_rate_RPMpersecond, m_desiredRPM);
  }
}