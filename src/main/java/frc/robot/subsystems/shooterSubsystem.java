/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.shooterConstants.shooter1;
import static frc.robot.Constants.shooterConstants.shooter2;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team2930.lib.util.linearInterpolator;

import frc.robot.Constants;
import frc.robot.RobotContainer;


public class shooterSubsystem extends SubsystemBase {

  private CANSparkMax neo_shooter1 = new CANSparkMax(shooter1, MotorType.kBrushless);
  private CANSparkMax neo_shooter2 = new CANSparkMax(shooter2, MotorType.kBrushless);
  private Solenoid hood = new Solenoid(Constants.shooterConstants.shooterHood);
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private double kMaxOutput, kMinOutput;
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  private linearInterpolator m_lt_angle;
  private linearInterpolator m_lt_feet;
  private linearInterpolator m_lt_hoodDownAngle;
  private linearInterpolator m_lt_hoodUpAngle;
  private linearInterpolator m_lt_hoodDownFeet;
  private linearInterpolator m_lt_hoodUpFeet;
  private int m_idleRPM = 1500;
  private double m_currentRPM = 0;
  private double m_error = 0;
  private double m_max_RPM_error = 15;

  private double m_rate_RPMpersecond = 2000;
  private SlewRateLimiter m_rateLimiter;

  // based on the reported limelight angle
  private double hoodDownAngle[][] = {
    {24.7, 2750}, // 4 feet
    {12.3, 2850}, // 7 feet
    {1.0,  2500}, // 10 feet
    {-2.85, 2700}, // 12 feet
    {-5.7,  2800} // 13 feet
  };

  private double hoodDownFeet[][] = {
    {4.0, 2600},  // 4 feet  2750
    {7.0, 2500},  // 7 feet
    {10.0, 2650}, // 10 feet
    {12.0, 2700}, // 12 feet
    {13.0, 2800}  // 13 feet
  };

  // based on the reported limelight angle
  private double hoodUpAngle[][] = {
    {5, 3000}, // 9 feet
    {-0.11, 3100}, // 10 ft
    {-4.3, 3250}, // 13 feet
    {-9.85, 3300}, // 17 feet
    {-11, 3300}, // 17+ feet
    {-16, 3600} // 25 feet
  };

  // RPM based on distance in feet from target
  private double hoodUpFeet[][] = {
    {9,  3000},
    {10, 3100},
    {13, 3250},
    {17, 3300},
    {18, 3300},
    {25, 3600}
  };



  /**
   * shooterSubsystem() - constructor for shooterSubsytem class
   */
  public shooterSubsystem() {

    neo_shooter1.restoreFactoryDefaults();
    neo_shooter2.restoreFactoryDefaults();

    // set min time to go from neutral to full power
    // NOTE: closedloop ramp rate interacts poorly with closed loop control sometimes.
    // neo_shooter1.setClosedLoopRampRate(0.5);
    // neo_shooter2.setClosedLoopRampRate(0.5);
    
    // Set coast mode
    neo_shooter1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    neo_shooter2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    
    neo_shooter1.setInverted(true);

    neo_shooter2.follow(neo_shooter1, true);
    m_pidController = neo_shooter1.getPIDController();
    m_encoder = neo_shooter1.getEncoder(EncoderType.kHallSensor, 4096);
    
    kMaxOutput = 1.0; 
    kMinOutput = -0.0;
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    setShooterPID(0.0009, 0.000001, 0.0, 0.0002, 200);

    // Build the linear Interpolators just once each.
    m_lt_hoodUpAngle = new linearInterpolator(hoodUpAngle);
    m_lt_hoodDownAngle = new linearInterpolator(hoodDownAngle);
    m_lt_hoodUpFeet = new linearInterpolator(hoodUpFeet);
    m_lt_hoodDownFeet = new linearInterpolator(hoodDownFeet);

    // pick a default, so that it is never undefined
    m_lt_angle = m_lt_hoodDownAngle;
    m_lt_feet = m_lt_hoodDownFeet;

    m_desiredRPM = 0;
    m_pidController.setReference(0, ControlType.kVelocity);
    m_rateLimiter = new SlewRateLimiter(m_rate_RPMpersecond, m_desiredRPM);

    SmartDashboard.putNumber("RPM set point", m_desiredRPM);
    SmartDashboard.putNumber("RPM", 0);
    SmartDashboard.putNumber("RPM error", 0);
    SmartDashboard.putNumber("Shooter Voltage", 0.0);

  }

  /**
   * periodic() - this function runs once every robot scheduler cycle.
   */
  @Override
  public void periodic() {

    m_currentRPM = m_encoder.getVelocity();
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

    m_pidController.setReference(setPoint, ControlType.kVelocity);
 

    SmartDashboard.putNumber("RPM", m_currentRPM);
    SmartDashboard.putNumber("RPM set point", setPoint);
    SmartDashboard.putNumber("RPM error", m_error);
    SmartDashboard.putBoolean("isAtSpeed", m_atSpeed);
    SmartDashboard.putNumber("Shooter Voltage", neo_shooter1.getAppliedOutput());
  }

  /**
   * setShooterRPM - set desired flywheel RPM
   * 
   * @param desiredRPM
   */
  public void setShooterRPM (double desiredRPM) {
    m_desiredRPM = desiredRPM;
    isAtSpeed();

    if (m_desiredRPM == 0) {
      setPercentOutput(0.0);

      // zero the RPM change rate limit. No rate limit to stop the flywheel
      m_rateLimiter = new SlewRateLimiter(m_rate_RPMpersecond, m_desiredRPM);
    }
  }

  /**
   * deployHood() - raise shooter hood
   */
  public void deployHood() {
    RobotContainer.m_limelight.setPipeline(4);
    m_lt_angle = m_lt_hoodUpAngle;
    m_lt_feet = m_lt_hoodUpFeet;
    hood.set(true);
  }

  /**
   * retractHood() - lower the shooter hood
   */
  public void retractHood() {
    RobotContainer.m_limelight.setPipeline(4);
    m_lt_angle = m_lt_hoodDownAngle;
    m_lt_feet = m_lt_hoodDownFeet;
    hood.set(false);
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
    m_pidController.setP(P);
    m_pidController.setI(I);
    m_pidController.setD(D);
    m_pidController.setFF(F);
    m_pidController.setIZone(iZ);
  }

  /**
   * setPercentOutput() - override flywheel motor with percent output
   * 
   * @param percent, percent motor output -1.0 to 1.0
   */
  public void setPercentOutput(double percent) {
    neo_shooter1.set(percent);
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
