/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
*/

import com.team2930.lib.util.linearInterpolator;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.shooterConstants;

public class shooterSubsystem extends SubsystemBase {


  //TODO: Change these to Falcons
  private CANSparkMax neo_shooter1 = new CANSparkMax(Constants.shooterConstants.shooter1, MotorType.kBrushless);
  private CANSparkMax neo_shooter2 = new CANSparkMax(Constants.shooterConstants.shooter2, MotorType.kBrushless);


  //TODO: Remove this Solenoid
  private Solenoid hood = new Solenoid(Constants.shooterConstants.shooterHood);
  private CANPIDController m_pidController;
  private CANEncoder m_encoder;
  private double kMaxOutput, kMinOutput;
  private double m_desiredRPM = 0;
  private boolean m_atSpeed = false;
  private long m_initialTime = 0;
  private linearInterpolator m_lt;
  private linearInterpolator m_lt_hoodDownP;
  private linearInterpolator m_lt_hoodUpP;
  private linearInterpolator m_lt_hoodDownC;
  private linearInterpolator m_lt_hoodUpC;
  private int m_idleRPM = 1500;
  
  private double hoodDownP[][] = {
    {15.4, 2600}, // 4.5 feet
    {3, 2650}, // 7 feet
    {-7.2, 2750}, // 10 feet
    {-12.2, 2900} // 12 feet
  };
  private double hoodUpP[][] = {
    {8, 3900}, // 9 feet
    {-0.1, 3550}, // 13 feet
    {-5, 3600}, // 17 feet
    {-8.5, 3800}, // 21 feet
    {-11, 4100} // 25 feet
  };

  private double hoodDownC[][] = {
    {24.7, 2750}, // 4 feet
    {12.3, 2850}, // 7 feet
    {2.75, 2950}, // 10 feet
    {-2.85, 3050} // 12 feet
  };
  private double hoodUpC[][] = {
    {5, 3750}, // 9 feet
    {-4.3, 3500}, // 13 feet
    {-9.85, 3600}, // 17 feet
    {-13.5, 3750}, // 21 feet
    {-16, 4300} // 25 feet
  };

  public shooterSubsystem() {
    neo_shooter1.restoreFactoryDefaults();
    neo_shooter2.restoreFactoryDefaults();


    // set min time to go from neutral to full power
    neo_shooter1.setClosedLoopRampRate(0.5);
    neo_shooter2.setClosedLoopRampRate(0.5);
    
    // Set coast mode
    neo_shooter1.setIdleMode(CANSparkMax.IdleMode.kCoast);
    neo_shooter2.setIdleMode(CANSparkMax.IdleMode.kCoast);
    

    neo_shooter2.follow(neo_shooter1, true);
    m_pidController = neo_shooter1.getPIDController();
    m_encoder = neo_shooter1.getEncoder(EncoderType.kHallSensor, 4096);
    
    kMaxOutput = 0.9; 
    kMinOutput = -0.0;
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // Build the linear Interpolators just once each.
    m_lt_hoodUpC = new linearInterpolator(hoodUpC);
    m_lt_hoodDownC = new linearInterpolator(hoodDownC);
    m_lt_hoodUpP = new linearInterpolator(hoodUpP);
    m_lt_hoodDownP = new linearInterpolator(hoodDownP);

    // pick a default, so that it is never undefined
    m_lt = m_lt_hoodDownC;

    SmartDashboard.putNumber("ShooterRPM", m_desiredRPM);
    SmartDashboard.putNumber("RPM_Error", 0);
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("ActualShooterRPM", (int) m_encoder.getVelocity());

    //
    // Manual RPM control from Smartdashboard
    //
    double rpm = SmartDashboard.getNumber("ShooterRPM", -1);
    if (rpm != -1) {
      if (rpm == 0.0) {
        // spin down, don't use PID (and power) to stop
        stop();
      }
      else if (m_desiredRPM != rpm ) {
        setShooterRPM(rpm);
        m_initialTime = System.nanoTime();
        m_atSpeed = false;
      }
    }

    //
    // Track how long it takes to reach desired RPM, set m_asSpeed
    // 
    if (isAtSpeed()) {
      if (!m_atSpeed) {
        SmartDashboard.putNumber("Time2RPM", System.nanoTime() - m_initialTime);
      }
      m_atSpeed = true;
    }
    else {
      m_atSpeed = false;
    }

    SmartDashboard.putBoolean("isAtSpeed", m_atSpeed);
  }

  /**
   * setShooterRPM - set desired flywheel RPM
   * 
   * @param desiredRPM
   */
  public void setShooterRPM (double desiredRPM) {
    m_desiredRPM = desiredRPM;
    m_initialTime = System.nanoTime();
    m_atSpeed = false;
    if (m_desiredRPM <= m_idleRPM) {
      m_pidController.setOutputRange(0, kMaxOutput);
      setShooterPID(0.0003, 0, 0, 0.00018, 0);
    }
    else {
      m_pidController.setOutputRange(kMinOutput, kMaxOutput);
      // Old values: setShooterPID(0.0005, 0.00000015, 0, 0.0002, 600);
      setShooterPID(0.0004, 0.000001, 0.0, 0.0002, 200);
    }
    m_pidController.setReference(desiredRPM, ControlType.kVelocity);
    SmartDashboard.putNumber("ShooterRPM", m_desiredRPM);
  }
  public void testMode(){
    m_desiredRPM = SmartDashboard.getNumber("DesiredShooterRPM", 0);
    System.out.println("Shooter desired RPM: "  + m_desiredRPM);
    m_pidController.setReference(m_desiredRPM, ControlType.kVelocity);
    System.out.println("Activating Test Mode");
  }

  public void deployHood() {
    RobotContainer.m_limelight.setPipeline(4);
    if (Robot.isCompBot == true) {
      m_lt = m_lt_hoodUpC;
    }
    else {
      m_lt = m_lt_hoodUpP;
    }
    hood.set(true);
  }

  public void retractHood() {
    RobotContainer.m_limelight.setPipeline(4);
    if (Robot.isCompBot == true) {
      m_lt = m_lt_hoodDownC;
    }
    else {
      m_lt = m_lt_hoodDownP;
    }
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

  //Current limiting on the fly switching removed due to the SparkMAX API not supporting that sort of switch.
  public void setPercentOutput(double percent) {
    neo_shooter1.set(percent);
  }

  /**
   * isAtSpeed() - check if flywheel is at the desired RPM
   * 
   * @return true if at correct speed, else false
   */
  public boolean isAtSpeed(){
    double error = m_desiredRPM - m_encoder.getVelocity();
    SmartDashboard.putNumber("RPM_Error", error);
    
    if (Math.abs(error) < 75) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * getRPMforTY() - return RPM based on limelight TY value
   * 
   * @param TY limelight TY value
   * @return RPM for flywheel
   */
  public double getRPMforTY(double TY) {
    return m_lt.getInterpolatedValue(TY);
  }

  /**
   * getRPMforDistanceFeet() - return RPM based on distance to target in FEET
   * 
   * @param distanceFeet distance in FEET to goal
   * @return RPM for flywheel
   */
  public double getRPMforDistanceFeet(double distanceFeet) {
    return m_lt.getInterpolatedValue(distanceFeet);
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
   * idle()  - run the flywheel at pre-determined idle speed
   */
  public void idle() {
    setShooterRPM(m_idleRPM);   
  }


  public void stop() {
    m_desiredRPM = 0;
    setPercentOutput(0.0);
  }
}