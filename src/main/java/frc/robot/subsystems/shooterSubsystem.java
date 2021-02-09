/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


import com.team2930.lib.util.linearInterpolator;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.shooterConstants;

public class shooterSubsystem extends SubsystemBase {


  private TalonFX talon_shooter1 = new TalonFX(Constants.shooterConstants.shooter1);
  private TalonFX talon_shooter2 = new TalonFX(Constants.shooterConstants.shooter2);


  private double kMaxOutput, kMinOutput;
  private double m_desiredRPM = 0;
  private double targetVelocity = 0;
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
    talon_shooter1.configFactoryDefault();
    talon_shooter1.configFactoryDefault();


    // set min time to go from neutral to full power
    talon_shooter1.configClosedloopRamp(0.5);
    talon_shooter1.configClosedloopRamp(0.5);
    
    // Set coast mode
    talon_shooter1.setNeutralMode(NeutralMode.Coast);
    talon_shooter2.setNeutralMode(NeutralMode.Coast);

    talon_shooter1.setInverted(true);
    
    talon_shooter2.follow(talon_shooter1);

    /* Config sensor used for Primary PID [Velocity] */
    talon_shooter1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);

    //m_pidController = neo_shooter1.getPIDController();
    
    //m_encoder = neo_shooter1.getEncoder(EncoderType.kHallSensor, 4096);
    
    kMaxOutput = 0.9; 
    kMinOutput = -0.0;
    talon_shooter1.configPeakOutputForward(kMaxOutput);
    talon_shooter1.configPeakOutputReverse(kMinOutput);

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

    SmartDashboard.putNumber("ActualShooterRPM", (int) (talon_shooter1.getSelectedSensorVelocity(0)));

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
    targetVelocity = m_desiredRPM *2048.0 / 600.0;
    m_initialTime = System.nanoTime();
    m_atSpeed = false;
    if (m_desiredRPM <= m_idleRPM) {
      
      talon_shooter1.configPeakOutputForward(kMaxOutput);
      talon_shooter1.configPeakOutputReverse(kMinOutput);
      setShooterPID(0.0003, 0, 0, 0.00018, 0);
    }
    else {
      talon_shooter1.configPeakOutputForward(kMaxOutput);
      talon_shooter1.configPeakOutputReverse(kMinOutput);
      // Old values: setShooterPID(0.0005, 0.00000015, 0, 0.0002, 600);
      //TODO: Test these values with new shooter
      setShooterPID(0.0004, 0.000001, 0.0, 0.0002, 200);
    }

    talon_shooter1.set(TalonFXControlMode.Velocity, targetVelocity);
    SmartDashboard.putNumber("ShooterRPM", m_desiredRPM);
  }
  public void testMode(){
    m_desiredRPM = SmartDashboard.getNumber("DesiredShooterRPM", 0);
    System.out.println("Shooter desired RPM: "  + m_desiredRPM);
    /**
			 * Convert Desired RPM to units / 100ms.
			 * 2048 Units/Rev * m_DesiredRPM / 600 100ms/min in either direction:
			 * velocity setpoint is in units/100ms
			 */
    double targetVelocity = m_desiredRPM *2048.0 / 600.0;
    talon_shooter1.set(TalonFXControlMode.Velocity, targetVelocity);
    System.out.println("Activating Test Mode");
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

  //Current limiting on the fly switching removed due to the SparkMAX API not supporting that sort of switch.
  public void setPercentOutput(double percent) {
    //TODO: Set the Percentage output to percent
    talon_shooter1.set(TalonFXControlMode.PercentOutput, percent);
  }

  /**
   * isAtSpeed() - check if flywheel is at the desired RPM
   * 
   * @return true if at correct speed, else false
   */
  public boolean isAtSpeed(){
    double error = m_desiredRPM - (talon_shooter1.getSelectedSensorVelocity(0));
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