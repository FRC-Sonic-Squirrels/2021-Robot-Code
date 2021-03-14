/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Relay;
import static frc.robot.Constants.canId;


public class intakeSubsystem extends SubsystemBase {

  private WPI_TalonFX m_intake = new WPI_TalonFX(canId.canId18_intake);
  private TalonFXSensorCollection m_encoder;
  private Relay intakeRelay = new Relay(0);

  private driveSubsystem m_drive;
  private double circOfIntake_meters = (1.4725 * Math.PI) * 0.0254;
  private double minIntakeRPM = 2500;
  private double maxIntakeRPM = 6000;
  private double intakeRPM = 0.0;
  private boolean dynamicMode = false;
  private static int kPIDLoopIdx = 0;
  private static int kTimeoutMs = 30;

  public intakeSubsystem(driveSubsystem drive) {
    m_drive = drive;
    
    m_intake.configFactoryDefault();
    m_intake.setInverted(true);
    m_intake.setNeutralMode(NeutralMode.Coast); 
    m_intake.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    m_intake.config_kP(kPIDLoopIdx, 0.2);
    m_intake.config_kI(kPIDLoopIdx, 0.0005);
    m_intake.config_kD(kPIDLoopIdx, 0);
    m_intake.config_kF(kPIDLoopIdx, 0.05);
    m_intake.config_IntegralZone(kPIDLoopIdx, 100);

    m_encoder = m_intake.getSensorCollection();
    //deployIntake();
    
    intakeRPM = 0.0;
    SmartDashboard.putNumber("Set Intake Motor RPM", 0.0);
    SmartDashboard.putNumber("Intake Motor RPM", 0.0);
    SmartDashboard.putNumber("Robot Speed m per s", 0.0);
  }

  @Override
  public void periodic() {
    boolean dynamic = SmartDashboard.getBoolean("Dynamic Mode", dynamicMode);
    dynamicMode = dynamic;
    if (dynamicMode) {
      setIntakeToSpeed();
    } else {
      double ir = SmartDashboard.getNumber("Set Intake Motor RPM", 0.0);
      if (ir != intakeRPM) {
        intakeRPM = ir;
        setIntakeMotorRPM(intakeRPM);
      }
    }
    SmartDashboard.putNumber("Intake Motor RPM", - m_encoder.getIntegratedSensorVelocity() * 600 / 2048);
    SmartDashboard.putNumber("Robot Speed m per s", (m_drive.getLeftVelocity() + m_drive.getRightVelocity()) / 2.0);
  }
  
  /**
   * Sets Intake Percent output to designated Percent
   */
  public void setIntakePercentOutput(double percent) {
    m_intake.set(ControlMode.PercentOutput, percent);
  }

  /**
   * Sets Intake RPM to designated RPM
   */
  public void setIntakeMotorRPM(double desiredRPM) {
    intakeRPM = desiredRPM;
    m_intake.set(ControlMode.Velocity, desiredRPM * 2048 / 600.0);
  }

  /**
   * Takes the speed at which the Robot moves and makes the Intake move a relative
   * speed
   */
  public void setIntakeToSpeed() {
    double robotMetersPerSec = (m_drive.getLeftVelocity() + m_drive.getRightVelocity()) / 2.0;
    double intakeRotationsPerSec = robotMetersPerSec / circOfIntake_meters;
    // Going Twice as Fast as the Robot Speed
    double _intakeRPM = intakeRotationsPerSec * 60 * 4.0;
    double desiredMotorRPM = _intakeRPM * Constants.intakeConstants.intakeGearRatio;
    if (desiredMotorRPM < minIntakeRPM) {
      desiredMotorRPM = minIntakeRPM;
    } else if (desiredMotorRPM > maxIntakeRPM) {
      desiredMotorRPM = maxIntakeRPM;
    }

    setIntakeMotorRPM(desiredMotorRPM);
  }

  /**
   * Sets Intake Speed to Match double robot speed at all times
   */
  public void setDynamicSpeed(boolean dynamic) {
    dynamicMode = dynamic;
  }

 /**
   * Toggle on/off intake dynamic speed mode
   */
  public void toggleDynamicMode() {
    dynamicMode = ! dynamicMode;
  }

  /**
   * release and deploy the intake
   */
  public void deployIntake() {
    intakeRelay.set(Relay.Value.kReverse);
  }

  /**
   * reset solenoids
   */
  public void retractIntake() {
    // There is no retract on this robot. Just reset solenoids
    intakeRelay.set(Relay.Value.kForward);
  }

  public void stop() {
    setDynamicSpeed(false);
    m_intake.setVoltage(0.0);
    setIntakeMotorRPM(0.0);
    intakeRelay.set(Relay.Value.kForward);
  }

    /**
   * seesPowerCell - Checks if we can see the powercell and returns a boolean based on that
   * 
   * @return Boolean
   */
  public boolean seesPowerCell(){
    return RobotContainer.m_limelightPowerCell.getTX() != 0.0;
  }

}
