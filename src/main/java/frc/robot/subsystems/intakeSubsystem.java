/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Relay;

public class intakeSubsystem extends SubsystemBase {

  private WPI_TalonFX m_intake = new WPI_TalonFX(Constants.intakeConstants.intakeMotor);
  private Relay intakeRelay = new Relay(0);

  private driveSubsystem m_drive;
  private double circOfIntake_meters = (1.4725 * Math.PI) * 0.0254;
  private double minIntakeRPM = 1000;
  private double maxIntakeRPM = 6000;
  private double intakeRPM = 0.0;
  private boolean dynamicMode = true;

  public intakeSubsystem(driveSubsystem drive) {
    m_intake.configFactoryDefault();
    m_intake.setInverted(true);
    m_drive = drive;
    
    intakeRelay.set(Relay.Value.kForward);
    
  }

  @Override
  public void periodic() {
    boolean dynamic = SmartDashboard.getBoolean("Dynamic Mode", dynamicMode);
    dynamicMode = dynamic;
    if (dynamicMode) {
      setIntakeToSpeed();
    } else {
      double ir = SmartDashboard.getNumber("Intake Motor RPM", 0.0);
      if (ir != intakeRPM) {
        intakeRPM = ir;
        setIntakeRPM(intakeRPM);
      }
    }
    SmartDashboard.putNumber("Intake Motor RPM", m_intake.getSensorCollection().getIntegratedSensorVelocity() * 600 / 2048);
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
  public void setIntakeRPM(double rpm) {
    m_intake.set(ControlMode.Velocity, rpm * 2048 / 600 * Constants.intakeConstants.intakeGearRatio);
  }

  /**
   * Takes the speed at which the Robot moves and makes the Intake move a relative
   * speed
   */
  public void setIntakeToSpeed() {
    double robotMetersPerSec = m_drive.getLeftVelocity() + m_drive.getRightVelocity() / 2;
    double intakeRotationsPerSec = robotMetersPerSec / circOfIntake_meters;
    // Going Twice as Fast as the Robot Speed
    double intakeRPM = intakeRotationsPerSec * 60 * 2;
    double desiredMotorRPM = intakeRPM * Constants.intakeConstants.intakeGearRatio;
    if (desiredMotorRPM < minIntakeRPM) {
      setIntakeRPM(minIntakeRPM);
    } else if (desiredMotorRPM > maxIntakeRPM) {
      setIntakeRPM(maxIntakeRPM);
    } else {
      setIntakeRPM(desiredMotorRPM);
    }
  }

  /**
   * Changes Intake Speed to Match double robot speed at all times
   */
  public void enableDynamicSpeed(boolean dynamic) {
    dynamicMode = dynamic;
  }

  /**
   * release and deploy the intake
   */
public void deployIntake() {
  // TODO: retract solenoids
}

public void retractIntake() {
  // There is no retract on this robot. 
}

  public void stop(){
    enableDynamicSpeed(false);
    m_intake.setVoltage(0.0);
    setIntakeRPM(0.0);
    intakeRelay.set(Relay.Value.kReverse);
  }
}
