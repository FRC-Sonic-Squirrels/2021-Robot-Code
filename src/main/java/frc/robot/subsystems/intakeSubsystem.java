// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intakeSubsystem extends SubsystemBase {

  private WPI_TalonFX m_intake = new WPI_TalonFX(Constants.intakeConstants.intakeMotor);
  private driveSubsystem m_drive = new driveSubsystem();
  private double circOfIntake_meters = (2 * Math.PI) * 0.0254;
  private double minIntakeRPM = 500;
  private double maxIntakeRPM = 6000;
  private double intakeRPM = 0.0;
  private boolean dynamicMode = true;

  public intakeSubsystem() { 
    m_intake.configFactoryDefault();
    m_intake.setInverted(true);
  }

  @Override
  public void periodic() {
    boolean dynamic = SmartDashboard.getBoolean("Dynamic Mode", dynamicMode);
    dynamicMode = dynamic;
    if (dynamicMode){ 
      setIntakeToSpeed();
    }
    else {
      double ir = SmartDashboard.getNumber("Intake RPM", 0.0);
      if (ir != intakeRPM){
        intakeRPM = ir;
        setIntakeRPM(intakeRPM);
      }
    }
    SmartDashboard.putNumber("Intake RPM", m_intake.getSensorCollection().getIntegratedSensorVelocity() * 600 / 2048);
  }

  public void setIntakePercentOutput(double percent){
      m_intake.set(ControlMode.PercentOutput, percent);
  }

  public void setIntakeRPM(double rpm){
      m_intake.set(ControlMode.Velocity, rpm * 2048/600);
  }

  /**
   * Takes the speed at which the Robot moves and makes the Intake move a relative speed
   */
  public void setIntakeToSpeed(){
      double robotMetersPerSec = m_drive.getLeftVelocity() + m_drive.getRightVelocity() / 2;
      double intakeRotationsPerSec = robotMetersPerSec / circOfIntake_meters;
      //Going Twice as Fast as the Robot Speed
      double intakeRPM = intakeRotationsPerSec * 60 * 2;
      if(intakeRPM < minIntakeRPM){
        setIntakeRPM(minIntakeRPM);
      }
      else if (intakeRPM > maxIntakeRPM){
        setIntakeRPM(maxIntakeRPM);
      }
      else {
        setIntakeRPM(intakeRPM);
      }
  }

  public void enableDynamicSpeed(boolean dynamic){
      dynamicMode = dynamic;
  }
}
