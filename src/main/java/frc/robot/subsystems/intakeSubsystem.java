/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
import edu.wpi.first.wpilibj.Solenoid;

public class intakeSubsystem extends SubsystemBase {

  private WPI_VictorSPX intakeC = new WPI_VictorSPX(intakeConstants.intakeMotor);
  public static Solenoid intakeSolenoid = new Solenoid(intakeConstants.intakeSolenoid);
    
  public intakeSubsystem() {
  }

  @Override
  public void periodic() {
  }
  
  public void setIntakePercentOutput(double percent) {
    intakeC.set(ControlMode.PercentOutput, percent);
  }

  public void deployIntake() {
    intakeSolenoid.set(true);
    setIntakePercentOutput(-0.8);
  }

  public void retractIntake() {
    intakeSolenoid.set(false);
    setIntakePercentOutput(0);
  }
}
