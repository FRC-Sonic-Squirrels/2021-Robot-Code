/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class blinkinSubsystem extends SubsystemBase {

  /*
   * Rev Robotics Blinkin takes a PWM signal from 1000-2000us This is identical to
   * a SparkMax motor. -1 corresponds to 1000us 0 corresponds to 1500us +1
   * corresponds to 2000us
   */
  private static Spark m_blinkin = null;

  /**
   * Creates a new Blinkin LED controller.
   * 
   * @param pwmPort The PWM port the Blinkin is connected to.
   */
  public blinkinSubsystem(int pwmPort) {
    m_blinkin = new Spark(pwmPort);
    solid_orange();
  }

  /*
   * Set the color and blink pattern of the LED strip.
   * 
   * Consult the Rev Robotics Blinkin manual Table 5 for a mapping of values to patterns.
   * 
   * @param val The LED blink color and patern value [-1,1]
   * 
   */ 
  public void set(double val) {
    if ((val >= -1.0) && (val <= 1.0)) {
      m_blinkin.set(val);
    }
  }

  public void rainbow() {
    set(-0.99);
  }

  public void solid_orange() {
    set(0.65);
  }

  public void solid_pink() {
    set(0.57);
  }

  public void solid_red() {
    set(0.61);
  }

  public void solid_yellow() {
    set(0.69);
  }

  public void solid_green_lime() {
    set(0.73);
  }

  public void solid_green() {
    set(0.77);
  }

  public void solid_blue_aqua() {
    set(0.81);
  }

  public void solid_blue_dark() {
    set(0.85);
  }

  public void solid_blue() {
    set(0.87);
  }

  public void solid_violet() {
    set(0.91);
  }

  public void solid_white() {
    set(0.93);
  }

  public void strobe_red() {
    set(-0.11);
  }

  public void strobe_blue() {
    set(-0.09);
  }

  public void strobe_white() {
    set(-0.05);
  }

  public void strobe_gold() {
    set(-0.07);
  }


}