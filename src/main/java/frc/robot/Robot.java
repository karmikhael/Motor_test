// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.GenericHID;



import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kJoystickPort = 0;

  private final SparkMax v_motor;
  //private final Joystick m_joystick;
  private final SparkMax bm_motor;

  private final SparkMax n_motor;

  private final TalonFX j_motorFx;

  private final XboxController the_xbox;

  
  /** Called once at the beginning of the robot program. */
  public Robot() {
    v_motor = new SparkMax(45, SparkLowLevel.MotorType.kBrushless);
    bm_motor = new SparkMax(3, SparkLowLevel.MotorType.kBrushless);
    n_motor = new SparkMax(46, SparkLowLevel.MotorType.kBrushless);

    //m_joystick = new Joystick(kJoystickPort);
    the_xbox = new XboxController(kJoystickPort);
    j_motorFx = new TalonFX(25);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {

  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    if(the_xbox.getAButtonPressed()){
      toggleTalon(j_motorFx, 0.03);
    }
    if(the_xbox.getBButtonPressed()){
      toggle(v_motor, 0.02);
    }
    if(the_xbox.getYButtonPressed()){
      toggle(n_motor, 0.03);
    }
    if(the_xbox.getXButtonPressed()){
      toggle(bm_motor, 0.02);
    }
  }

  public void toggle(MotorController rc, double speed){
    if (rc.get() == 0){
      rc.set(speed);
    
    }
    else {
      rc.set(0);
    }

  }
  public void toggleTalon(TalonFX rc, double speedFx){
    if (rc.get() == 0){
      rc.set(speedFx);
    
    }
    else {
      rc.set(0);
    }

  }

}
