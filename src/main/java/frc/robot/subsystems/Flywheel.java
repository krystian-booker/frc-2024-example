// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanIds;
import frc.robot.Constants.MotorSetPoint;

public class Flywheel extends SubsystemBase {

  private final CANSparkMax motor = new CANSparkMax(CanIds.FLYWHEEL, MotorType.kBrushless);
  private SparkPIDController pid;
  private RelativeEncoder encoder;

  public Flywheel() {
    // Clear all previous settings on the motor
    motor.restoreFactoryDefaults();

    motor.setIdleMode(IdleMode.kCoast);
    motor.setInverted(false);

    pid = motor.getPIDController();
    encoder = motor.getEncoder();

    pid.setP(0.1);
    pid.setI(0);
    pid.setD(0);

    motor.burnFlash();
  }

  @Override
  public void periodic() {
  }

  public void fullSpeedShot() {
    setFlywheelVelocity(MotorSetPoint.FLYWHEEL_FULL_SPEED);
  }

  public void halfSpeedShot() {
    setFlywheelVelocity(MotorSetPoint.FLYWHEEL_HALF_SPEED);
  }

  public void stopShooter(){
    setFlywheelVelocity(0);
    motor.stopMotor();
  }

  private void setFlywheelVelocity(double velocity) {
    pid.setReference(velocity, ControlType.kVelocity);
  }

  private boolean isFlywheelAtSetpoint(){
    return true;
  }
}
