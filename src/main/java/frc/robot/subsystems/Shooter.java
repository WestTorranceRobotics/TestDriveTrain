// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

CANSparkMax shootLeader;
CANSparkMax shootFollower;

double kP;
double kI;
double kD;

  /** Creates a new Shooter. */
  public Shooter() {

CANSparkMax shootLeader = new CANSparkMax(RobotMap.ShooterMap.shootLeaderCANID, MotorType.kBrushless);
CANSparkMax shooterFollower = new CANSparkMax(RobotMap.ShooterMap.shootFollowerCANID, MotorType.kBrushless);

shootLeader.setInverted(true);
shootFollower.setInverted(true);

shootFollower.follow(shootLeader);

PIDController pidController = new PIDController(kP, kI, kD);
  }

  public void shootBall(double speed){
shootLeader.set(speed);
  }

  public void reverseBall(double reverse){
shootLeader.set(-reverse);
  }

  public void stopShooter(){
shootLeader.stopMotor();
  }

  public double getShooterVelocity(){
return shootLeader.getEncoder().getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
