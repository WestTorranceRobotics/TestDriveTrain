// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class runToRPM extends CommandBase {

private Shooter shooter;
private double rpm;

public boolean isFinished = false;

  /** Creates a new runToRPM. */
  public runToRPM(Shooter shooter, double rpm) {

this.shooter = shooter;
this.rpm = rpm;

//if runToRPM requires the shooter to already be moving, would adding shooter as a requirement be bad since it needs RunShooter
addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

double rpm = shooter.getShooterVelocity();

if (rpm <= 5000){
isFinished = true;
}
else{
}

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

shooter.stopShooter();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
