// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.Limelight;

public class AimAssist extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveTrainSubsystem m_drive;
  private final Limelight m_light;
  private double Kp = -0.1;
  private double min_command = 0.05;
  private int objectTracking;
  private DoubleSupplier m_forward;

  public AimAssist(DriveTrainSubsystem drive, Limelight lm, int tracked, DoubleSupplier forward) {
    m_drive = drive;
    m_light = lm;
    objectTracking = tracked;
    m_forward = forward;
    addRequirements(drive);
    addRequirements(lm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    m_light.setPipeline(objectTracking);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (m_light.hasTarget())
    {
        double heading_error = -m_light.getYaw();
        double steering_adjust = 0.0;
        if (m_light.getYaw() > 1.0)
        {
                steering_adjust = Kp*heading_error - min_command;
        }
        else if (m_light.getYaw() < 1.0)
        {
                steering_adjust = Kp*heading_error + min_command;
        }
        m_drive.arcadeDrive(m_forward.getAsDouble(), steering_adjust);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
