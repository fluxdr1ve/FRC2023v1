// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.RelativeEncoder;

public class DriveTrainSubsystem extends SubsystemBase {
  private final CANSparkMax left = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          left,
          new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless));
          private final CANSparkMax right = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
          private final MotorControllerGroup m_rightMotors =
              new MotorControllerGroup(
                 right ,
                  new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless));
          private final RelativeEncoder leftEncoder = left.getEncoder();
          private final RelativeEncoder rightEncoder = right.getEncoder();  

  // The motors on the right side of the drive.


  private SlewRateLimiter filter1;
  private SlewRateLimiter filter2;
  
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  public DriveTrainSubsystem()
  {
    SlewRateLimiter filter1 = new SlewRateLimiter(3.5);
    SlewRateLimiter filter2 = new SlewRateLimiter(3);
  }


  public void arcadeDrive(double speed, double rot)
  {
    m_drive.arcadeDrive(filter1.calculate(speed), filter2.calculate(rot));
  }

  public void curvatureDrive(double throttle, double rot, boolean turnInPlace)
  {
    m_drive.curvatureDrive(filter1.calculate(throttle), filter2.calculate(rot), turnInPlace);
  }
  
  
  public void stop()
  {
    m_leftMotors.set(0);
    m_rightMotors.set(0);
  }

  public double getLeftSpeed()
  {
    return m_leftMotors.get();
  }

  public double getRightSpeed()
  {
    return m_rightMotors.get();
  }



  public void setMaxOutput(double maxOutput)
  {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getEncoderMeters()
  {
    return (leftEncoder.getPosition()+ rightEncoder.getPosition())/2;
  }

  
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
