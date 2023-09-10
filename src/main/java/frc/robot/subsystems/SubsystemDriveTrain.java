// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveTrain.*;

public class SubsystemDriveTrain extends SubsystemBase {

  CANSparkMax motorFrontLeft;
  CANSparkMax motorFrontRight;
  CANSparkMax motorRearLeft;
  CANSparkMax motorRearRight;

  SparkMaxPIDController PIDFrontLeft;
  SparkMaxPIDController PIDFrontRight;
  SparkMaxPIDController PIDRearLeft;
  SparkMaxPIDController PIDRearRight;

  MecanumDriveKinematics kinematics;


  /** Creates a new DriveTrain. */
  public SubsystemDriveTrain() {
    motorFrontLeft = new CANSparkMax(MotorIDs.frontLeft, MotorType.kBrushless);
    motorFrontRight = new CANSparkMax(MotorIDs.frontRight, MotorType.kBrushless);
    motorRearLeft = new CANSparkMax(MotorIDs.rearLeft, MotorType.kBrushless);
    motorRearRight = new CANSparkMax(MotorIDs.rearRight, MotorType.kBrushless);

    PIDFrontLeft = motorFrontLeft.getPIDController();
    PIDFrontRight = motorFrontRight.getPIDController();
    PIDRearLeft = motorRearLeft.getPIDController();
    PIDRearRight = motorRearRight.getPIDController();

    kinematics = new MecanumDriveKinematics(
      PhysicalDimensions.wheelPosFrontLeft, 
      PhysicalDimensions.wheelPosFrontRight, 
      PhysicalDimensions.wheelPosRearLeft, 
      PhysicalDimensions.wheelPosRearRight
    );
  }

  /**Sets the PID targets for each wheel to the given values
   * 
   * @param speedFrontLeft The percent speed [-1.0, 1.0] to run the front left motor at
   * @param speedFrontRight The percent speed [-1.0, 1.0] to run the front right motor at
   * @param speedRearLeft The percent speed [-1.0, 1.0] to run the rear left motor at
   * @param speedRearRight The percent speed [-1.0, 1.0] to run the rear right motor at
   * 
   * H!
   */
  protected void setMotorSpeedsTarget(double speedFrontLeft, double speedFrontRight, double speedRearLeft, double speedRearRight) {
    PIDFrontLeft.setReference(speedFrontLeft, ControlType.kVelocity);
    PIDFrontRight.setReference(speedFrontRight, ControlType.kVelocity);
    PIDRearLeft.setReference(speedRearLeft, ControlType.kVelocity);
    PIDRearRight.setReference(speedRearRight, ControlType.kVelocity);
  }

  /**Sets the raw speed values for each wheel to the given values
   * @deprecated Using direct motor percents is unideal. Use {@link #setMotorSpeedsTarget(double, double, double, double)} instead
   * 
   * @param speedFrontLeft The percent speed [-1.0, 1.0] to run the front left motor at
   * @param speedFrontRight The percent speed [-1.0, 1.0] to run the front right motor at
   * @param speedRearLeft The percent speed [-1.0, 1.0] to run the rear left motor at
   * @param speedRearRight The percent speed [-1.0, 1.0] to run the rear right motor at
   * 
   * H!
   */
  protected void setMotorSpeedsRaw(double speedFrontLeft, double speedFrontRight, double speedRearLeft, double speedRearRight) {
    motorFrontLeft.set(speedFrontLeft);
    motorFrontRight.set(speedFrontRight);
    motorRearLeft.set(speedRearLeft);
    motorRearRight.set(speedRearRight);
  }

  /**Sets the raw speed values for each wheel to the given values
   * @deprecated Using direct motor percents is unideal. Use {@link #setMotorSpeedsTarget(double, double, double, double)} instead
   * 
   * @param speeds The {@link MecanumDriveWheelSpeeds} object containing all the speeds for each wheel
   * 
   * H!
   */
  protected void setMotorSpeedsRaw(MecanumDriveWheelSpeeds speeds) {
    setMotorSpeedsRaw(speeds.frontLeftMetersPerSecond, speeds.frontRightMetersPerSecond, speeds.rearLeftMetersPerSecond, speeds.rearRightMetersPerSecond);
  }

  public void setCartesianVelocityTarget(double x, double y, double omega) {
    // TODO
  }

  /**Sets the velocity of the drive train in x, y, and theta, using raw percent speeds
   * @deprecated Using direct motor percents is unideal. Use {@link #setCartesianVelocityTarget(double, double, double)} instead
   * 
   * @param x The speed to move in the x direction (positive = right)
   * @param y The speed to move in the y direction (positive = left)
   * @param omega THe speed at which to rotate (positive = counterclockwise)
   * 
   * H!
   */
  public void setCartesianVelocityRaw(double x, double y, double omega) {
    MecanumDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(x, y, omega));

    setMotorSpeedsRaw(wheelSpeeds);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
