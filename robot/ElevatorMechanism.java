/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.command.Subsystem;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
/**
 * Add your docs here.
 */
public class ElevatorMechanism extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public WPI_TalonSRX liftMotor = new WPI_TalonSRX(RobotMap.p_liftmotor);
  // public Encoder liftEncoder = new Encoder(RobotMap.p_encoderchannel1, RobotMap.p_encoderchannel2, false, Encoder.EncodingType.k4X);
  // public FeedbackDevice encoder = FeedbackDevice.QuadEncoder;

  public ElevatorMechanism(){
    // liftEncoder.setDistancePerPulse(1000);
    // liftEncoder.reset();
    liftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void resetEncoder(){
    // liftEncoder.reset();
    liftMotor.setSelectedSensorPosition(0);
  }
  public double getEncoderCount(){
    // return liftEncoder.get();
    return liftMotor.getSelectedSensorPosition();
  }
  public void setEncoderDestination(int counts){
    liftMotor.getSelectedSensorPosition(counts);
  }

}
