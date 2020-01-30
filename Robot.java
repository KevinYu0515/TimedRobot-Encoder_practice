package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;

public class Robot extends TimedRobot {
  private WPI_VictorSPX leftmotor1 = new WPI_VictorSPX(1);
    private WPI_VictorSPX leftmotor2 = new WPI_VictorSPX(2);
    private WPI_VictorSPX rightmotor1 = new WPI_VictorSPX(3);
    private WPI_VictorSPX rightmotor2 = new WPI_VictorSPX(4);
    private final Joystick joy1 = new Joystick(0);
    private Encoder encoder = new Encoder(0, 1, true, EncodingType.k4X);
    private final double kDriveTick2Feet = 1.0/128*6*Math.PI/12;
    private double lastTimestamp;
    private double errorSum;
  
  @Override
  public void robotInit() {
    leftmotor1.setInvrted(true);
    leftmotor2.follow(leftmotor1);
    righttmotor1.setInvrted(false);
    rightmotor2.follow(rightmotor1);
    
  }
  @Override
  public void robotPeriodic(){
    SmartDashboard.putNumber("encoder value", encoder.get() *kDriveTick2Feet);
  }

  @Override
  public void autonomousInit() {
    encoder.reset();
    errorSum= 0;
    lastTimestamp=Timer.getFPGATimestamp(); 
  }
  final double kP= 0.5;
  final double kI= 0.5;
  double setpoint= 1;
  double iLimit=1;

  @Override
  public void autonomousPeriodic() {
    double sensorPosition= encoder.get()*kDriveTick2Feet;
    double dt= Timer.getFPGATimestamp()-lastTimestamp;
    double error= setpoint-sensorPosition;
    if(Math.abs(error) < iLimit){
      errorSum+= error*dt;
    }
    double outputSpeed= kP*error+ kI*errorSum;
    LeftMotor.set(ControlMode.PercentOutput,outputSpeed);
    RightMotor.set(ControlMode.PercentOutput,outputSpeed);
    
    lastTimestamp= Timer.getFPGATimestamp();
    lastError= error;
    
}
  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }


