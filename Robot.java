package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Robot extends TimedRobot {
  private Joystick joy;
  private double velocity, Mag, LS, RS, px1, py1, TrigAxi, px2, py2, graus;
  private final VictorSPX leftMotor1 = new VictorSPX(4);
  private final VictorSPX leftMotor2 = new VictorSPX(3);
  private final VictorSPX rightMotor1 = new VictorSPX(2);
  private final VictorSPX rightMotor2 = new VictorSPX(1);
  private boolean a, b, x;
  private int quad, pov;
  @Override
  public void robotInit() {
    joy = new Joystick(0);
    rightMotor1.setInverted(true);
    rightMotor2.follow(rightMotor1);
    leftMotor2.follow(leftMotor1);

  }
  @Override
  public void teleopPeriodic() {
    pov = joy.getPOV();
    a = joy.getRawButton(1);
    b = joy.getRawButton(2);
    x = joy.getRawButton(3);
    TrigAxi = Deadzone(joy.getRawAxis(3) - joy.getRawAxis(2));
    velocity = getVelocity(a, b, x);
      px2 = Deadzone(-joy.getRawAxis(4));
      py2 = Deadzone(joy.getRawAxis(5));
    px1 = Deadzone(joy.getRawAxis(0));
    py1 = Deadzone(-joy.getRawAxis(1));
    if (pov == -1) {
      if (calculateMag(px2, py2) != 0) {
        calculateMotorSpeeds(px2, py2);
      }else{
       calculateMotorSpeeds(px1, py1);
      }
    }else if (pov != -1) {
      POVS();
    }else{
      LS = RS = TrigAxi*velocity;
    }
    leftMotor1.set(ControlMode.PercentOutput, LS);
    rightMotor1.set(ControlMode.PercentOutput, RS);
    updateSmartDashboard();
  }
  private double getVelocity(boolean a, boolean b, boolean x) {
    if (a) return 0.5;
    if (b) return 0.25;
    if (x) return 1;
    return velocity;
  }
  private double Deadzone(double valor){
    if (Math.abs(valor) < 0.02) {
      return 0;
    } else{
      return valor;
    }
  }
  private void POVS() {
    
    switch (pov) {
        case 0:  
            LS = 1;
            RS = 1;
            break;
        case 45: 
            LS = 1;
            RS = 0; 
            break;
        case 90:  
            LS = 1;
            RS = -1;
            break;
        case 135: 
            LS = 0; 
            RS = -1;
            break;
        case 180:
            LS = -1;
            RS = -1;
            break;
        case 225: 
            LS = -1;
            RS = 0;
            break;
        case 270: 
            LS = -1;
            RS = 1;
            break;
        case 315:
            LS = 0; 
            RS = 1;
            break;
        default:  
            LS = 0;
            RS = 0;
            break;
    }
    LS *= 0.5;
    RS *= 0.5;
}
private void calculateMotorSpeeds(double px, double py) {
  double turnratio = Math.abs(px/Mag);
  quad = getquad(px, py);
  Mag = calculateMag(px, py);
  graus =  Math.toDegrees(Math.atan2(px, py * -1));
  if (Mag > 1) {
    Mag = 1;
  }
  switch (quad) {
    case 1:
      LS = Mag;
      RS = Mag - turnratio;
      break;
      case 2:
      RS = Mag;
      LS = Mag - turnratio;
      break;
      case 3:
      LS = -Mag;
      RS = -Mag + turnratio;
      break;
      case 4:
      RS = -Mag;
      LS = -Mag + turnratio;
      break;
    default:
    if (px > 0 && py == 0) {
      LS = Mag;
      RS = -Mag;
    }
    if (px < 0 && py == 0) {
      RS = Mag;
      LS = -Mag;
    }
    if (py > 0 && px == 0) {
      LS = RS = Mag;
    }
    if (py < 0 && px == 0) {
      LS = RS = -Mag;
    }
      break;
  }
  if (TrigAxi == 0) {
    TrigAxi =1;
  }
  LS *= TrigAxi * velocity;
  RS *= TrigAxi * velocity;
  }
  private double calculateMag(double px, double py) {
    return Math.sqrt(px * px + py * py);
  }
  private int getquad(double px, double py) {
    if (px > 0 && py > 0) return 1;
    if (px < 0 && py > 0) return 2;
    if (px > 0 && py < 0) return 4;
    if (px < 0 && py < 0) return 3;
    return 0;
  }
    private void updateSmartDashboard() {
    SmartDashboard.putNumber("Left Motor Speed", LS);
    SmartDashboard.putNumber("Right Motor Speed", RS);
    SmartDashboard.putNumber("Graus da curva", graus);
    SmartDashboard.putNumber("Velocidade", velocity);
    SmartDashboard.putNumber("Magnitude", Mag);
    SmartDashboard.putNumber("Quad", quad);
  }
}