package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Robot extends TimedRobot {
  private Joystick joy;
  private double velocity, Mag, turnRatio, LS, RS, px, py, TrigAxi;
  private final VictorSPX leftMotor1 = new VictorSPX(0);
  private final VictorSPX leftMotor2 = new VictorSPX(1);
  private final VictorSPX rightMotor1 = new VictorSPX(2);
  private final VictorSPX rightMotor2 = new VictorSPX(3);
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
    px = Deadzone(joy.getRawAxis(0));
    py = Deadzone(-joy.getRawAxis(1));
    if (pov != -1) {
      POVS();
    }else{
      calculateMotorSpeeds(px, py);
      if (px == 0 && py == 0) {
        RS = TrigAxi * velocity;
        LS = TrigAxi * velocity;
      }
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
            RS = 0.5; 
            break;
        case 90:  
            LS = 0;
            RS = 1;
            break;
        case 135: 
            LS = -0.5; 
            RS = -1;
            break;
        case 180:
            LS = -1;
            RS = -1;
            break;
        case 225: 
            LS = -1;
            RS = -0.5;
            break;
        case 270: 
            LS = 0;
            RS = 1;
            break;
        case 315:
            LS = 0.5; 
            RS = 1;
            break;
        default:  
            LS = 0;
            RS = 0;
            break;
    }
    LS *= velocity;
    RS *= velocity;
    LS = Math.max(-velocity, Math.min(velocity, LS));
    RS = Math.max(-velocity, Math.min(velocity, RS));
}
  private void calculateMotorSpeeds(double px, double py) {
    Mag = calculateMag(px, py);
    double AngEmRadiano = Math.atan2(px, py);
    double graus = Math.toDegrees(AngEmRadiano);
    turnRatio = Math.abs(graus / 180) * TrigAxi;
    
    Mag = Math.min(Mag, 1);
    quad = getquad(px, py);
    switch (quad) {
      case 1:
        LS = (TrigAxi + turnRatio * 2) * Mag;
        RS = (TrigAxi - turnRatio * 2) * Mag;
        break;
      case 2:
        LS = (TrigAxi - turnRatio * 2) * Mag;
        RS = (TrigAxi + turnRatio * 2) * Mag;
        break;
      case 3:
        LS = (TrigAxi + turnRatio * 2) * -Mag;
        RS = (TrigAxi - turnRatio * 2) * Mag;
        break;
      case 4:
        LS = (TrigAxi - turnRatio * 2) * Mag;
        RS = (TrigAxi + turnRatio * 2) * -Mag;
        break;
      default:
        LS = 0;
        RS = 0;
        break;
    }
    if (graus == 0) {
      LS = Mag * TrigAxi;
      RS = Mag * TrigAxi;
    }
    if (graus == 180) {
      LS = -Mag * TrigAxi;
      RS = -Mag * TrigAxi;
    }
    if (graus == 90) {
      LS = Mag * TrigAxi;
      RS = 0;
    }
    if (graus == -90) {
      LS = 0;
      RS = Mag * TrigAxi;
    }
    LS *= velocity;
    RS *= velocity;
    LS = Math.max(-velocity, Math.min(velocity, LS));
    RS = Math.max(-velocity, Math.min(velocity, RS));
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
    SmartDashboard.putNumber("Graus da curva", Math.toDegrees(Math.atan2(joy.getRawAxis(0), joy.getRawAxis(1) * -1)));
    SmartDashboard.putNumber("Velocidade", velocity);
    SmartDashboard.putNumber("Magnitude", calculateMag(px, py));
    SmartDashboard.putNumber("QUad", quad);
  }
}