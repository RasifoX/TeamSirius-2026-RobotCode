package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * MAIN CLASS - BAŞLANGIÇ NOKTASI
 * * Burası Java uygulamasının giriş kapısıdır.
 * * Doğrudan Robot sınıfını başlatır.
 * * Buraya ASLA mantık kodu yazılmaz.
 */
public final class Main {
  private Main() {
  }

  public static void main(String... args) {
    // Robot.java sınıfını başlatır.
    RobotBase.startRobot(Robot::new);
  }
}