package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * ROBOT KERNEL - İŞLETİM SİSTEMİ ÇEKİRDEĞİ
 * * Robotun yaşam döngüsünü (Init, Periodic, Disabled) yönetir.
 * * Command-Based yapının kalbi olan "CommandScheduler" burada çalıştırılır.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  /**
   * ROBOT BAŞLATMA (Boot)
   * Robot elektriği aldığı an bir kere çalışır.
   */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    // --- KARA KUTU BAŞLATILIYOR ---
    // RoboRIO'nun içindeki loglamayı başlatır. (USB takılıysa oraya yazar)
    edu.wpi.first.wpilibj.DataLogManager.start();

    // DriverStation'dan gelen verileri (Joystick, FMS durumu) de kaydet.
    edu.wpi.first.wpilibj.DriverStation.startDataLog(
        edu.wpi.first.wpilibj.DataLogManager.getLog());
  }

  /**
   * ROBOT DÖNGÜSÜ (Heartbeat)
   * Mod ne olursa olsun (Teleop, Auto, Disabled) her 20ms'de bir çalışır.
   * BURASI ÇOK KRİTİK! SCHEDULER BURADA KOŞAR.
   */
  @Override
  public void robotPeriodic() {
    // Görev Yöneticisi'ne (Scheduler) "İşini yap" der.
    // Bu satır olmazsa Joystick okumaz, Motor dönmez, LED yanmaz.
    CommandScheduler.getInstance().run();
  }

  /**
   * DISABLED MOD (Robot Bağlı Ama Kapalı)
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * OTONOM BAŞLANGICI
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // DÜZELTME 3: schedule() yerine Scheduler kullanıyoruz.
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  /**
   * OTONOM DÖNGÜSÜ
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * TELEOP (SÜRÜCÜ) BAŞLANGICI
   */
  @Override
  public void teleopInit() {
    // Otonomdan kalan komut varsa iptal et.
    // Böylece sürücü kontrolü devraldığında robot kendi kendine gitmeye çalışmaz.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * TELEOP DÖNGÜSÜ
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * TEST MODU
   */
  @Override
  public void testInit() {
    // Teste girince her şeyi iptal et.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}