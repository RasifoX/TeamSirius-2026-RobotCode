package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

/**
 * AUTOS - OTONOM FABRİKASI
 * * Burası otonom komutlarının üretildiği yerdir.
 * * Şu anlık sadece boş bir komut döndürüyoruz.
 * * İleride "PathPlanner" buraya entegre edilecek.
 */
public final class Autos {

  /**
   * Örnek Otonom Komutu
   * Şimdilik hiçbir şey yapmıyor (Idle).
   */
  public static Command exampleAuto(DriveSubsystem subsystem) {
    // Robot olduğu yerde durur.
    return Commands.none();
  }

  // Bu sınıfın nesnesi oluşturulamaz (Utility Class mantığı)
  private Autos() {
    throw new UnsupportedOperationException("Bu bir utility sınıfıdır!");
  }
}