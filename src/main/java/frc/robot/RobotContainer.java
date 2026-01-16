package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ROBOT CONTAINER - KOKPİT VE KABLOLAMA MERKEZİ
 * * Alt sistemler (Subsystems), Komutlar (Commands) ve Joystickler burada
 * tanımlanır.
 * * "Hangi tuşa basınca ne olsun?" sorusunun cevabı buradadır.
 */
public class RobotContainer {

  // 1. ALT SİSTEMLERİ YARAT
  // Robotun fiziksel parçalarını (Drivetrain) burada oluşturuyoruz.
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // 2. KUMANDAYI YARAT
  // Sürücü kumandası (Driver Controller). USB 0 portuna takılı olmalı.
  private final XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // Otonom Seçici (Dashboard'da görünecek kutu)
  private final SendableChooser<Command> m_autoChooser;

  /**
   * CONSTRUCTOR
   * Robot açıldığında çalışır ve varsayılan ayarları yapar.
   */
  public RobotContainer() {

    // --- VARSAYILAN SÜRÜŞ KOMUTU (DEFAULT DRIVE) ---
    // Hiçbir tuşa basılmasa bile, robot sürekli Joystick'i dinlesin.
    // Oluşturduğumuz "SwerveJoystickCmd" sınıfını burada bağlıyoruz.
    m_robotDrive.setDefaultCommand(new SwerveJoystickCmd(
        m_robotDrive,

        // 1. İleri/Geri Hız (Sol Stick Y Ekseni)
        // Joystick ileri itilince negatif değer verir, o yüzden (-) ile çarpıp
        // tersliyoruz.
        () -> -m_driverController.getLeftY(),

        // 2. Sağ/Sol Hız (Sol Stick X Ekseni)
        // Sola itince negatif verir, WPILib'de Sol pozitiftir (Y ekseni). O yüzden
        // tersliyoruz.
        () -> -m_driverController.getLeftX(),

        // 3. Dönüş Hızı (Sağ Stick X Ekseni)
        // Sağa itince pozitif verir ama biz Saat Yönü Tersine (CCW) pozitif kabul
        // ederiz. Tersliyoruz.
        () -> -m_driverController.getRightX(),

        // 4. Field Oriented Modu (Her zaman aktif)
        // Pilotun kafası karışmasın diye robot hep sahaya göre sürülür.
        () -> true));

    // PATHPLANNER NAMED COMMANDS (İsimli Komutlar)
    // Otonom çizerken "IntakeAl" dediğinde hangi kod çalışsın?
    // Şimdilik boş, ama ileride buraya Intake komutlarını ekleyeceğiz.
    // NamedCommands.registerCommand("IntakeAl", new IntakeCommand());

    // OTONOM SEÇİCİYİ OLUŞTUR
    // PathPlanner "deploy/pathplanner/autos" klasöründeki dosyaları okur.
    m_autoChooser = AutoBuilder.buildAutoChooser();

    // Dashboard'a koy
    SmartDashboard.putData("Auto Chooser", m_autoChooser);

    // Tuş atamalarını yap
    configureBindings();
  }

  /**
   * TUŞ ATAMALARI (Button Bindings)
   * Joystick üzerindeki tuşlara görev atıyoruz.
   */
  private void configureBindings() {

    // GYRO SIFIRLAMA (Y Tuşu)
    // Maç sırasında robotun "Önü" karışırsa, pilot Y tuşuna basıp düzeltir.
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .onTrue(new RunCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    // SAVUNMA MODU / X-STANCE (A Tuşu)
    // Pilot A tuşuna basılı tuttuğu sürece tekerlekler X şeklini alır ve
    // kilitlenir.
    // Rakip robot bizi itmeye çalışırsa yerimizden kıpırdamayız.
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(() -> m_robotDrive.stopModules(), m_robotDrive));
  }

  /**
   * OTONOM KOMUTU
   * Maçın ilk 15 saniyesinde çalışacak komut.
   * Şimdilik null döndürüyoruz, sonra PathPlanner ekleyeceğiz.
   */
  public Command getAutonomousCommand() {
    // Seçili otonomu döndür
    return m_autoChooser.getSelected();
  }
}