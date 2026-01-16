package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * SWERVE JOYSTICK COMMAND - ROBOTUN REFLEKSLERİ
 * * Bu komut, Joystick verilerini alır ve DriveSubsystem'i sürer.
 * * "Supplier" (Tedarikçi) yapısı kullanır, böylece anlık veri okur.
 * * Deadband (Ölü Bölge) ve SlewRate (Yumuşak Hızlanma) uygular.
 */
public class SwerveJoystickCmd extends Command {

  private final DriveSubsystem m_driveSubsystem;

  // Joystick Veri Kaynakları (Neden Double değil de Supplier? Çünkü değer sürekli
  // değişiyor)
  private final Supplier<Double> m_xSpdFunction; // İleri/Geri
  private final Supplier<Double> m_ySpdFunction; // Sağ/Sol
  private final Supplier<Double> m_turningSpdFunction; // Dönüş
  private final Supplier<Boolean> m_fieldOrientedFunction; // Saha Odaklı mı?

  // İvme Sınırlayıcılar (Robotun devrilmesini ve patinajı önler)
  private final SlewRateLimiter m_xLimiter;
  private final SlewRateLimiter m_yLimiter;
  private final SlewRateLimiter m_turningLimiter;

  /**
   * CONSTRUCTOR
   * 
   * @param driveSubsystem        Sürüş sistemi
   * @param xSpdFunction          İleri hız kaynağı (Joystick Y ekseni)
   * @param ySpdFunction          Yana hız kaynağı (Joystick X ekseni)
   * @param turningSpdFunction    Dönüş kaynağı (Joystick Sağ X ekseni)
   * @param fieldOrientedFunction Field Oriented modu aktif mi?
   */
  public SwerveJoystickCmd(DriveSubsystem driveSubsystem,
      Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction,
      Supplier<Boolean> fieldOrientedFunction) {

    this.m_driveSubsystem = driveSubsystem;
    this.m_xSpdFunction = xSpdFunction;
    this.m_ySpdFunction = ySpdFunction;
    this.m_turningSpdFunction = turningSpdFunction;
    this.m_fieldOrientedFunction = fieldOrientedFunction;

    // Slew Rate Limiter Ayarı:
    // Saniyede 3 birim hıza çıkabilir. Yani 0'dan %100 hıza 1/3 saniyede çıkar.
    // Robot çok agresifse bu sayıyı düşür (örn: 1.5), çok hantalsa arttır.
    this.m_xLimiter = new SlewRateLimiter(3.0);
    this.m_yLimiter = new SlewRateLimiter(3.0);
    this.m_turningLimiter = new SlewRateLimiter(3.0);

    // Bu komut DriveSubsystem'i kullanır, o yüzden onu kilitler.
    // Aynı anda başka bir komut (örn: Otonom) tekerleklere erişemez.
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    // Komut başladığında yapılacak özel bir iş yok.
  }

  @Override
  public void execute() {
    // 1. JOYSTICK VERİLERİNİ OKU (Canlı Veri)
    double xSpeed = m_xSpdFunction.get();
    double ySpeed = m_ySpdFunction.get();
    double turningSpeed = m_turningSpdFunction.get();

    // 2. DEADBAND UYGULA (Titremeyi Yok Et)
    // Eğer sürücü elini joystickten çekerse, milimetrik titremeleri 0 kabul et.
    xSpeed = MathUtil.applyDeadband(xSpeed, OIConstants.kDriveDeadband);
    ySpeed = MathUtil.applyDeadband(ySpeed, OIConstants.kDriveDeadband);
    turningSpeed = MathUtil.applyDeadband(turningSpeed, OIConstants.kDriveDeadband);

    // KÜBİK GİRİŞ (Cubic Input) - Hassasiyet Ayarı
    // Math.pow(x, 3) işlemi, düşük değerleri daha da küçültür.
    // Örn: Joystick 0.5 ise -> 0.5^3 = 0.125 hız verir. (Yavaş hareket)
    // Örn: Joystick 1.0 ise -> 1.0^3 = 1.0 hız verir. (Tam gaz)
    // Math.copySign, işaretin (+ veya -) kaybolmamasını sağlar.

    xSpeed = Math.copySign(Math.pow(xSpeed, 3), xSpeed);
    ySpeed = Math.copySign(Math.pow(ySpeed, 3), ySpeed);
    turningSpeed = Math.copySign(Math.pow(turningSpeed, 3), turningSpeed);

    // 3. YUMUŞATMA (Slew Rate Limiter)
    // Ani hız değişimlerini engelle.
    xSpeed = m_xLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed = m_yLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;

    // Dönüş hızı daha hassas olduğu için açısal hız sabitiyle çarpıyoruz.
    turningSpeed = m_turningLimiter.calculate(turningSpeed) * DriveConstants.kMaxAngularSpeed;

    // 4. ALT SİSTEME GÖNDER
    // Hesaplanan temiz verileri DriveSubsystem'e iletiyoruz.
    m_driveSubsystem.drive(xSpeed, ySpeed, turningSpeed, m_fieldOrientedFunction.get());
  }

  @Override
  public void end(boolean interrupted) {
    // Eğer komut kesilirse veya biterse robotu güvenli şekilde durdur.
    m_driveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    // Bu komut asla kendiliğinden bitmez.
    // Joystick olduğu sürece (Teleop boyunca) çalışmaya devam eder.
    return false;
  }
}