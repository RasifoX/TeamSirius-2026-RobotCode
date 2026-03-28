package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * CONSTANTS (SABİTLER) - ROBOTUN DNA'SI
 *
 * Burası "Ana Kadran Santrali"dir.
 * Robotun fiziksel özellikleri, port numaraları ve limitleri buradan yönetilir.
 * Başka hiçbir dosyada el ile sayı girilmez.
 */
public final class Constants {

  // =============================================================================
  // 1. MODÜL AYARLARI (Module Constants)
  //    MK4i L2 modülü.
  // =============================================================================
  public static final class ModuleConstants {

    // Voltaj kompansasyonu: Pil voltajı ne olursa olsun motorlar sanki 12V varmış gibi çalışır.
    public static final double kNominalVoltage = 12.0;

    // ---------------------------------------------------------------------------
    // FİZİKSEL ÖLÇÜLER
    // ---------------------------------------------------------------------------
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumference   = kWheelDiameterMeters * Math.PI;

    // MK4i L2 Dişli Oranları
    public static final double kDriveGearRatio = 6.75;           // Sürüş
    public static final double kSteerGearRatio = 150.0 / 7.0;   // Dönüş (≈ 21.43)

    // ---------------------------------------------------------------------------
    // DÖNÜŞÜM FAKTÖRLERİ
    // SparkMax, encoder'ı "motor rotasyonu" cinsinden sayar. Bunu gerçek birimlere çeviriyoruz.
    // ---------------------------------------------------------------------------
    // Drive: 1 motor rotasyonu = (1 / GearRatio) * çevre = metre
    public static final double kDrivePositionFactor = (1.0 / kDriveGearRatio) * kWheelCircumference; // m/rot
    public static final double kDriveVelocityFactor = kDrivePositionFactor / 60.0;                   // m/s per RPM

    // Steer: 1 motor rotasyonu = (1 / GearRatio) * 2π = radyan
    public static final double kSteerPositionFactor = (1.0 / kSteerGearRatio) * (2.0 * Math.PI); // rad/rot
    public static final double kSteerVelocityFactor = kSteerPositionFactor / 60.0;               // rad/s per RPM

    // ---------------------------------------------------------------------------
    // PID AYARLARI (Dönüş Motoru)
    // Titreme varsa P'yi azalt. Yavaş tepkiyse artır.
    // ---------------------------------------------------------------------------
    public static final double kTurningP = 1.0;
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.05; // Küçük D titremeleri bastırır

    // ---------------------------------------------------------------------------
    // AKIM LİMİTLERİ
    // ---------------------------------------------------------------------------
    public static final int kDriveCurrentLimit = 50;
    public static final int kSteerCurrentLimit = 20;
  }

  // =============================================================================
  // 2. ŞASİ AYARLARI (Drive Constants)
  // =============================================================================
  public static final class DriveConstants {

    // ---------------------------------------------------------------------------
    // ROBOT BOYUTLARI — TEKERLEKLERİN MERKEZİNDEN MERKEZİNE (metre)
    // Cetvel ile ölç, buraya yaz.
    // ---------------------------------------------------------------------------
    public static final double kTrackWidth = Units.inchesToMeters(24.5); // Sol ↔ Sağ
    public static final double kWheelBase  = Units.inchesToMeters(24.5); // Ön  ↔ Arka

    // Kinematik harita: robotun merkezinden (0,0) her tekerleğin offset'i
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d( kWheelBase / 2,  kTrackWidth / 2),  // Ön Sol  (+X, +Y)
        new Translation2d( kWheelBase / 2, -kTrackWidth / 2),  // Ön Sağ  (+X, -Y)
        new Translation2d(-kWheelBase / 2,  kTrackWidth / 2),  // Arka Sol (-X, +Y)
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)   // Arka Sağ (-X, -Y)
    );

    // ---------------------------------------------------------------------------
    // GYRO SEÇİMİ
    // NavX varsa kUseNavX = true yap, yoksa Pigeon kullanılır (varsayılan).
    // ---------------------------------------------------------------------------
    public static final boolean kUseNavX     = false;
    public static final int     kPigeonID    = 13;
    public static final boolean kGyroReversed = false; // Gyro ters dönüyorsa true

    // ---------------------------------------------------------------------------
    // HIZ SINIRLARI
    // ---------------------------------------------------------------------------
    public static final double kMaxSpeedMetersPerSecond = 4.5;      // m/s
    public static final double kMaxAngularSpeed          = 2 * Math.PI; // rad/s (1 tam tur/s)
  }

  // =============================================================================
  // 3. PORT HARİTASI (Port Constants)
  //
  // ✅ OFFSET NASIL YAZILIR?
  //    1. Robotu sehpaya al (tekerlekler havada).
  //    2. Tüm tekerlekleri fiziksel olarak ileriye hizala (cetvel kullan).
  //    3. Phoenix Tuner X aç → Her CANcoder'ın "Absolute Position" değerini bak.
  //    4. O değeri (ROTASYONda, örn: 0.347) aşağıya AYNEN yaz.
  //       ❌ Eksi YAPMA, dereceye ÇEVİRME — sadece gördüğün sayıyı yaz.
  //    5. Kodu deploy et. Kalibrasyon bitti.
  //
  //    Örnek: Tuner X'te FL CANcoder "0.347" gösteriyorsa →  kFrontLeftOffset = 0.347
  // =============================================================================
  public static final class PortConstants {

    // --- ÖN SOL (Front Left) ---
    public static final int    kFrontLeftDriveID    = 1;
    public static final int    kFrontLeftSteerID    = 2;
    public static final int    kFrontLeftCANCoderID = 3;
    public static final double kFrontLeftOffset     = 0.0; // ← Tuner X'teki ROTASYON değeri

    // MK4i: Sol taraf drive motorları genellikle ters çevrilmez.
    public static final boolean kFrontLeftDriveInverted  = false;
    public static final boolean kFrontLeftSteerInverted  = false;

    // --- ÖN SAĞ (Front Right) ---
    public static final int    kFrontRightDriveID    = 4;
    public static final int    kFrontRightSteerID    = 5;
    public static final int    kFrontRightCANCoderID = 6;
    public static final double kFrontRightOffset     = 0.0;

    // MK4i: Sağ taraf drive motorları genellikle ters çevrilir.
    // ⚠️  Eğer robot ileri giderken sola/sağa kıvrılıyorsa bunu true ↔ false çevir.
    public static final boolean kFrontRightDriveInverted = true;
    public static final boolean kFrontRightSteerInverted = false;

    // --- ARKA SOL (Rear Left) ---
    public static final int    kRearLeftDriveID    = 7;
    public static final int    kRearLeftSteerID    = 8;
    public static final int    kRearLeftCANCoderID = 9;
    public static final double kRearLeftOffset     = 0.0;

    public static final boolean kRearLeftDriveInverted = false;
    public static final boolean kRearLeftSteerInverted = false;

    // --- ARKA SAĞ (Rear Right) ---
    public static final int    kRearRightDriveID    = 10;
    public static final int    kRearRightSteerID    = 11;
    public static final int    kRearRightCANCoderID = 12;
    public static final double kRearRightOffset     = 0.0;

    public static final boolean kRearRightDriveInverted = true;
    public static final boolean kRearRightSteerInverted = false;
  }

  // =============================================================================
  // 4. OPERATÖR AYARLARI (OI Constants)
  // =============================================================================
  public static final class OIConstants {
    public static final int    kDriverControllerPort = 0;
    public static final double kDriveDeadband        = 0.05; // %5 joystick ölü bölge
  }
}