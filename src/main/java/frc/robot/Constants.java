package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * CONSTANTS (SABİTLER) - ROBOTUN DNA'SI
 * * Burası "Ana Kadran Santrali"dir. 
 * Robotun fiziksel özellikleri, port numaraları ve limitleri buradan yönetilir.
 * Başka hiçbir dosyada el ile sayı girilmez.
 */
public final class Constants {

  // =============================================================================
  // 1. MODÜL AYARLARI (Module Constants)
  // Tekerleklerin fiziksel özellikleri. MK4i (L2) kullandığını varsayıyoruz.
  // =============================================================================
  public static final class ModuleConstants {
    
    // Sürücü voltajı ne olursa olsun, motorlar sanki hep 12V varmış gibi çalışır.
    public static final double kNominalVoltage = 12.0;

    // ---------------------------------------------------------------------------
    // FİZİKSEL ÖLÇÜLER
    // ---------------------------------------------------------------------------
    // Tekerlek Çapı: Standart 4 inç. (Metreye çeviriyoruz çünkü fizik kütüphanesi metre sever)
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    
    // Tekerlek Çevresi: Çap * Pi
    public static final double kWheelCircumference = kWheelDiameterMeters * Math.PI;

    // DİŞLİ ORANLARI (Gear Ratios) - MK4i L2 Modülü
    // Motor kaç tur atınca tekerlek 1 tur atıyor?
    public static final double kDriveGearRatio = 6.75;      // Sürüş (Drive) Oranı
    public static final double kSteerGearRatio = 150.0 / 7.0; // Dönüş (Steer) Oranı (Yaklaşık 21.43)

    // ---------------------------------------------------------------------------
    // MATEMATİKSEL DÖNÜŞÜM FAKTÖRLERİ (Magic Math)
    // Motorun "Devir" dilinden, Robotun "Metre" diline çeviri katsayıları.
    // ---------------------------------------------------------------------------
    
    // Sürüş Pozisyon Faktörü: 1 Motor Devri = Kaç Metre yol?
    // Formül: (1 / Dişli Oranı) * Tekerlek Çevresi
    public static final double kDrivePositionFactor = (1.0 / kDriveGearRatio) * kWheelCircumference;
    
    // Sürüş Hız Faktörü: 1 RPM = Kaç Metre/Saniye?
    // Formül: Pozisyon Faktörü / 60 saniye
    public static final double kDriveVelocityFactor = kDrivePositionFactor / 60.0;

    // Dönüş Pozisyon Faktörü: 1 Motor Devri = Kaç Radyan açı?
    // Formül: (1 / Dişli Oranı) * 2 Pi
    public static final double kSteerPositionFactor = (1.0 / kSteerGearRatio) * 2 * Math.PI;
    
    // Dönüş Hız Faktörü: 1 RPM = Kaç Radyan/Saniye?
    public static final double kSteerVelocityFactor = kSteerPositionFactor / 60.0;

    // ---------------------------------------------------------------------------
    // PID & GÜVENLİK AYARLARI
    // ---------------------------------------------------------------------------
    // Dönüş Motoru PID (Tekerlek hedef açıya giderken titrerse P'yi düşür)
    public static final double kTurningP = 1.0; 
    public static final double kTurningI = 0.0;
    public static final double kTurningD = 0.0;

    // Akım Limitleri (Amper) - Motorları yakmamak için sigorta
    public static final int kDriveCurrentLimit = 50; // Sürüş motoru güçlüdür, 50A verilir.
    public static final int kSteerCurrentLimit = 20; // Dönüş motoru küçüktür, 20A yeter.
  }

  // =============================================================================
  // 2. ŞASİ AYARLARI (Drive Constants)
  // Robotun gövdesinin özellikleri.
  // =============================================================================
  public static final class DriveConstants {
    
    // ---------------------------------------------------------------------------
    // ROBOT BOYUTLARI (! ÖLÇÜLMELİ VE DEĞİŞTİRİLMELİ !)
    // Tekerleklerin MERKEZİNDEN MERKEZİNE ölçülecek.
    // ---------------------------------------------------------------------------
    public static final double kTrackWidth = Units.inchesToMeters(24.5); // Sağ ve Sol teker arası
    public static final double kWheelBase = Units.inchesToMeters(24.5);  // Ön ve Arka teker arası

    // ---------------------------------------------------------------------------
    // KİNEMATİK HARİTASI
    // Robotun beynine tekerleklerin yerini öğretiyoruz.
    // Merkez noktası (0,0) robotun tam ortasıdır.
    // ---------------------------------------------------------------------------
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),   // Ön Sol (+X, +Y)
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),  // Ön Sağ (+X, -Y)
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Arka Sol (-X, +Y)
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // Arka Sağ (-X, -Y)
    );

    // ---------------------------------------------------------------------------
    // GYRO (PUSULA) AYARLARI
    // ---------------------------------------------------------------------------
    public static final int kPigeonID = 13; // Pigeon 2.0 CAN ID'si
    public static final boolean kGyroReversed = false; // Gyro ters dönüyorsa true yap

    // ---------------------------------------------------------------------------
    // HIZ SINIRLARI
    // ---------------------------------------------------------------------------
    public static final double kMaxSpeedMetersPerSecond = 4.5; // Saniyede 4.5 metre (Çok hızlı)
    public static final double kMaxAngularSpeed = 2 * Math.PI; // Saniyede 1 tam tur dönüş
  }

  // =============================================================================
  // 3. PORT HARİTASI (Port Constants)
  // Kablolama planı. REV Hardware Client ve Phoenix Tuner'daki ID'ler buraya!
  // =============================================================================
  public static final class PortConstants {
    
    // --- ÖN SOL (Front Left) ---
    public static final int kFrontLeftDriveID = 1;      // Sürüş Motoru
    public static final int kFrontLeftSteerID = 2;      // Dönüş Motoru
    public static final int kFrontLeftCANCoderID = 3;   // Mutlak Encoder
    // OFFSET: Tekerleği elinle düzle, Phoenix Tuner'dan okuduğun değeri buraya yaz.
    // Radyan cinsinden olmalı! (Derece okuyorsan Math.toRadians() içine yaz)
    public static final double kFrontLeftOffset = -Math.toRadians(0.0); 

    // --- ÖN SAĞ (Front Right) ---
    public static final int kFrontRightDriveID = 4;
    public static final int kFrontRightSteerID = 5;
    public static final int kFrontRightCANCoderID = 6;
    public static final double kFrontRightOffset = -Math.toRadians(0.0);

    // --- ARKA SOL (Back Left) ---
    public static final int kBackLeftDriveID = 7;
    public static final int kBackLeftSteerID = 8;
    public static final int kBackLeftCANCoderID = 9;
    public static final double kBackLeftOffset = -Math.toRadians(0.0);

    // --- ARKA SAĞ (Back Right) ---
    public static final int kBackRightDriveID = 10;
    public static final int kBackRightSteerID = 11;
    public static final int kBackRightCANCoderID = 12;
    public static final double kBackRightOffset = -Math.toRadians(0.0);
  }

  // =============================================================================
  // 4. OPERATÖR (OI Constants)
  // Joystick ve Sürücü Ayarları
  // =============================================================================
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // USB 0'a takılı Xbox Kumandası
    
    // Deadband (Ölü Bölge): Joystick'i bıraktığında oluşan milimetrik titremeleri yoksay.
    // %5'in altındaki hareketleri "Duruyor" kabul et.
    public static final double kDriveDeadband = 0.05; 
  }
}