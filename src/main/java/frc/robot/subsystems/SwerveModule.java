package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ResetMode; // Yeni yerleri burası
import com.revrobotics.spark.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.DriveConstants;

/**
 * SWERVE MODULE - ROBOTUN KAS HÜCRESİ
 * * Bu sınıf tek bir tekerlek modülünü temsil eder (Örn: Sol Ön).
 * * İçinde 2 Motor (Sürüş + Dönüş) ve 3 Encoder (Sürüş + Dönüş + CANcoder)
 * vardır.
 * * REV 2026 API'si ile tamamen optimize edilmiştir.
 */
public class SwerveModule {
  // DONANIM NESNELERİ
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;

  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final SparkClosedLoopController m_turningPIDController; // Yeni PID Kontrolcüsü

  private final CANcoder m_turningCanCoder;
  private final double m_chassisAngularOffset; // Magnet ofseti (Kalibrasyon)

  /**
   * Modül Kurucusu (Constructor)
   * 
   * @param driveMotorId         Sürüş Motoru CAN ID
   * @param turningMotorId       Dönüş Motoru CAN ID
   * @param canCoderId           CANcoder ID
   * @param chassisAngularOffset Tekerleğin düz durması için gereken açı farkı
   */
  @SuppressWarnings("removal") // REV'in eski uyarılarını gizle
  public SwerveModule(int driveMotorId, int turningMotorId, int canCoderId, double chassisAngularOffset) {
    m_chassisAngularOffset = chassisAngularOffset;

    // 1. MOTORLARI YARAT (Fırçasız NEO Motorlar)
    m_driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
    m_turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);

    // 2. SENSÖRLERİ TANI
    m_turningCanCoder = new CANcoder(canCoderId);

    // -------------------------------------------------------------------------
    // SÜRÜŞ MOTORU KONFİGÜRASYONU (Drive Motor Config)
    // -------------------------------------------------------------------------
    SparkMaxConfig driveConfig = new SparkMaxConfig();

    // Fren Modu: Robot durunca tekerlekler kilitlensin (Kaymasın)
    driveConfig.idleMode(IdleMode.kBrake);

    // Voltaj Dengeleme: Pil düşse bile performans sabit kalsın (12V referans)
    driveConfig.voltageCompensation(ModuleConstants.kNominalVoltage);

    // Akım Sınırı: Motoru yakmamak için amper limiti (Constants'tan gelir)
    driveConfig.smartCurrentLimit(ModuleConstants.kDriveCurrentLimit);

    // Encoder Çevirimi: Motor devrini "Metre" cinsine çevir
    driveConfig.encoder.positionConversionFactor(ModuleConstants.kDrivePositionFactor);
    driveConfig.encoder.velocityConversionFactor(ModuleConstants.kDriveVelocityFactor);

    // Ayarları Motora Uygula (ResetMode: Fabrika ayarlarını sil, PersistMode:
    // Kalıcı hafızaya yaz)
    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // -------------------------------------------------------------------------
    // DÖNÜŞ MOTORU KONFİGÜRASYONU (Turning Motor Config)
    // -------------------------------------------------------------------------
    SparkMaxConfig turnConfig = new SparkMaxConfig();

    turnConfig.idleMode(IdleMode.kBrake);

    // Voltaj Dengeleme
    turnConfig.voltageCompensation(ModuleConstants.kNominalVoltage);

    turnConfig.smartCurrentLimit(ModuleConstants.kSteerCurrentLimit);

    // Encoder Çevirimi: Motor devrini "Radyan" cinsine çevir
    turnConfig.encoder.positionConversionFactor(ModuleConstants.kSteerPositionFactor);
    turnConfig.encoder.velocityConversionFactor(ModuleConstants.kSteerVelocityFactor);

    // PID Ayarları: Tekerleğin hedef açıya titremeden gitmesi için
    turnConfig.closedLoop.pid(
        ModuleConstants.kTurningP,
        ModuleConstants.kTurningI,
        ModuleConstants.kTurningD);

    // SÜREKLİ DÖNÜŞ (Position Wrapping) - ÇOK ÖNEMLİ!
    // Tekerlek 359 dereceden 1 dereceye geçerken motoru geri sarmaz, 360'ı geçip
    // devam eder.
    // Giriş aralığı: 0 ile 2*Pi radyan arası.
    turnConfig.closedLoop.positionWrappingEnabled(true);
    turnConfig.closedLoop.positionWrappingInputRange(0, 2 * Math.PI);

    // Ayarları Motora Uygula
    m_turningMotor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // -------------------------------------------------------------------------
    // NESNE BAĞLANTILARI
    // -------------------------------------------------------------------------
    // Ayarlar yapıldıktan sonra encoder ve PID nesnelerini alıyoruz.
    m_driveEncoder = m_driveMotor.getEncoder();
    m_turningEncoder = m_turningMotor.getEncoder();
    m_turningPIDController = m_turningMotor.getClosedLoopController();

    // Robot her açıldığında mutlak encoder (CANcoder) ile senkronize ol.
    resetToAbsolute();
  }

  /**
   * Modül Durumu (Hız ve Açı) - Telemetri için
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveEncoder.getVelocity(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * Modül Pozisyonu (Toplam Gidilen Yol ve Açı) - Odometry için
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveEncoder.getPosition(), new Rotation2d(m_turningEncoder.getPosition()));
  }

  /**
   * MODÜLE EMİR VERME (Kalbin Attığı Yer)
   * 
   * @param desiredState Hedeflenen hız ve açı
   */
  @SuppressWarnings("removal")
  public void setDesiredState(SwerveModuleState desiredState) {
    // 1. Mevcut açıyı al (Optimizasyon hesabı için)
    Rotation2d currentAngle = new Rotation2d(m_turningEncoder.getPosition());

    // 2. OPTİMİZASYON (Tembel Robot Prensibi) - GÜNCELLENDİ (2026 Uyumlu)
    // WPILib 2026: optimize() artık void döndürür ve desiredState'i yerinde
    // değiştirir.
    desiredState.optimize(currentAngle);

    // 3. Dönüş Motoruna PID ile Hedef Açıyı Ver - GÜNCELLENDİ (Slot Eklendi)
    // REV artık Slot belirtmemizi istiyor (Varsayılan: Slot 0)
    m_turningPIDController.setReference(
        desiredState.angle.getRadians(), // Artık direkt desiredState kullanıyoruz
        SparkMax.ControlType.kPosition,
        ClosedLoopSlot.kSlot0);

    // 4. Sürüş Motoruna Açık Döngü (Open Loop) Hız Ver
    // Hızı maksimum hıza bölerek -1 ile 1 arası bir değer buluyoruz.
    m_driveMotor.set(desiredState.speedMetersPerSecond / DriveConstants.kMaxSpeedMetersPerSecond);
  }

  // CANcoder'dan ofsetsiz ham değeri radyan olarak döndürür
  public double getAbsolutePosition() {
    return m_turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
  }

  // Akım limitini dinamik ayarlar
  public void setDriveCurrentLimit(double amps) {
    SparkMaxConfig config = new SparkMaxConfig();
    config.smartCurrentLimit((int) amps);

    // 2026'da kNoResetSafeParameters ve kNoPersistParameters kullanırken
    // SparkMax hiyerarşisini kullanmaya özen göster
m_driveMotor.configure(config, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);
  }

  /**
   * MUTLAK ENCODER SENKRONİZASYONU
   * Robot elektriği kestiğinde Spark Max açısını unutur.
   * CANcoder (Pilli/Hafızalı) gerçek açıyı bilir. Başlangıçta onu kopyalıyoruz.
   */
  public void resetToAbsolute() {
    // CANcoder 0-1 arası değer verir, bunu Radyana (0-2Pi) çeviriyoruz.
    double absolutePosition = m_turningCanCoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;

    // Offseti (Kalibrasyon farkını) düşüyoruz.
    absolutePosition -= m_chassisAngularOffset;

    // Spark Max'ın beynine "Sen şu an buradasın" diyoruz.
    m_turningEncoder.setPosition(absolutePosition);
  }

  /**
   * Sürüş motorunun sıcaklığını döndürür.
   * 
   * @return Santigrat derece cinsinden sıcaklık.
   */
  public double getTemp() {
    // Sürüş motoru SparkMax olduğu için motorun sıcaklığını çekiyoruz
    return m_driveMotor.getMotorTemperature();
  }

  // Acil Durum Freni
  public void stop() {
    m_driveMotor.set(0);
    m_turningMotor.set(0);
  }
}