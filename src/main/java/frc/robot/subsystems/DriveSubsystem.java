package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;

/**
 * DRIVE SUBSYSTEM - ROBOTUN OMURİLİĞİ
 * * 4 Swerve Modülünü ve Gyro'yu (Pigeon2) yönetir.
 * * Joystick'ten gelen ham veriyi, tekerleklerin anlayacağı hıza ve açıya
 * çevirir.
 * * Sahadaki konumunu (Odometry) sürekli takip eder.
 */
public class DriveSubsystem extends SubsystemBase {

  // =====================================================================
  // 1. DONANIM TANIMLAMALARI (Hardware)
  // Constants dosyasındaki ID ve Offsetleri kullanarak modülleri yaratıyoruz.
  // =====================================================================

  // Sol Ön (Front Left) Modülü
  private final SwerveModule m_frontLeft = new SwerveModule(
      PortConstants.kFrontLeftDriveID,
      PortConstants.kFrontLeftSteerID,
      PortConstants.kFrontLeftCANCoderID,
      Preferences.getDouble("FL_Offset", PortConstants.kFrontLeftOffset));

  // Sağ Ön (Front Right) Modülü
  private final SwerveModule m_frontRight = new SwerveModule(
      PortConstants.kFrontRightDriveID,
      PortConstants.kFrontRightSteerID,
      PortConstants.kFrontRightCANCoderID,
      Preferences.getDouble("FR_Offset", PortConstants.kFrontRightOffset));

  // Sol Arka (Rear Left) Modülü
  private final SwerveModule m_rearLeft = new SwerveModule(
      PortConstants.kRearLeftDriveID,
      PortConstants.kRearLeftSteerID,
      PortConstants.kRearLeftCANCoderID,
      Preferences.getDouble("RL_Offset", PortConstants.kRearLeftOffset));

  // Sağ Arka (Rear Right) Modülü
  private final SwerveModule m_rearRight = new SwerveModule(
      PortConstants.kRearRightDriveID,
      PortConstants.kRearRightSteerID,
      PortConstants.kRearRightCANCoderID,
      Preferences.getDouble("RR_Offset", PortConstants.kRearRightOffset));

  // GYRO (Pusula)
  // Robotun sahada nereye baktığını (Heading) söyler.
  private final GyroIO m_gyroIO;

  // ODOMETRY (Konum Takipçisi)
  // Robotun (x, y) koordinatını hesaplar.
  private final SwerveDriveOdometry m_odometry;

  /**
   * CONSTRUCTOR (Kurucu Metot)
   * Robot açıldığında bir kez çalışır.
   */
  public DriveSubsystem() {
    // 1. Gyro'yu sıfırla. Robot başlangıçta karşıya bakıyor kabul edilir.
    // (Maç başında robotu nasıl koyduğun çok önemlidir!)
    NavXIO navx = new NavXIO();
    if (navx.isConnected()) {
      m_gyroIO = navx;
      System.out.println("[KOKPİT] NavX Aktif!");
    } else {
      m_gyroIO = new Pigeon2IO(DriveConstants.kPigeonID);
      System.out.println("[KOKPİT] Pigeon2 Aktif!");
    }

    // 2. Odometry sistemini başlat.
    // Başlangıç noktası (0,0) ve açısı 0 derece olarak ayarlanır.
    m_odometry = new SwerveDriveOdometry(
        DriveConstants.kDriveKinematics,
        getRotation2d(),
        getModulePositions());

    // --- PATHPLANNER KURULUMU (AUTOBUILDER) ---
    try {
      // Robotun fiziksel özelliklerini (Atalet momenti, motor torku vs.) yükle
      // GUI'den oluşturacağın "config" dosyasına göre çalışır.
      // Şimdilik GUI kullanmadığımız için manuel config veriyoruz:
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose, // Robot şu an nerede? (Pose2d)
          this::resetOdometry, // Robotun konumu nasıl sıfırlanır?
          this::getChassisSpeeds, // Robot şu an ne hızla gidiyor? (Bunu aşağıda yazacağız)
          (speeds, feedforwards) -> driveRobotRelative(speeds), // Robotu sür (ChassisSpeeds ile)
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID (İlerleme hatasını düzelt)
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID (Dönüş hatasını düzelt)
          ),
          config,
          () -> {
            // Kırmızı ittifak isek yolu otomatik ters çevir (Mirror)
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Gereksinim (Subsystem)
      );
    } catch (Exception e) {
      // Config yüklenemezse hata bas (GUI'den ayar yapmadığımız için düşebilir)
      System.out.println("PathPlanner Config Hatası: " + e.getMessage());
    }
  }

  /**
   * PERİYODİK DÖNGÜ (Heartbeat)
   * Saniyede 50 kere (20ms'de bir) çalışır.
   */
  @Override
  public void periodic() {
    m_odometry.update(m_gyroIO.getRotation2d(), getModulePositions());

    // --- UÇAK KONSOLU DIAGNOSTICS ---
    updateDashboard();

    // --- DINAMIK AKIM KORUMASI ---
    // Voltaj 10V altına düşerse akımı kıs, robot kapanmasın (Brownout protection)
    double currentLimit = RobotController.getBatteryVoltage() < 10.5 ? 30.0 : 50.0;
    updateDriveCurrentLimits(currentLimit);
  }

  private void updateDashboard() {
    SmartDashboard.putBoolean("Cockpit/Gyro OK", m_gyroIO.isConnected());
    SmartDashboard.putBoolean("Cockpit/DriveStick OK", DriverStation.isJoystickConnected(0));
    // Isınan motoru anında yakala
    SmartDashboard.putNumber("Cockpit/Max Motor Temp", getMaxModuleTemp());
  }

  // OFSETLERI CANLI KAYDETME (Pit alanında hayat kurtarır)
  public void saveOffsets() {
    Preferences.setDouble("FL_Offset", m_frontLeft.getAbsolutePosition());
    Preferences.setDouble("FR_Offset", m_frontRight.getAbsolutePosition());

    Preferences.setDouble("RL_Offset", m_rearLeft.getAbsolutePosition());
    Preferences.setDouble("RR_Offset", m_rearRight.getAbsolutePosition());

    System.out.println("[OK] Ofsetler RoboRIO hafızasına kazındı!");
  }

  // Akım limitlerini tüm modüllere basar
  private void updateDriveCurrentLimits(double amps) {
    m_frontLeft.setDriveCurrentLimit(amps);
    m_frontRight.setDriveCurrentLimit(amps);
    m_rearLeft.setDriveCurrentLimit(amps);
    m_rearRight.setDriveCurrentLimit(amps);
  }

  // En sıcak modülün sıcaklığını döndürür
  private double getMaxModuleTemp() {
    return Math.max(
        Math.max(m_frontLeft.getTemp(), m_frontRight.getTemp()),
        Math.max(m_rearLeft.getTemp(), m_rearRight.getTemp()));
  }

  /**
   * ANA SÜRÜŞ FONKSİYONU (Drive Method)
   * Joystick komutlarının işlendiği yer.
   * * @param xSpeed İleri/Geri Hız (m/s)
   * 
   * @param ySpeed        Sağ/Sol Hız (m/s)
   * @param rot           Dönüş Hızı (rad/s)
   * @param fieldRelative True: Sahaya göre sür (Önerilen), False: Robota göre sür
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    // 1. KİNEMATİK HESAPLAMA
    // Joystick'ten gelen hızları (ChassisSpeeds), 4 tekerleğin yapması gereken
    // hız ve açılara (SwerveModuleState) dönüştürür.
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    // 2. DESATURATION (Hız Dengeleme) - KRİTİK GÜVENLİK
    // Eğer bir tekerlek fiziksel limitinden (örn: 4.5 m/s) daha hızlı dönmesi
    // gerekirse,
    // tüm tekerlekleri oranlayarak yavaşlatır. Böylece robotun rotası şaşmaz.
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);

    // 3. TEKERLEKLERE EMİR VER
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * SAVUNMA MODU (X-Stance)
   * Tekerlekleri X şeklinde kilitler. Robotu itseler bile kıpırdamaz.
   */
  public void stopModules() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * GYRO SIFIRLAMA
   * Robotun o an baktığı yönü "Ön" (0 derece) kabul eder.
   * Maç sırasında robotun kafası karışırsa sürücü buna basar.
   */
  public void zeroHeading() {
    m_gyroIO.reset();
  }

  /**
   * Robotun şu anki açısını (Heading) döndürür.
   * 
   * @return Derece (-180 ile 180 arası)
   */
  public double getHeading() {
    return getRotation2d().getDegrees();
  }

  /**
   * Robotun açısını Rotation2d objesi olarak döndürür.
   * (Odometry ve Kinematik hesapları bunu kullanır)
   */
  public Rotation2d getRotation2d() {
    // Pigeon2 bazen sürekli artan değer verir (365, 366...), bunu normalize
    // edebilirsin
    // ama Odometry genelde raw veriyi sever.
    // Eğer Gyro ters montajlıysa buraya .unaryMinus() ekle.
    return DriveConstants.kGyroReversed ? m_gyroIO.getRotation2d().unaryMinus() : m_gyroIO.getRotation2d();
  }

  /**
   * Robotun sahadaki (x, y) konumunu döndürür.
   * Otonom için gereklidir.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Otonom başlangıcında robotun konumunu elle ayarlamak için.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        getRotation2d(),
        getModulePositions(),
        pose);
  }

  /**
   * Tüm modüllerin anlık pozisyonlarını (metre ve açı) paketler.
   * Odometry güncellemesi için şarttır.
   */
  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /**
   * Robotun anlık hızlarını (İleri ve Dönüş) döndürür.
   * PathPlanner bunu PID hesabı için kullanır.
   */
  public ChassisSpeeds getChassisSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Robotu "Robota Göre" (Robot Relative) sürer.
   * PathPlanner otonomda bunu kullanır.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Robot Relative olduğu için fieldRelative = false gönderiyoruz.
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  /**
   * Modül durumlarını (States) döndüren yardımcı metot.
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }
}