
#  Team Sirius 2026 - Robot Code

##  Proje Hakkında

Bu depo (repo), **Team Sirius**'un 2026 FRC sezonu robotunun kontrol yazılımını içerir.

Bu mimari; spagetti kodu önlemek, modülerliği artırmak ve sahadaki değişikliklere (dişli oranı, motor değişimi vb.) saniyeler içinde adapte olabilmek için tasarlanmıştır.

### Mimari Özellikler (Ranix Architecture)

-   **Single Source of Truth (Tek Gerçeklik Kaynağı):** Robotun tüm fiziksel ölçüleri, PID değerleri ve Port numaraları sadece `Constants.java` dosyasında bulunur. Başka hiçbir dosyada "sihirli sayı" (magic number) kullanılmaz.
    
-   **Dynamic Swerve Modules:** Modül kodları (SwerveModule.java) dinamiktir. ID ve Offset değerlerini dışarıdan alır, böylece tek bir sınıf 4 tekerleği de yönetir.
    
-   **Gelişmiş Sürüş Kontrolü:**
    
    -   **Field Oriented:** Robotun burnu nereye bakarsa baksın, ileri komutu sahaya göre ileridir.
        
    -   **Cubic Input:** Joystick hassasiyeti kübik fonksiyonla yumuşatılır (Hassas nişan alma).
        
    -   **Slew Rate Limiter:** Robotun patinaj çekmesi ve devrilmesi matematiksel limitlerle engellenir.
        
    -   **Voltage Compensation:** Pil 12.5V iken de 10V iken de motorlara giden güç sabit tutulur (Tutarlılık).
        
-   **Kara Kutu (Logging):**  `DataLogManager` ile her maçın verisi (akım, voltaj, komutlar) otomatik kaydedilir.
    
-   **PathPlanner Ready:** Otonom sürüş için PathPlanner ve AutoBuilder entegrasyonu yapılmıştır.
    

##  Donanım Haritası (Hardware Map)

Robot üzerindeki temel bileşenler aşağıdadır:

-   **Swerve Modülleri:** SDS MK4i (L2 Ratio)
    
    -   Sürüş Oranı: 6.75:1
        
    -   Dönüş Oranı: 21.43:1
        
-   **Motorlar:** NEO Brushless (SparkMax Sürücü ile)
    
-   **Sensörler:**
    
    -   Tekerlek Yönü: CTRE CANcoder
        
    -   Alan Yönü (Gyro): CTRE Pigeon 2.0
        

##  CAN ID Listesi

Kablolama ve yazılım eşleşmesi için bu ID'ler kritik öneme sahiptir.

### Ön Sol Modül (Front Left)

-   **Drive Motor ID:** 1
    
-   **Steer Motor ID:** 2
    
-   **CANcoder ID:** 3
    

### Ön Sağ Modül (Front Right)

-   **Drive Motor ID:** 4
    
-   **Steer Motor ID:** 5
    
-   **CANcoder ID:** 6
    

### Arka Sol Modül (Back Left)

-   **Drive Motor ID:** 7
    
-   **Steer Motor ID:** 8
    
-   **CANcoder ID:** 9
    

### Arka Sağ Modül (Back Right)

-   **Drive Motor ID:** 10
    
-   **Steer Motor ID:** 11
    
-   **CANcoder ID:** 12
    

### Sensörler

-   **Pigeon 2.0 (Gyro):** 13
    

##  Sürücü Kontrolleri (Driver Station)

Sürücü **USB 0** portuna bağlı bir **Xbox Kontrolcüsü** kullanır.

-   **Sol Stick (Y):** İleri / Geri Hız (Field Oriented)
    
-   **Sol Stick (X):** Sola / Sağa Kayma (Strafe)
    
-   **Sağ Stick (X):** Kendi Ekseninde Dönüş (Rotation)
    
-   **Y Tuşu:**  **Zero Heading** (Gyro'yu sıfırlar, robotun önünü düzeltir)
    
-   **A Tuşu:**  **X-Stance** (Savunma Modu - Tekerlekleri kilitler)
    

##  Kurulum ve Kalibrasyon

Projeyi çalıştırmak için bilgisayarınızda şunlar yüklü olmalıdır:

1.  **WPILib 2026.1.1** (veya daha güncel sürümü)
    
2.  **REV Hardware Client** (Motor Firmware güncellemesi ve ID atama için)
    
3.  **Phoenix Tuner X** (CANcoder ve Pigeon yönetimi için)
    

### Projeyi Çalıştırma

```
git clone [https://github.com/TeamSirius/2026-Robot-Code.git](https://github.com/TeamSirius/2026-Robot-Code.git)
cd 2026-Robot-Code
./gradlew build

```

###  Kritik: Swerve Offset Kalibrasyonu (Sıfırlama)

Robot ilk kez çalıştırıldığında veya bir modül söküldüğünde bu ayar **yapılmak zorundadır**:

1.  Robotu sehpaya alın (Tekerlekler yere değmesin).
    
2.  Robot kapalıyken, tüm tekerlekleri elinizle **dişliler aynı yöne bakacak şekilde** dümdüz hizalayın (Cetvel kullanın).
    
3.  **Phoenix Tuner X** uygulamasını açıp robotla bağlanın.
    
4.  Her bir modülün (FL, FR, BL, BR) CANcoder'ına gidip **"Absolute Position"** değerini okuyun.
    
5.  Bu değerleri `src/main/java/frc/robot/Constants.java` dosyasındaki `PortConstants` bölümüne (eksi işaretiyle) girin.
    

Örnek:

```
public static final double kFrontLeftOffset = -Math.toRadians(125.4); // Okunan değer 125.4 ise
```

6.  Kodu tekrar yükleyin (Deploy).
    

## Yapılacaklar (TODO)

-   [ ] **Vision Entegrasyonu:** Limelight veya PhotonVision eklenerek Odometry hataları düzeltilecek.
    
-   [ ] **Otonom Rotaları:** PathPlanner üzerinde yarışma senaryoları çizilecek.
    
-   [ ] **PID Tuning:** Dönüş motorlarının P değerleri sahada test edilip ince ayar yapılacak.
    

Team Sirius Software Dept.