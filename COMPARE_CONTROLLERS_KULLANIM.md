# compare_controllers — Kullanım Kılavuzu

Üç kontrolcüyü (PID, LQR, MPC) aynı harita ve koşullarda simüle edip sonuçlarını karşılaştıran tam iş akışı.

---

## Gereksinimler

- MATLAB + Simulink
- Control System Toolbox (`dlqr`, `c2d` için)
- Optimization Toolbox (`quadprog` — MPC için)

---

## Klasör Yapısı

```
8. Week Birleştirme/
├── compare_controllers.m       ← karşılaştırma ana scripti
├── export_run.m                ← log → .mat dışa aktarım
├── mpc_to_compare.m            ← MPC çıktısını köprüler
│
├── PID/
│   ├── PIDScript2.m            ← PID setup scripti
│   └── Bicycle_PID2.slx        ← PID Simulink modeli
│
├── DiscreteLQRwith_FFandLookahead/
│   ├── LQRScript_DiscKappa.m   ← LQR setup scripti
│   └── Bicycle_LQR_DiscKappa.slx ← LQR Simulink modeli
│
└── MPC/
    ├── MPC_Setup.m             ← MPC setup scripti
    └── TrajectoryTracking_MPC.slx ← MPC Simulink modeli
```

---

## Adım Adım Kullanım

### Ön Not: Harita Seçimi Tutarlılığı

Üç kontrolcü için **aynı harita tipini** (`map_type`) seçmek zorundasınız, aksi hâlde karşılaştırma geçersiz olur.

| `map_type` | Değer |
|---|---|
| Çember (YuvarlakMap) | `1` |
| Dikdörtgen (KareMap) | `2` |
| El Figürü (HandFigureMap) | `3` |

---

### ADIM 1 — PID Simülasyonunu Çalıştır

1. MATLAB'da **`PID/`** klasörünü çalışma dizini yapın veya o klasörü açın.
2. **`PIDScript2.m`** dosyasını açın.
3. Üst kısımdaki `map_type` değişkenini istediğiniz haritaya göre ayarlayın (varsayılan: `2` = Dikdörtgen).
4. Scripti çalıştırın:
   ```matlab
   run('PID/PIDScript2.m')
   ```
   Terminal çıktısında referans yol uzunluğu, tur sayısı ve başlangıç koordinatları görünür. Bir referans yol önizleme figürü açılır.
5. **`Bicycle_PID2.slx`** modelini açın ve simülasyonu başlatın (▶ tuşu).
6. Simülasyon bitince `sim_log_PID` workspace'e yazılır. Bunu diske kaydedin:
   ```matlab
   export_run('PID')
   ```
   → `results_PID.mat` oluşur.

> **Scope logu (opsiyonel — daha hassas metrikler için):**  
> Simulink modelindeki Scope bloğunun `Log data to workspace` ayarının açık olduğundan ve değişken adının `scope_PID`, formatının `Structure With Time` olduğundan emin olun. Sinyal sırası: `1) ey  2) eh  3) delta`.

---

### ADIM 2 — LQR Simülasyonunu Çalıştır

1. **`DiscreteLQRwith_FFandLookahead/`** klasörünü çalışma dizini yapın.
2. **`LQRScript_DiscKappa.m`** dosyasını açın.
3. `map_type` değerini PID ile **aynı** değere ayarlayın.
4. İsteğe bağlı olarak LQR ağırlık matrislerini düzenleyin:
   ```matlab
   Q_lqr = [70, 0; 0, 50];   % yan hata vs. heading hata ağırlığı
   R_lqr = 15;                % direksiyon çabası cezası
   ```
5. Scripti çalıştırın:
   ```matlab
   run('DiscreteLQRwith_FFandLookahead/LQRScript_DiscKappa.m')
   ```
   Terminal'de hesaplanan `K_lqr` kazancı ve yol bilgisi yazdırılır.
6. **`Bicycle_LQR_DiscKappa.slx`** modelini açın ve simülasyonu başlatın.
7. Simülasyon bitince diske kaydedin:
   ```matlab
   export_run('LQR')
   ```
   → `results_LQR.mat` oluşur.

---

### ADIM 3 — MPC Simülasyonunu Çalıştır

1. **`MPC/`** klasörünü çalışma dizini yapın.
2. **`MPC_Setup.m`** scriptini çalıştırın:
   ```matlab
   run('MPC/MPC_Setup.m')
   ```
   Bir menü açılır, oradan haritayı seçin (PID/LQR ile aynı harita!).
3. Script tamamlandığında workspace'de `mpc_params`, `ref_path`, `sim_params` yapıları oluşur. İki figür açılır: referans yol ve eğrilik profili.
4. **`TrajectoryTracking_MPC.slx`** modelini açın ve simülasyonu başlatın.
5. Simülasyon bitince **köprü scriptini** çalıştırın (MPC, logu farklı isimlendirdiği için):
   ```matlab
   mpc_to_compare()
   ```
   Bu komut `sim_log` → `sim_log_MPC` dönüşümünü yapar ve otomatik olarak `export_run('MPC')` çağırır.
   → `results_MPC.mat` oluşur.

---

### ADIM 4 — Karşılaştırmayı Çalıştır

Üç simülasyonun da `results_*.mat` dosyaları oluştuktan sonra **herhangi bir klasörden** çalıştırın:

```matlab
compare_controllers()
```

#### Üretilen Çıktılar

| Figür | İçerik |
|---|---|
| **Trajectory overlay** | Üç kontrolcünün xy yörüngeleri + referans yol (kesik çizgi) |
| **İlerleme s(t)** | Yol boyunca ilerleme karşılaştırması |
| **ey / eh / delta** | Her kontrolcü için yan hata, heading hatası ve direksiyon açısı grafikleri |
| **Terminal metrik tablosu** | RMS ey, Maks ey, RMS eh, Maks eh, kontrol çabası, toplam süre |

#### Örnek Terminal Çıktısı

```
Kontrolcu  RMS_ey[m]  Max_ey[m]  RMS_eh[deg]  Max_eh[deg]    Effort    t_son[s]
PID         0.1823      0.4921       2.341        8.120       12.3456    120.0
LQR         0.0941      0.2810       1.205        4.330        8.7123    120.0
MPC         0.0612      0.1944       0.891        3.102        9.0011    120.0
```

---

## Sık Kullanılan Parametreler

| Parametre | PIDScript2.m | LQRScript.m | MPC_Setup.m |
|---|---|---|---|
| Harita | `map_type` | `map_type` | menü |
| Hız | `v_ref = 8` | `v_ref = 8` | `v_const = 8` |
| Tur sayısı | `N_loops = 3` | `N_loops = 3` | `n_laps = 3` |
| Öngörme adımı | `lookahead_steps = 10` | — | `N_p = 50` |

---

## Farklı Oturumlarda Çalışma

`results_*.mat` dosyaları diske kaydedildiği için tüm simülasyonları aynı anda çalıştırmak zorunda değilsiniz. Her simülasyondan sonra `export_run` çağrısı yapıldığı sürece `compare_controllers` dosyaları diskten okur.

```
Oturum 1: PID çalıştır → export_run('PID')
Oturum 2: LQR çalıştır → export_run('LQR')
Oturum 3: MPC çalıştır → mpc_to_compare()
Oturum 4: compare_controllers()   ← tümünü karşılaştırır
```

---

## Düzenlenmesi Gereken Parametreler

Aşağıdaki dosyalarda **kullanıcıya özgü mutlak yollar** bulunmaktadır. Projeyi başka bir bilgisayarda veya farklı bir kullanıcı hesabında çalıştırmadan önce bu satırları güncellemeniz gerekir.

### `persistent_anim_update.m` — Araç Sprite Yolu

Aynı satır **4 ayrı klasörde** tekrarlanmaktadır:

| Dosya | Satır |
|---|---|
| `PID/persistent_anim_update.m` | 57 |
| `DiscreteLQRwith_FFandLookahead/persistent_anim_update.m` | 57 |
| `MPC/persistent_anim_update.m` | 58 |
| `MPC_Old/persistent_anim_update.m` | 65 |

**Mevcut hardcoded yol:**
```matlab
iconPath = '/Users/kasimesen/Desktop/Tasarım/car.png';
```

**Yapılması gereken:** `car.png` dosyasını projenin bulunduğu klasöre kopyalayın, ardından bu satırı dinamik yolla değiştirin:
```matlab
iconPath = fullfile(fileparts(mfilename('fullpath')), '..', 'car.png');
```
> `car.png` ana klasöre (`8. Week Birleştirme/`) konulursa `..` ile bir üst dizine çıkılır. Dosyayı her kontrolcünün alt klasörüne koyarsanız `'..'` kısmını kaldırın.

**Not:** Bu yol `try/catch` içine alınmıştır. Dosya bulunamazsa animasyon beyaz kare ile çalışmaya devam eder, simülasyon çökmez. Araç simgesi görünmeyecek ama tüm diğer grafikler ve metrikler etkilenmez.

---

## Sorun Giderme

| Hata | Neden | Çözüm |
|---|---|---|
| `Hicbir kontrolcu logu yok` | `results_*.mat` yok, workspace temiz | Simülasyonları çalıştırıp `export_run` kullanın |
| `sim_log_MPC bulunamadi` | `mpc_to_compare()` çağrılmadı | MPC simülasyonu bittikten sonra `mpc_to_compare()` çalıştırın |
| Metrikler `NaN` | Scope logu yok | Scope bloğu `Log to workspace` ayarını kontrol edin veya `sim_log` içindeki `ey/eh/delta` alanlarının dolu olduğunu doğrulayın |
| Yörüngeler çakışmıyor | Farklı haritalar seçilmiş | Üç scriptte de `map_type` değerlerini eşitleyin |
| `quadprog` hatası | Optimization Toolbox eksik | `ver` komutuyla toolbox varlığını doğrulayın |
