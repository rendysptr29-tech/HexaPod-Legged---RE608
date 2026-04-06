# Forward & Inverse Kinematics — 3 Joint Leg Simulation

> Simulasi kaki 3 sendi menggunakan metode geometri untuk **Forward Kinematics (FK)** dan **Inverse Kinematics (IK)** berbasis Python + Matplotlib.

---

## Konfigurasi Segmen Kaki

| Segmen | Variabel | Panjang | Fungsi |
|--------|----------|---------|--------|
| Coxa   | `L1`     | 4       | Rotasi horizontal pangkal kaki |
| Femur  | `L2`     | 8       | Elevasi naik/turun kaki |
| Tibia  | `L3`     | 10      | Tekukan ujung kaki ke target |

---

## Forward Kinematics (FK)

> **Konsep:** FK bekerja dari **sudut → posisi**. Diberikan nilai θ₁, θ₂, θ₃, FK menghitung koordinat (x, y, z) setiap joint secara berantai dari pangkal ke ujung kaki.

### Rumus

<img width="1201" height="481" alt="image" src="https://github.com/user-attachments/assets/300eee70-9aa9-44d1-9c73-b267a6454ad4" />

**Joint 1 — Coxa**
```
x₁ = L₁ · cos(θ₁)
y₁ = L₁ · sin(θ₁)
z₁ = 0
```

**Joint 2 — Femur**
```
x₂ = x₁ + L₂ · cos(θ₁) · cos(θ₂)
y₂ = y₁ + L₂ · sin(θ₁) · cos(θ₂)
z₂ = L₂ · sin(θ₂)
```

**Joint 3 — Tibia / End-effector**
```
x₃ = x₂ + L₃ · cos(θ₁) · cos(θ₂ + θ₃)
y₃ = y₂ + L₃ · sin(θ₁) · cos(θ₂ + θ₃)
z₃ = z₂ + L₃ · sin(θ₂ + θ₃)
```

---

### Penjelasan Rumus FK

**Joint 1 — Coxa** `x₁, y₁, z₁`

Coxa adalah segmen pertama yang terhubung langsung ke titik asal `(0, 0, 0)`. Karena coxa hanya berputar secara horizontal di bidang XY, posisinya cukup dihitung dengan trigonometri dasar. `x₁ = L₁·cos(θ₁)` dan `y₁ = L₁·sin(θ₁)` menguraikan panjang L₁ ke arah X dan Y berdasarkan sudut rotasi θ₁. Nilai `z₁ = 0` karena coxa tidak memiliki komponen vertikal — ia hanya berputar mendatar.

**Joint 2 — Femur** `x₂, y₂, z₂`

Femur dimulai dari ujung coxa `(x₁, y₁, z₁)`, lalu ditambah kontribusi segmen L₂. Femur bergerak dalam dua sumbu sekaligus: **horizontal** mengikuti arah θ₁, dan **vertikal** berdasarkan sudut elevasi θ₂. Rumusnya mengalikan `cos(θ₁)·cos(θ₂)` untuk komponen X/Y — `cos(θ₂)` menyatakan seberapa besar proyeksi femur ke bidang horizontal. Sedangkan `z₂ = L₂·sin(θ₂)` menyatakan ketinggian yang dicapai femur akibat sudut elevasinya.

**Joint 3 — Tibia / End-effector** `x₃, y₃, z₃`

Tibia dimulai dari ujung femur dan ditambah kontribusi L₃. Sudut yang digunakan adalah `(θ₂ + θ₃)` — yaitu sudut kumulatif femur ditambah tekukan tibia sendiri, karena tibia terhubung ke femur dan sudutnya bersifat relatif. `cos(θ₂+θ₃)` menentukan proyeksi horizontal tibia, sedangkan `sin(θ₂+θ₃)` menentukan tambahan ketinggian. Hasilnya `(x₃, y₃, z₃)` adalah posisi akhir ujung kaki yang digambar sebagai **titik merah** di simulasi.

---

### Implementasi Kode — FK

```python
def forward_kinematics(theta1, theta2, theta3):

    # Joint 1 - Coxa (rotasi horizontal)
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    z1 = 0

    # Joint 2 - Femur (elevasi vertikal)
    x2 = x1 + L2 * np.cos(theta1) * np.cos(theta2)
    y2 = y1 + L2 * np.sin(theta1) * np.cos(theta2)
    z2 = L2 * np.sin(theta2)

    # Joint 3 - Tibia / End-effector
    x3 = x2 + L3 * np.cos(theta1) * np.cos(theta2 + theta3)
    y3 = y2 + L3 * np.sin(theta1) * np.cos(theta2 + theta3)
    z3 = z2 + L3 * np.sin(theta2 + theta3)

    return (x1,y1,z1), (x2,y2,z2), (x3,y3,z3)
```

---

### Pengaruh ke 3 Joint Saat Simulasi (FK)

| Joint | Sudut | Pengaruh di Simulasi |
|-------|-------|----------------------|
| Coxa | `θ₁` | Menentukan arah horizontal seluruh kaki. Mengubah `theta1` memutar rantai kaki mengelilingi sumbu Z — terlihat sebagai perubahan arah **garis biru** di plot. |
| Femur | `θ₂` | Mengatur naik/turunnya segmen tengah. Nilai positif mengangkat kaki ke atas, negatif menjulurkan ke bawah — terlihat sebagai perubahan ketinggian **garis oranye**. |
| Tibia | `θ₃` | Menentukan tekukan ujung kaki relatif terhadap femur. Mempengaruhi seberapa jauh end-effector bisa menjangkau — terlihat sebagai perubahan posisi **garis ungu**. |

---

## Inverse Kinematics (IK)

> **Konsep:** IK adalah kebalikan dari FK — diberikan posisi target (x, y, z), IK menghitung θ₁, θ₂, θ₃ yang dibutuhkan agar ujung kaki tepat mencapai titik tersebut menggunakan pendekatan geometri analitik dan *law of cosines*.

### Rumus

<img width="1536" height="827" alt="image" src="https://github.com/user-attachments/assets/74602698-5502-41b2-aec0-d1116eca9967" />

**Step 1 — θ₁ : Sudut rotasi horizontal coxa**
```
θ₁ = atan2(y, x)
```

**Step 2 — r : Jarak planar efektif**
```
r = √(x² + y²) − L₁
```

**Step 3 — D : Law of cosines**
```
D = (r² + z² − L₂² − L₃²) / (2 · L₂ · L₃)
```

**Step 4 — θ₃ : Sudut tekukan tibia**
```
θ₃ = atan2(−√(1 − D²), D)     ← konfigurasi elbow-down
```

**Step 5 — θ₂ : Sudut elevasi femur**
```
θ₂ = atan2(z, r) − atan2(L₃·sin θ₃, L₂ + L₃·cos θ₃)
```

---

### Penjelasan Rumus IK

**Step 1 — θ₁** : Sudut rotasi horizontal coxa

Langkah pertama adalah menentukan ke mana kaki harus menghadap. `atan2(y, x)` menghitung sudut antara titik target dan sumbu X di bidang horizontal. Fungsi `atan2` digunakan (bukan arctan biasa) karena ia mampu menangani semua kuadran — target di depan, belakang, kiri, maupun kanan — tanpa ambiguitas tanda. Hasilnya langsung menjadi θ₁ untuk joint coxa.

**Step 2 — r** : Jarak planar efektif

`√(x²+y²)` menghitung jarak horizontal total dari titik asal ke target, lalu dikurangi L₁ (panjang coxa) karena coxa sudah "memakan" sebagian jarak tersebut. Nilai `r` adalah jarak efektif yang harus dijangkau oleh kombinasi femur dan tibia di bidang vertikal — bukan dari titik asal, melainkan dari pangkal femur.

**Step 3 — D** : Law of cosines

D adalah nilai cosinus sudut yang dibentuk antara femur dan tibia, dihitung menggunakan **hukum cosinus segitiga**. Segitiga yang dimaksud dibentuk oleh segmen femur (L₂), tibia (L₃), dan jarak lurus dari pangkal femur ke target `√(r²+z²)`. Pembagi `2·L₂·L₃` adalah normalisasi standar dari hukum cosinus. Nilai D kemudian di-clamp ke `[-1, 1]` agar tidak menghasilkan `NaN` saat target berada di batas atau di luar jangkauan kaki.

**Step 4 — θ₃** : Sudut tekukan tibia

Setelah D diketahui, θ₃ dihitung dengan `atan2(−√(1−D²), D)`. Komponen `√(1−D²)` adalah nilai sinus dari sudut tersebut (identitas sin²+cos²=1). Tanda **minus** pada sinus adalah kunci konfigurasi — ia memaksa tibia selalu menekuk ke arah bawah (**elbow-down**), yang merupakan posisi natural kaki saat berdiri dan melangkah.

**Step 5 — θ₂** : Sudut elevasi femur

`atan2(z, r)` menghitung sudut total yang dibutuhkan untuk mencapai ketinggian z dari jarak r — yaitu sudut bidang vertikal ke target. Kemudian dikurangi `atan2(L₃·sinθ₃, L₂+L₃·cosθ₃)` yang menyatakan sudut yang sudah "dipakai" oleh tibia. Selisihnya adalah porsi sudut yang harus diambil femur agar total rantai persis sampai ke target. θ₂ dihitung terakhir karena bergantung pada θ₃ yang sudah diketahui.

---

### Implementasi Kode — IK

```python
def inverse_kinematics(x, y, z):

    # Step 1: θ₁ - rotasi horizontal
    theta1 = np.arctan2(y, x)

    # Step 2: jarak planar setelah dikurangi coxa
    r = np.sqrt(x**2 + y**2) - L1

    # Step 3: law of cosines → D
    D = (r**2 + z**2 - L2**2 - L3**2) / (2 * L2 * L3)
    D = np.clip(D, -1.0, 1.0)   # clamp agar tidak NaN

    # Step 4: θ₃ - konfigurasi elbow-down
    theta3 = np.arctan2(-np.sqrt(1 - D**2), D)

    # Step 5: θ₂ - sudut femur (dihitung terakhir)
    theta2 = np.arctan2(z, r) \
           - np.arctan2(L3 * np.sin(theta3),
                        L2 + L3 * np.cos(theta3))

    return theta1, theta2, theta3
```

---

### Pengaruh ke 3 Joint Saat Simulasi (IK)

| Joint | Sudut | Pengaruh di Simulasi |
|-------|-------|----------------------|
| Coxa | `θ₁` | Dihitung dari `atan2(y, x)` target. Setiap frame, coxa otomatis berputar menghadap posisi target yang bergerak sehingga bidang gerak kaki selalu sejajar arah tujuan. |
| Femur | `θ₂` | Dihitung terakhir setelah `theta3` diketahui. Menyesuaikan sudut femur agar kombinasi femur + tibia tepat menjangkau ketinggian z target — berubah setiap frame seiring target naik-turun. |
| Tibia | `θ₃` | Dihitung dari D via law of cosines. Tanda minus memaksa tibia selalu menekuk ke bawah (elbow-down). `np.clip` menjaga D agar kaki tidak patah saat target mendekati batas jangkauan. |

---

## Hubungan FK dan IK dalam Simulasi

FK dan IK berjalan sebagai **pipeline satu arah** di setiap frame animasi — tidak ada tabrakan atau konflik antar keduanya.

```
Target (x, y, z)
      │
      ▼
┌─────────────┐
│     IK      │  → Hitung θ₁, θ₂, θ₃ dari posisi target
└─────────────┘
      │
      ▼
┌─────────────┐
│     FK      │  → Hitung posisi (x,y,z) tiap joint dari sudut
└─────────────┘
      │
      ▼
Gambar segmen kaki di plot 3D
```

```python
def update(frame):
    # 1. Hitung posisi target baru tiap frame
    target_x = 8 + 3*np.cos(t)
    target_y = 3*np.sin(t)
    target_z = -10 + 2*np.sin(t)

    # 2. IK jalan duluan → dapat sudut joint
    theta1, theta2, theta3 = inverse_kinematics(target_x, target_y, target_z)

    # 3. FK jalan setelahnya → dapat posisi tiap joint
    (x1,y1,z1),(x2,y2,z2),(x3,y3,z3) = forward_kinematics(theta1, theta2, theta3)

    # 4. Gambar segmen kaki dari hasil FK
    ax.plot([0,x1],[0,y1],[0,z1], color='blue')      # Coxa
    ax.plot([x1,x2],[y1,y2],[z1,z2], color='orange') # Femur
    ax.plot([x2,x3],[y2,y3],[z2,z3], color='purple') # Tibia
```

> **Catatan:** IK dan FK adalah fungsi murni (input → output) tanpa state bersama, sehingga aman dijalankan berurutan setiap frame tanpa risiko konflik.

---

## Simulasi 3 Joint FK & IK 

![0406](https://github.com/user-attachments/assets/1cea418a-2397-412e-a25f-34b55223bd81) ![Video Simulasi 2](https://github.com/user-attachments/assets/cc0653fd-26be-4a20-9151-d9d9392f7923)



