import cv2
import numpy as np
import cv2.aruco as aruco
from pymavlink import mavutil
import time

# --- OTONOMİ AYARLARI ---
Kp = 0.005  
MAX_SPEED = 0.4  
ILERI_HIZ = 0.3  
HEDEF_ALAN = 80000     # Vana için 0.5m mesafesi
ALAN_TOLERANSI = 10000 
DOCKING_IDLER = [28, 7, 19, 96]
HEDEF_DERINLIK = 1.5   # Dalış görevi için hedef derinlik (metre)

def sinirla(deger, max_deger):
    if deger > max_deger: return max_deger
    if deger < -max_deger: return -max_deger
    return deger

def hiz_komutu_gonder(master, hiz_x, hiz_y, hiz_z=0):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        int(0b0000111111000111), 
        0, 0, 0, hiz_x, hiz_y, hiz_z, 0, 0, 0, 0, 0)

def main():
    print("Sanal Denizalti araniyor...")
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()
    print("Baglanti Basarili! TAUV Ana Otonomi Sistemi Baslatiliyor...")

    master.set_mode('GUIDED')
    time.sleep(1)
    master.arducopter_arm()
    time.sleep(1)

    cap = cv2.VideoCapture(0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center_x = int(width / 2)
    center_y = int(height / 2)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # --- DURUM (STATE) DEĞİŞKENLERİ ---
    guncel_durum = 0 # YENİ: 0: Dalış, 1: Pipeline, 2: Valve, 3: Docking, 4: Finish
    
    # Hafıza Listeleri
    boru_marker_listesi = []
    vana_marker_listesi = []
    
    # Sensör Değişkenleri
    anlik_derinlik = 0.0
    anlik_pusula = 0
    
    # GÖREV 1 KONTROLLERİ
    boru_hic_goruldu_mu = False  
    boru_kayip_baslangici = 0
    
    # GÖREV 2 KONTROLLERİ
    vana_kilit_baslangici = 0
    
    # GÖREV 3 KONTROLLERİ
    docking_inis_baslangici = 0
    docking_basladi = False

    while True:
        ret, frame = cap.read()
        if not ret: break

        # --- TELEMETRİ OKUMA (Sensör Verileri) ---
        msg = master.recv_match(type='VFR_HUD', blocking=False)
        if msg:
            anlik_derinlik = abs(msg.alt) # ArduSub'da alt genelde negatif gelir, mutlak değerini alıyoruz
            anlik_pusula = msg.heading

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)
        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

        hedef_hiz_x, hedef_hiz_y, hedef_hiz_z = 0, 0, 0
        durum_metni = ""

        # ==========================================
        # DURUM 0: DALIS (DIVE)
        # ==========================================
        if guncel_durum == 0:
            durum_metni = f"GOREV 0: DALIS ({HEDEF_DERINLIK}m BEKLENIYOR)"
            hedef_hiz_x, hedef_hiz_y = 0, 0
            hedef_hiz_z = 0.3 # Z ekseninde aşağı inme hızı
            
            # Telemetri kontrolü ile geçiş
            if anlik_derinlik >= HEDEF_DERINLIK:
                print(f"{HEDEF_DERINLIK} Metre derinlige ulasildi! DURUM 1'e geciliyor...")
                guncel_durum = 1
                hedef_hiz_z = 0 # Dalışı durdur

        # ==========================================
        # DURUM 1: PIPELINE INSPECTION
        # ==========================================
        elif guncel_durum == 1:
            durum_metni = "GOREV 1: PIPELINE INSPECTION"
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):
                    m_id = int(ids[i][0])
                    if 1 <= m_id <= 99 and m_id not in boru_marker_listesi:
                        boru_marker_listesi.append(m_id)

            maske = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([40, 255, 255]))
            contours, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            boru_bulundu = False
            if len(contours) > 0:
                en_buyuk = max(contours, key=cv2.contourArea)
                if cv2.contourArea(en_buyuk) > 500:
                    boru_bulundu = True
                    boru_hic_goruldu_mu = True 
                    boru_kayip_baslangici = 0  
                    
                    x, y, w, h = cv2.boundingRect(en_buyuk)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)
                    M = cv2.moments(en_buyuk)
                    if M["m00"] != 0:
                        obje_x = int(M["m10"] / M["m00"])
                        error_x = obje_x - center_x
                        hedef_hiz_y = sinirla(error_x * Kp, MAX_SPEED) 
                        hedef_hiz_x = ILERI_HIZ 

            if not boru_bulundu:
                if not boru_hic_goruldu_mu:
                    cv2.putText(frame, "Sari Boru Bekleniyor...", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                else:
                    if boru_kayip_baslangici == 0:
                        boru_kayip_baslangici = time.time()
                    
                    gecen_kayip_sure = time.time() - boru_kayip_baslangici
                    cv2.putText(frame, f"Boru Kayip! Gecis icin: {3 - int(gecen_kayip_sure)}sn", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                    
                    if gecen_kayip_sure > 3.0:
                        print("Boru hatti bitti. DURUM 2'ye (Valve) geciliyor...")
                        guncel_durum = 2

            cv2.putText(frame, f"Boru ID'ler: {boru_marker_listesi}", (20, height - 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)

        # ==========================================
        # DURUM 2: VALVE INTERVENTION & VISUAL INSPECTION
        # ==========================================
        elif guncel_durum == 2:
            durum_metni = "GOREV 2: VALVE & VISUAL INSPECTION"
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):
                    m_id = int(ids[i][0])
                    if 1 <= m_id <= 99 and m_id not in vana_marker_listesi:
                        vana_marker_listesi.append(m_id)

            maske = cv2.inRange(hsv, np.array([15, 100, 100]), np.array([45, 255, 255]))
            contours, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            yapi_bulundu = False
            if len(contours) > 0:
                en_buyuk = max(contours, key=cv2.contourArea)
                guncel_alan = cv2.contourArea(en_buyuk)
                
                if guncel_alan > 1000:
                    yapi_bulundu = True
                    x, y, w, h = cv2.boundingRect(en_buyuk)
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 165, 255), 3)
                    M = cv2.moments(en_buyuk)
                    if M["m00"] != 0:
                        obje_x, obje_y = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])
                        
                        hedef_hiz_y = sinirla((obje_x - center_x) * Kp, MAX_SPEED)  
                        hedef_hiz_z = sinirla((obje_y - center_y) * Kp, MAX_SPEED)  

                        if guncel_alan < (HEDEF_ALAN - ALAN_TOLERANSI):
                            hedef_hiz_x = 0.25 
                            vana_kilit_baslangici = 0
                            cv2.putText(frame, "YAPIYA YAKLASILIYOR", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                        elif guncel_alan > (HEDEF_ALAN + ALAN_TOLERANSI):
                            hedef_hiz_x = -0.25 
                            vana_kilit_baslangici = 0
                            cv2.putText(frame, "COK YAKIN! GERI CIKILIYOR", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                        else:
                            hedef_hiz_x, hedef_hiz_y, hedef_hiz_z = 0, 0, 0
                            
                            if vana_kilit_baslangici == 0:
                                vana_kilit_baslangici = time.time()
                                
                            kilit_suresi = time.time() - vana_kilit_baslangici
                            cv2.putText(frame, f"VANA CEVRILIYOR: {int(kilit_suresi)}/5 sn", (center_x - 150, center_y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)
                            
                            if kilit_suresi > 5.0:
                                print("Vana operasyonu tamamlandi! DURUM 3'e (Docking) geciliyor...")
                                guncel_durum = 3

            if not yapi_bulundu:
                vana_kilit_baslangici = 0
                cv2.putText(frame, "Sari Yapi Araniyor...", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(frame, f"Yapi ID'ler: {vana_marker_listesi}", (20, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)

        # ==========================================
        # DURUM 3: SUBSEA DOCKING
        # ==========================================
        elif guncel_durum == 3:
            durum_metni = "GOREV 3: SUBSEA DOCKING"
            hedef_bulundu = False

            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                for i in range(len(ids)):
                    if ids[i][0] in DOCKING_IDLER:
                        hedef_bulundu = True
                        marker_corners = corners[i][0] 
                        m_x = int(sum([c[0] for c in marker_corners]) / 4)
                        m_y = int(sum([c[1] for c in marker_corners]) / 4)
                        
                        error_x = m_x - center_x
                        error_y = m_y - center_y
                        
                        hedef_hiz_y = sinirla(error_x * Kp, MAX_SPEED)   
                        hedef_hiz_x = sinirla(-error_y * Kp, MAX_SPEED)  
                        hedef_hiz_z = 0

                        if abs(error_x) < 60 and abs(error_y) < 60:
                            if not docking_basladi:
                                docking_basladi = True
                                docking_inis_baslangici = time.time()
                            
                            gecen_sure = time.time() - docking_inis_baslangici
                            if gecen_sure <= 10:
                                hedef_hiz_x, hedef_hiz_y = 0, 0
                                hedef_hiz_z = 0.2 
                                cv2.putText(frame, f"DOCKING... Beklenen: {int(gecen_sure)}/10 sn", (center_x - 150, center_y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 3)
                            else:
                                print("Docking tamamlandi! DURUM 4'e (Bitis) geciliyor...")
                                guncel_durum = 4
                        else:
                            docking_basladi = False

            if not hedef_bulundu:
                docking_basladi = False
                cv2.putText(frame, "Docking Istasyonu Araniyor...", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # ==========================================
        # DURUM 4: FINISH
        # ==========================================
        elif guncel_durum == 4:
            durum_metni = "TUM GOREVLER TAMAMLANDI! DISARM BEKLENIYOR."
            hedef_hiz_x, hedef_hiz_y, hedef_hiz_z = 0, 0, 0
            
            cv2.putText(frame, f"Gorev Sonu Raporu:", (20, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(frame, f"Boru ID'leri: {boru_marker_listesi}", (20, (height // 2) + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f"Vana ID'leri: {vana_marker_listesi}", (20, (height // 2) + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

        # Hızları araca gönder
        hiz_komutu_gonder(master, hedef_hiz_x, hedef_hiz_y, hedef_hiz_z)
        
        # --- EKRAN BİLGİLERİ VE TELEMETRİ HUD ---
        cv2.putText(frame, durum_metni, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(frame, f"X:{hedef_hiz_x:.2f} Y:{hedef_hiz_y:.2f} Z:{hedef_hiz_z:.2f}", (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Canlı Telemetriyi Sağ Üste Ekle
        cv2.putText(frame, f"Derinlik: {anlik_derinlik:.2f} m", (width - 250, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, f"Yaw: {anlik_pusula} deg", (width - 250, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        cv2.imshow("TAUV - Ana Otonomi Sistemi", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Acil Durum (Q): Motorlar durduruluyor (DISARM).")
            master.arducopter_disarm()
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()