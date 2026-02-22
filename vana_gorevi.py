import cv2
import numpy as np
from pymavlink import mavutil
import time

# Otonomi Ayarları
Kp = 0.005  
MAX_SPEED = 0.4  
HEDEF_ALAN = 80000  # 0.5 metreyi simüle eden piksel büyüklüğü (Kameraya göre değişebilir)
ALAN_TOLERANSI = 10000 # İleri-geri titremeyi önlemek için hata payı

def sinirla(deger, max_deger):
    if deger > max_deger: return max_deger
    if deger < -max_deger: return -max_deger
    return deger

def hiz_komutu_gonder(master, hiz_x, hiz_y, hiz_z=0):
    master.mav.set_position_target_local_ned_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        int(0b0000111111000111), 
        0, 0, 0,
        hiz_x, hiz_y, hiz_z, 
        0, 0, 0,
        0, 0)

def main():
    print("Sanal Denizalti araniyor...")
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
    master.wait_heartbeat()
    print("Baglanti Basarili! Valve Intervention Protokolu Basliyor...")

    master.set_mode('GUIDED')
    time.sleep(1)
    master.arducopter_arm()
    time.sleep(1)

    cap = cv2.VideoCapture(0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center_x = int(width / 2)
    center_y = int(height / 2)

    while True:
        ret, frame = cap.read()
        if not ret: break

        # Altın sarısı yapı için filtre (RAL 1004)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        alt_sari = np.array([15, 100, 100])
        ust_sari = np.array([45, 255, 255])
        maske = cv2.inRange(hsv, alt_sari, ust_sari)
        
        contours, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
        yapi_bulundu = False

        if len(contours) > 0:
            en_buyuk_kontur = max(contours, key=cv2.contourArea)
            guncel_alan = cv2.contourArea(en_buyuk_kontur)
            
            if guncel_alan > 1000:  # Ufak parlamaları yoksay
                yapi_bulundu = True
                x, y, w, h = cv2.boundingRect(en_buyuk_kontur)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 165, 255), 3)

                M = cv2.moments(en_buyuk_kontur)
                if M["m00"] != 0:
                    obje_x = int(M["m10"] / M["m00"])
                    obje_y = int(M["m01"] / M["m00"])

                    cv2.circle(frame, (obje_x, obje_y), 5, (0, 0, 255), -1)
                    cv2.line(frame, (center_x, center_y), (obje_x, obje_y), (0, 255, 0), 2)

                    # Merkezleme (3D Otonomi - X ve Y ekseni)
                    error_x = obje_x - center_x
                    error_y = obje_y - center_y
                    
                    hedef_hiz_y = sinirla(error_x * Kp, MAX_SPEED)  # Sway
                    hedef_hiz_z = sinirla(error_y * Kp, MAX_SPEED)  # Heave

                    # MESAFE KONTROLÜ (Surge - Z ekseni büyüklüğüne göre)
                    if guncel_alan < (HEDEF_ALAN - ALAN_TOLERANSI):
                        hedef_hiz_x = 0.25 # Yapı çok küçük, demek ki uzaktayız -> İleri Git
                        durum_metni = "YAKLASILIYOR..."
                        renk = (0, 255, 255)
                    elif guncel_alan > (HEDEF_ALAN + ALAN_TOLERANSI):
                        hedef_hiz_x = -0.25 # Yapı çok büyük, çok dibine girdik -> Geri Git
                        durum_metni = "COK YAKIN! GERI CIKILIYOR"
                        renk = (0, 0, 255)
                    else:
                        hedef_hiz_x = 0 # Tam 0.5m mesafedeyiz -> Dur ve Kilitle
                        hedef_hiz_y = 0 
                        hedef_hiz_z = 0
                        durum_metni = "0.5m KORUNUYOR - VANAYA KILITLENDI!"
                        renk = (0, 255, 0)
                        # Otonom manipülatör (kol) kodları buraya eklenebilir!

                    hiz_komutu_gonder(master, hedef_hiz_x, hedef_hiz_y, hedef_hiz_z)

                    cv2.putText(frame, durum_metni, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, renk, 2)
                    cv2.putText(frame, f"Alan (Mesafe): {int(guncel_alan)} / Hedef: {HEDEF_ALAN}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    cv2.putText(frame, f"Surge: {hedef_hiz_x:.2f} | Sway: {hedef_hiz_y:.2f} | Heave: {hedef_hiz_z:.2f}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        if not yapi_bulundu:
            hiz_komutu_gonder(master, 0, 0, 0)
            cv2.putText(frame, "Sari Yapi Araniyor...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("TAUV - Valve Intervention", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Motorlar durduruluyor (DISARM).")
            master.arducopter_disarm()
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()