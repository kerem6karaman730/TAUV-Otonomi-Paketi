import cv2
import numpy as np
import cv2.aruco as aruco
from pymavlink import mavutil
import time

# Otonomi Ayarları
Kp = 0.005  
MAX_SPEED = 0.5  
ILERI_HIZ = 0.3  

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
    print("Baglanti Basarili! Pipeline Görevi Basliyor...")

    master.set_mode('GUIDED')
    time.sleep(1)
    master.arducopter_arm()
    time.sleep(1)

    cap = cv2.VideoCapture(0)
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    center_x = int(width / 2)
    center_y = int(height / 2)

    # ArUco Dedektör Kurulumu (Görev Kuralı: Original ArUco)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Tespit edilen markerları tutacağımız liste
    bulunan_markerlar = []

    while True:
        ret, frame = cap.read()
        if not ret: break

        # 1. ARUCO TESPİTİ (Borunun üzerindeki markerları okuma)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                # Eğer marker ID'si 1-99 arasındaysa ve listemizde yoksa kaydet
                if 1 <= marker_id <= 99 and marker_id not in bulunan_markerlar:
                    bulunan_markerlar.append(marker_id)
                    print(f"!!! YENI MARKER TESPIT EDILDI: ID {marker_id} !!!")
                    print(f"Guncel Liste: {bulunan_markerlar}")

        # 2. SARI BORU TAKİBİ (Otonom Sürüş)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        alt_sari = np.array([20, 100, 100])
        ust_sari = np.array([40, 255, 255])
        maske = cv2.inRange(hsv, alt_sari, ust_sari)
        contours, _ = cv2.findContours(maske, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
        boru_bulundu = False

        if len(contours) > 0:
            en_buyuk_kontur = max(contours, key=cv2.contourArea)
            if cv2.contourArea(en_buyuk_kontur) > 500:
                boru_bulundu = True
                x, y, w, h = cv2.boundingRect(en_buyuk_kontur)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 255), 2)

                M = cv2.moments(en_buyuk_kontur)
                if M["m00"] != 0:
                    obje_x = int(M["m10"] / M["m00"])
                    obje_y = int(M["m01"] / M["m00"])

                    error_x = obje_x - center_x
                    hedef_hiz_y = sinirla(error_x * Kp, MAX_SPEED) 
                    hedef_hiz_x = ILERI_HIZ 

                    hiz_komutu_gonder(master, hedef_hiz_x, hedef_hiz_y, 0)

                    cv2.putText(frame, "BORU TAKIP & TARAMA", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    
        if not boru_bulundu:
            hiz_komutu_gonder(master, 0, 0, 0)
            cv2.putText(frame, "Boru Kaybedildi...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # Bulunan markerları ekranda liste olarak göster
        liste_metni = "Bulunan ID'ler: " + str(bulunan_markerlar)
        cv2.putText(frame, liste_metni, (20, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 100, 255), 2)

        cv2.imshow("TAUV - Pipeline Inspection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print(f"GOREV SONU RAPORU: Bulunan Marker Listesi: {bulunan_markerlar}")
            master.arducopter_disarm()
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()