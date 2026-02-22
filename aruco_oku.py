import cv2
import cv2.aruco as aruco
from pymavlink import mavutil
import time

# Otonomi Ayarları
Kp = 0.005 
MAX_SPEED = 0.5 
MERKEZ_HASSASIYETI = 60  

HEDEF_IDLER = [28, 7, 19, 96]

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
    print("Baglanti Basarili! Subsea Docking Protokolu Baslatiliyor...")

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

    # DOCKING ZAMANLAYICI DEĞİŞKENLERİ
    inis_basladi = False
    inis_baslama_zamani = 0
    gorev_tamamlandi = False

    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = detector.detectMarkers(gray)

        cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

        hedef_bulundu = False

        if ids is not None and not gorev_tamamlandi:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            
            for i in range(len(ids)):
                if ids[i][0] in HEDEF_IDLER:
                    hedef_bulundu = True
                    
                    marker_corners = corners[i][0] 
                    m_x = int(sum([c[0] for c in marker_corners]) / 4)
                    m_y = int(sum([c[1] for c in marker_corners]) / 4)
                    
                    cv2.circle(frame, (m_x, m_y), 5, (0, 0, 255), -1)
                    cv2.line(frame, (center_x, center_y), (m_x, m_y), (0, 255, 0), 2)
                    
                    error_x = m_x - center_x
                    error_y = m_y - center_y
                    
                    # Normal Merkezleme Hızları
                    hedef_hiz_y = sinirla(error_x * Kp, MAX_SPEED)   
                    hedef_hiz_x = sinirla(-error_y * Kp, MAX_SPEED)  
                    hedef_hiz_z = 0

                    # 10 SANİYE KURALI MANTIĞI
                    if abs(error_x) < MERKEZ_HASSASIYETI and abs(error_y) < MERKEZ_HASSASIYETI:
                        if not inis_basladi:
                            inis_basladi = True
                            inis_baslama_zamani = time.time() # Kronometreyi başlat
                        
                        gecen_sure = time.time() - inis_baslama_zamani
                        
                        if gecen_sure <= 10:
                            # 10 saniye boyunca aşağı itmeye devam et ki pucklar tam temas etsin
                            hedef_hiz_z = 0.2 
                            hedef_hiz_x = 0   
                            hedef_hiz_y = 0
                            cv2.putText(frame, f"DOCKING... Beklenen Sure: {int(gecen_sure)}/10 sn", (center_x - 200, center_y - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 3)
                        else:
                            # 10 saniye doldu! Görevi bitir.
                            hedef_hiz_z = 0
                            hedef_hiz_x = 0
                            hedef_hiz_y = 0
                            gorev_tamamlandi = True
                    else:
                        # Eğer araç 10 saniye dolmadan merkezden kayarsa sayacı sıfırla!
                        inis_basladi = False
                        
                    hiz_komutu_gonder(master, hedef_hiz_x, hedef_hiz_y, hedef_hiz_z)
                    
                    cv2.putText(frame, f"Sway(Y): {hedef_hiz_y:.2f}", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Surge(X): {hedef_hiz_x:.2f}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(frame, f"Heave(Z): {hedef_hiz_z:.2f}", (20, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                    break 
        
        if not hedef_bulundu and not gorev_tamamlandi:
            hiz_komutu_gonder(master, 0, 0, 0)
            inis_basladi = False # Hedef kaybolursa sayacı sıfırla
            cv2.putText(frame, "Hedef Istasyon Araniyor...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        if gorev_tamamlandi:
            hiz_komutu_gonder(master, 0, 0, 0)
            cv2.putText(frame, "GOREV BASARILI: 10 SN DOCKING TAMAMLANDI!", (20, height // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 3)

        cv2.imshow("TAUV - Subsea Docking Sistemi", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("Motorlar durduruluyor (DISARM).")
            master.arducopter_disarm()
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()