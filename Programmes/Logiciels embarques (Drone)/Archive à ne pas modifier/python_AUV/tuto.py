import Drivers.driver_motors as dm
import Drivers.driver_IMU as dimu
import Drivers.driver_camera as dcam
import time

"""
#récupération de la camera et enregistrement de la vidéo
#les paramètres sont : visu = affiche le video, record = enregistre la video sur le bureau dans le dossier "enregistrements" 
dcam.start_n_save_cam(visu = True,record = True) #par exemple, la on lance la camera, affiche la video et sauvegarde la video
"""

"""
#recupération des données IMU
IMU_data = dimu.get_IMU_data() #on obtient un dictionnaire avec les valeurs voulues sous le forme :
#{'heading': 193.625, 'roll': -66.6875, 'pitch': 115.125, 'temperature': 25, 'gyro_x': 0.0033333333333333335, 'gyro_y': 0.0011111111111111111, 'gyro_z': -0.0033333333333333335, 'acc_x': -0.05, 'acc_y': 0.03, 'acc_z': 0.11}

while True:
    IMU_data = dimu.get_IMU_data()
    print(IMU_data)
    time.sleep(1)
"""

"""
#control des moteurs
dm.arm_motors() #on arm les propulseurs les variateurs devraient bipper
dm.set_motors(100,100,100,100) #on leurs donne une vitesse entre -500 et 500 (valeurs négatives = marche arrière), dans cet ordre : moteur avant, moteur arrière, moteur gauche, moteur droite
time.sleep(4)
dm.set_motors(0,0,0,0)
time.sleep(4)
dm.set_motors(-100,-100,-100,-100)
time.sleep(4)
dm.disarm_motors() #on désarm les propulseurs
"""
