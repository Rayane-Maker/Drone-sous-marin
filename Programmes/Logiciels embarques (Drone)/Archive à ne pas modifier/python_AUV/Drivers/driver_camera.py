import time
import cv2
import datetime

def start_n_save_cam(visu = False,record = False):
    # Initialiser la webcam
    cap = cv2.VideoCapture(0)

    # Définir les dimensions du flux vidéo
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Initialiser l'objet VideoWriter pour enregistrer la séquence
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    date_now = datetime.datetime.now().strftime("%d_%m_%Y__%H_%M_%S")
    out = ""
    if record:
        out = cv2.VideoWriter("Desktop/Enregistrements/video_mission_"+date_now+".avi", fourcc, 20.0, (width, height))
    stop = True
    t0 = time.time()
    while stop:
        # Lire un frame de la webcam
        ret, frame = cap.read()

        # Ajouter le frame modifié à la séquence enregistrée
        if record:
            out.write(frame)
        if visu:
            cv2.imshow("video",frame)

        # Si l'utilisateur appuie sur 'q', quitter la boucle
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        if time.time() - t0 > 100:
            stop = False

    cap.release()
    if record:
        out.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    start_n_save_cam(visu = True,record = True)
