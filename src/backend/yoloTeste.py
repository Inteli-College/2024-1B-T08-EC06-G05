from ultralytics import YOLO
import cv2
from collections import defaultdict
import numpy as np

def main():

    cap = cv2.VideoCapture(0)
    model = YOLO("./home/grupo-05-t08/2024-1B-T08-EC06-G05/src/backend/runs/detect/train7/weights/best.pt")

    track_history = defaultdict(lambda: [])
    seguir = True
    deixar_rastro = True

    while True:
        success, img = cap.read()

        if success:
            if seguir:
                results = model.track(img, persist=True)
            else:
                results = model(img)

            # Process results list
            for result in results:
                # Visualize the results on the frame
                img = result.plot()

                if seguir and deixar_rastro:
                    try:
                        # Get the boxes and track IDs
                        boxes = result.boxes.xywh.cpu()
                        track_ids = result.boxes.id.int().cpu().tolist()

                        # Plot the tracks
                        for box, track_id in zip(boxes, track_ids):
                            x, y, w, h = box
                            track = track_history[track_id]
                            track.append((float(x), float(y)))  # x, y center point
                            if len(track) > 30:  # retain 90 tracks for 90 frames
                                track.pop(0)

                            # Draw the tracking lines
                            points = np.hstack(track).astype(np.int32).reshape((-1, 1, 2))
                    except:
                        pass

            cv2.imshow("Tela", img)

        k = cv2.waitKey(1)
        if k == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("desligando")

if __name__ == "__main__":
    main()
