from ultralytics import YOLO
import cv2
from collections import defaultdict
import numpy as np
from PIL import ImageGrab
from Xlib import display

class WindowCapture:
    def __init__(self, size=None, origin=None):
        if size and origin:
            self.bbox = (origin[0], origin[1], origin[0] + size[0], origin[1] + size[1])
        else:
            raise ValueError("Você deve especificar tanto o tamanho quanto a origem.")

    def get_screenshot(self):
        screenshot = ImageGrab.grab(bbox=self.bbox)
        return cv2.cvtColor(np.array(screenshot), cv2.COLOR_RGB2BGR)

# Carrega a imagem estática
image_path = "../data-base/imgs/img1.jpeg"
img = cv2.imread(image_path)

if img is None:
    print(f"Erro ao carregar a imagem do caminho: {image_path}")
    exit()

# Usa modelo da Yolo
model = YOLO("./home/grupo-05-t08/2024-1B-T08-EC06-G05/src/backend/runs/detect/train3/weights/best.pt")

track_history = defaultdict(lambda: [])
seguir = True
deixar_rastro = True

# Processa a imagem estática
if seguir:
    results = model.track(img, persist=True)
else:
    results = model(img)

# Verifica se os resultados estão disponíveis
if not results:
    print("Nenhum resultado foi retornado pelo modelo.")
    exit()

# Processa os resultados
for result in results:
    # Verifica se há caixas detectadas
    if not hasattr(result, 'boxes') or result.boxes is None:
        print("Nenhuma caixa detectada.")
        continue
    
    # Visualiza os resultados na imagem
    img = result.plot()

    if seguir and deixar_rastro:
        try:
            # Obtém as caixas e IDs de rastreamento
            boxes = result.boxes.xywh.cpu().numpy()  # Converte para numpy array
            track_ids = result.boxes.id.int().cpu().tolist()

            # Plota as trilhas
            for box, track_id in zip(boxes, track_ids):
                x, y, w, h = box
                track = track_history[track_id]
                track.append((float(x), float(y)))  # ponto central x, y
                if len(track) > 30:  # mantém 30 trilhas para 30 quadros
                    track.pop(0)

                # Desenha as linhas de rastreamento
                points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(img, [points], isClosed=False, color=(230, 0, 0), thickness=5)
        except Exception as e:
            print(f"Erro ao processar trilhas: {e}")

# Exibe a imagem resultante
cv2.imshow("Resultado", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
print("Processo concluído")