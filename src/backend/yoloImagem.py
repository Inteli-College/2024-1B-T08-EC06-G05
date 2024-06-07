import cv2
from ultralytics import YOLO

# Load the imag
image_path = "../data-base/imgs/img2.png"
img = cv2.imread(image_path)

# Load the YOLO model
model = YOLO("./home/grupo-05-t08/2024-1B-T08-EC06-G05/src/backend/runs/detect/train4/weights/best.pt")

def main(img):
    # Process the image
    results = model(img)

    # Process results list
    for result in results:
        # Visualize the results on the frame
        img = result.plot()

    # Save the result image
    output_path = "../data-base/imgs/img2_result.png"
    cv2.imwrite(output_path, img)
    print(f"Processed image saved as {output_path}")

if __name__ == "__main__":
    main(img)
