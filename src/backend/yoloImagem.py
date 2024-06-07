import os
import cv2
from ultralytics import YOLO

def process_image(image_path, output_dir="../data-base/imgs_results"):
    
    image_path = "../data-base/imgs/img2.png"
    # Load the image
    img = cv2.imread(image_path)

    # Check if the image was loaded successfully
    if img is None:
        print(f"Error: Unable to load image at {image_path}")
        return

    # Load the YOLO model
    model = YOLO("./home/grupo-05-t08/2024-1B-T08-EC06-G05/src/backend/runs/detect/train4/weights/best.pt")

    # Process the image
    results = model(img)

    # Process results list
    for result in results:
        # Visualize the results on the frame
        img = result.plot()

    # Ensure the output directory exists
    os.makedirs(output_dir, exist_ok=True)

    # Create the output path
    base_name = os.path.basename(image_path)
    name, ext = os.path.splitext(base_name)
    output_path = os.path.join(output_dir, f"{name}_result{ext}")

    # Save the result image
    cv2.imwrite(output_path, img)
    print(f"Processed image saved as {output_path}")

if __name__ == "__main__":
    image_path = "../data-base/imgs/img2.png"  # Replace with your image path
    process_image(image_path)