import cv2
import sys


def main(input_image_path, output_image_path):
    # Read the image
    image = cv2.imread(input_image_path)

    # Check if image loading was successful
    if image is None:
        print(f"Error loading image {input_image_path}")
        sys.exit(1)

    # Process the image with OpenCV, e.g., applying a Gaussian blur
    blurred_image = cv2.GaussianBlur(image, (5, 5), 0)

    # Save the processed image
    cv2.imwrite(output_image_path, blurred_image)
    print(f"Image saved to {output_image_path}")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python process_image.py <input_image_path> <output_image_path>")
        sys.exit(1)

    input_image_path = sys.argv[1]
    output_image_path = sys.argv[2]

    main(input_image_path, output_image_path)
