import os
import sys
import numpy as np

EMNIST_CLASSES = [
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    'a', 'b', 'd', 'e', 'f', 'g', 'h', 'n', 'q', 'r', 't'
]

# Lowercase letters that are merged in EMNIST ByMerge (mapped to uppercase)
MERGED_LOWERCASE = ['c', 'i', 'j', 'k', 'l', 'm', 'o', 'p', 's', 'u', 'v', 'w', 'x', 'y', 'z']

def render_terminal(img):
    # Ensure range [0, 255]
    max_val = np.max(img)
    if max_val <= 1.0:
        img = img * 255.0
        
    # Standard ASCII shading characters that work perfectly in any encoding (e.g. cp950)
    shades = " .:-=+*#%@"
    print("\n--- Image Rendered in Terminal ---")
    for r in range(28):
        row_str = ""
        for c in range(28):
            val = img[r, c]
            # Map val to [0, len(shades) - 1]
            shade_idx = min(int(val / 256.0 * len(shades)), len(shades) - 1)
            # Print twice to compensate for terminal character aspect ratio
            row_str += shades[shade_idx] * 2
        print(row_str)
    print("----------------------------------\n")

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    npz_path = os.path.join(script_dir, "..", "emnist_28x28.npz")
    arduino_dir = os.path.join(script_dir, "..", "arduino")
    input_h_path = os.path.join(arduino_dir, "input.h")
    
    if not os.path.exists(npz_path):
        print(f"Error: Dataset {npz_path} not found.")
        sys.exit(1)
        
    # Prompt the user for a character
    if len(sys.argv) > 1:
        user_input = sys.argv[1]
    else:
        try:
            user_input = input("Enter a character (0-9, A-Z, a-z) to load: ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting.")
            sys.exit(0)
            
    if len(user_input) != 1:
        print("Error: Please enter exactly one alphanumeric character.")
        sys.exit(1)
        
    char = user_input
    # Handle merged lowercase characters
    if char in MERGED_LOWERCASE:
        print(f"Note: Lowercase '{char}' is merged in EMNIST ByMerge. Mapping to uppercase '{char.upper()}'.")
        char = char.upper()
        
    if char not in EMNIST_CLASSES:
        print(f"Error: Character '{char}' is not supported by the EMNIST ByMerge dataset.")
        sys.exit(1)
        
    char_idx = EMNIST_CLASSES.index(char)
    print(f"Searching dataset for character '{char}' (Class Index {char_idx})...")
    
    # Load dataset using memory mapping to save RAM and time
    data = np.load(npz_path, mmap_mode='r')
    x_test = data['x_test']
    y_test = data['y_test']
    
    # Find matching indices
    # Since y_test is one-hot encoded, we look at the column of our target class
    matching_indices = np.where(y_test[:, char_idx] == 1)[0]
    
    if len(matching_indices) == 0:
        print(f"Error: No images found for character '{char}' in dataset.")
        sys.exit(1)
        
    print(f"Found {len(matching_indices)} images. Selecting a random sample...")
    selected_idx = np.random.choice(matching_indices)
    
    # Extract raw image (28x28)
    img = x_test[selected_idx].reshape(28, 28)
    
    # Render upright image to terminal
    render_terminal(img)
    
    # Normalize image to [0.0, 1.0] for C++ array
    max_val = np.max(img)
    img_normalized = img / 255.0 if max_val > 1.0 else img
    
    # Transpose the image!
    # Because the model was trained on transposed data, we write it transposed to input.h
    # so the Arduino code does not need to waste CPU/RAM transposing it.
    img_transposed = np.transpose(img_normalized)
    
    # Write to input.h
    print(f"Writing transposed image to {input_h_path}...")
    
    cpp_lines = []
    cpp_lines.append("#ifndef INPUT_H")
    cpp_lines.append("#define INPUT_H")
    cpp_lines.append("")
    cpp_lines.append(f"// Selected Character: '{char}'")
    cpp_lines.append(f"// EMNIST ByMerge Class Index: {char_idx}")
    cpp_lines.append(f"// Original dataset index: {selected_idx}")
    cpp_lines.append("const float input_image[28][28] = {")
    
    for r in range(28):
        row_vals = ", ".join(f"{img_transposed[r, c]:.6f}f" for c in range(28))
        comma = "," if r < 27 else ""
        cpp_lines.append(f"    {{{row_vals}}}{comma}")
        
    cpp_lines.append("};")
    cpp_lines.append("")
    cpp_lines.append("#endif // INPUT_H")
    
    with open(input_h_path, "w") as f:
        f.write("\n".join(cpp_lines) + "\n")
        
    print("Successfully updated input.h!")

if __name__ == "__main__":
    main()
