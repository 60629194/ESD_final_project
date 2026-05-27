import os
import numpy as np

def format_array_cpp(arr, indent=4):
    spacing = " " * indent
    if len(arr.shape) == 1:
        # Check if float or int
        if np.issubdtype(arr.dtype, np.floating):
            return "{" + ", ".join(f"{x:.6f}f" for x in arr) + "}"
        else:
            return "{" + ", ".join(str(int(x)) for x in arr) + "}"
    else:
        inner = []
        for sub in arr:
            inner.append(format_array_cpp(sub, indent + 4))
        spacer = ",\n" + spacing
        return "{\n" + spacing + spacer.join(inner) + "\n" + (" " * (indent - 4)) + "}"

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    hidden_layer_dir = os.path.join(script_dir, "hiddenLayer")
    arduino_dir = os.path.join(script_dir, "..", "arduino")
    os.makedirs(arduino_dir, exist_ok=True)
    
    output_path = os.path.join(arduino_dir, "weights.h")
    print(f"Loading weights from {hidden_layer_dir}...")
    
    # Load quantized weights as integers
    conv1 = np.loadtxt(os.path.join(hidden_layer_dir, "conv1.txt"), dtype=int).reshape(4, 1, 3, 3)
    conv2 = np.loadtxt(os.path.join(hidden_layer_dir, "conv2.txt"), dtype=int).reshape(4, 1, 3, 3)
    h1_trainable = np.loadtxt(os.path.join(hidden_layer_dir, "H1_trainable.txt"), dtype=int).reshape(36, 576, 12)
    h1_fixed = np.loadtxt(os.path.join(hidden_layer_dir, "H1_fixed.txt"), dtype=int).reshape(4, 26, 12)
    h2 = np.loadtxt(os.path.join(hidden_layer_dir, "H2.txt"), dtype=int).reshape(480, 128)
    h3 = np.loadtxt(os.path.join(hidden_layer_dir, "H3.txt"), dtype=int).reshape(128, 47)
    
    # Load scales
    conv1_scale = float(np.loadtxt(os.path.join(hidden_layer_dir, "conv1_scale.txt")))
    conv2_scale = float(np.loadtxt(os.path.join(hidden_layer_dir, "conv2_scale.txt")))
    h1_t_scale = float(np.loadtxt(os.path.join(hidden_layer_dir, "H1_trainable_scale.txt")))
    h1_f_scale = float(np.loadtxt(os.path.join(hidden_layer_dir, "H1_fixed_scale.txt")))
    h2_scale = float(np.loadtxt(os.path.join(hidden_layer_dir, "H2_scale.txt")))
    h3_scale = float(np.loadtxt(os.path.join(hidden_layer_dir, "H3_scale.txt")))
    
    print("Formatting weights into C++ header...")
    
    cpp_content = f"""#ifndef WEIGHTS_H
#define WEIGHTS_H

// EMNIST Classes
const char EMNIST_CLASSES[47] = {{
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z',
    'a', 'b', 'd', 'e', 'f', 'g', 'h', 'n', 'q', 'r', 't'
}};

// Quantization Scales
const float conv1_scale = {conv1_scale:.10e}f;
const float conv2_scale = {conv2_scale:.10e}f;
const float H1_trainable_scale = {h1_t_scale:.10e}f;
const float H1_fixed_scale = {h1_f_scale:.10e}f;
const float H2_scale = {h2_scale:.10e}f;
const float H3_scale = {h3_scale:.10e}f;

// Quantized Weights (Stored in Flash memory on SAM3X8E due to 'const' storage)
const int8_t conv1_weights[4][1][3][3] = {format_array_cpp(conv1)};

const int8_t conv2_weights[4][1][3][3] = {format_array_cpp(conv2)};

const int8_t H1_trainable_weights[36][576][12] = {format_array_cpp(h1_trainable)};

const int8_t H1_fixed_weights[4][26][12] = {format_array_cpp(h1_fixed)};

const int8_t H2_weights[480][128] = {format_array_cpp(h2)};

const int8_t H3_weights[128][47] = {format_array_cpp(h3)};

#endif // WEIGHTS_H
"""
    
    with open(output_path, "w") as f:
        f.write(cpp_content)
        
    print(f"Successfully generated weights.h at {output_path}!")

if __name__ == "__main__":
    main()
