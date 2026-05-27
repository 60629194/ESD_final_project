import os
import re
import numpy as np
import torch
from run_model import ModelV3_1, SOBEL_X, SOBEL_Y, SOBEL_DIAG1, SOBEL_DIAG2, SLICES_CONFIG, EMNIST_CLASSES

def parse_input_h(input_h_path):
    with open(input_h_path, "r") as f:
        content = f.read()
    
    # Extract matrix rows using regex
    row_pattern = r"\{(.*?)\}"
    rows = re.findall(row_pattern, content)
    
    matrix = []
    for r in rows:
        # Ignore empty matches or non-numeric matches
        parts = [float(x.strip().replace('f', '')) for x in r.split(",") if x.strip()]
        if len(parts) == 28:
            matrix.append(parts)
            
    return np.array(matrix, dtype=np.float32)

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    hidden_layer_dir = os.path.join(script_dir, "hiddenLayer")
    input_h_path = os.path.join(script_dir, "..", "arduino", "input.h")
    
    if not os.path.exists(input_h_path):
        print(f"Error: {input_h_path} not found. Please run modify_input.py first.")
        return
        
    print("Loading image from input.h...")
    # This image is already transposed in input.h
    img_transposed = parse_input_h(input_h_path)
    
    # ----------------------------------------------------
    # 1. PyTorch Reference Inference
    # ----------------------------------------------------
    print("\nRunning PyTorch Reference Inference...")
    model = ModelV3_1()
    
    # Load quantized weights back to float32
    c1_q = np.loadtxt(os.path.join(hidden_layer_dir, "conv1.txt")).reshape(4, 1, 3, 3)
    c2_q = np.loadtxt(os.path.join(hidden_layer_dir, "conv2.txt")).reshape(4, 1, 3, 3)
    h1_t_q = np.loadtxt(os.path.join(hidden_layer_dir, "H1_trainable.txt")).reshape(36, 576, 12)
    h1_f_q = np.loadtxt(os.path.join(hidden_layer_dir, "H1_fixed.txt")).reshape(4, 26, 12)
    h2_q = np.loadtxt(os.path.join(hidden_layer_dir, "H2.txt")).reshape(480, 128)
    h3_q = np.loadtxt(os.path.join(hidden_layer_dir, "H3.txt")).reshape(128, 47)
    
    c1_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "conv1_scale.txt")))
    c2_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "conv2_scale.txt")))
    h1_t_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "H1_trainable_scale.txt")))
    h1_f_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "H1_fixed_scale.txt")))
    h2_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "H2_scale.txt")))
    h3_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "H3_scale.txt")))
    
    model.conv1.weight.data = torch.tensor(c1_q * c1_s, dtype=torch.float32)
    model.conv2.weight.data = torch.tensor(c2_q * c2_s, dtype=torch.float32)
    model.H1_trainable.data = torch.tensor(h1_t_q * h1_t_s, dtype=torch.float32)
    model.H1_fixed.data = torch.tensor(h1_f_q * h1_f_s, dtype=torch.float32)
    model.H2.data = torch.tensor(h2_q * h2_s, dtype=torch.float32)
    model.H3.data = torch.tensor(h3_q * h3_s, dtype=torch.float32)
    
    model.eval()
    with torch.no_grad():
        # Note: Since the image inside input.h is already transposed, and PyTorch expects a transposed image,
        # we can feed it directly.
        x_tensor = torch.tensor(img_transposed).unsqueeze(0).unsqueeze(0)
        pytorch_out = model(x_tensor).numpy().flatten()
        
    pytorch_predicted = np.argmax(pytorch_out)
    print(f"PyTorch Result: Index {pytorch_predicted} ('{EMNIST_CLASSES[pytorch_predicted]}') with score {pytorch_out[pytorch_predicted]:.4f}")
    
    # ----------------------------------------------------
    # 2. NumPy Implementation Mimicking Arduino C++
    # ----------------------------------------------------
    print("\nRunning custom NumPy emulation (mimics Arduino C++)...")
    
    # Stream 1: Trainable Conv
    c1 = np.zeros((4, 26, 26), dtype=np.float32)
    for c in range(4):
        for y in range(26):
            for x in range(26):
                sum_val = np.sum(img_transposed[y:y+3, x:x+3] * c1_q[c, 0])
                c1[c, y, x] = sum_val * c1_s
                
    c2 = np.zeros((4, 24, 24), dtype=np.float32)
    for c in range(4):
        for y in range(24):
            for x in range(24):
                sum_val = np.sum(c1[c, y:y+3, x:x+3] * c2_q[c, 0])
                c2[c, y, x] = sum_val * c2_s
                
    # Trainable projection H1_trainable with on-the-fly reverse slicing mapping
    o_trainable = np.zeros((36, 12), dtype=np.float32)
    for ch in range(36):
        s = ch // 4
        c = ch % 4
        I_val, D_val, _ = SLICES_CONFIG[s]
        for z in range(12):
            sum_val = 0.0
            for idx in range(576):
                # Reverse mapping
                v = idx // I_val
                u = idx % I_val
                k = u * D_val + v
                y = k // 24
                x = k % 24
                
                sum_val += c2[c, y, x] * h1_t_q[ch, idx, z]
            o_trainable[ch, z] = max(0.0, sum_val) * h1_t_s
            
    # Stream 2: Fixed Conv Edge Detection
    fixed_conv_out = np.zeros((4, 26, 26), dtype=np.float32)
    sobels = [SOBEL_X, SOBEL_Y, SOBEL_DIAG1, SOBEL_DIAG2]
    for c in range(4):
        for y in range(26):
            for x in range(26):
                fixed_conv_out[c, y, x] = np.sum(img_transposed[y:y+3, x:x+3] * sobels[c])
                
    o_fixed = np.zeros((4, 12), dtype=np.float32)
    for c in range(4):
        S = np.zeros(26, dtype=np.float32)
        for y in range(26):
            S[y] = np.sum(fixed_conv_out[c, :, y])
            
        for z in range(12):
            sum_val = np.sum(S * h1_f_q[c, :, z])
            o_fixed[c, z] = max(0.0, sum_val) * h1_f_s
            
    # Concatenate & Classifier
    o_flat = np.concatenate([o_trainable.flatten(), o_fixed.flatten()])
    h2 = np.maximum(0.0, np.dot(o_flat, h2_q)) * h2_s
    cpp_out = np.dot(h2, h3_q) * h3_s
    
    cpp_predicted = np.argmax(cpp_out)
    print(f"Emulation Result: Index {cpp_predicted} ('{EMNIST_CLASSES[cpp_predicted]}') with score {cpp_out[cpp_predicted]:.4f}")
    
    # ----------------------------------------------------
    # 3. Final Verification and Compare
    # ----------------------------------------------------
    match = (pytorch_predicted == cpp_predicted)
    diff = np.abs(pytorch_out - cpp_out)
    max_diff = np.max(diff)
    mean_diff = np.mean(diff)
    
    print("\n" + "=" * 50)
    print("VERIFICATION STATUS:")
    if match and max_diff < 1e-4:
        print("  SUCCESS: Custom Arduino C++ math matches PyTorch exactly!")
    elif match:
        print("  WARNING: Predictions match, but minor numerical differences exist.")
    else:
        print("  FAILURE: Predictions do NOT match!")
    print(f"  Max Absolute Logit Difference: {max_diff:.8f}")
    print(f"  Mean Absolute Logit Difference: {mean_diff:.8f}")
    print("=" * 50)

if __name__ == "__main__":
    main()
