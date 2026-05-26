import sys
import os
import cv2
import numpy as np
import time
import torch

script_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(script_dir)

# Append parent directory so we can import student architecture v3.1 directly
import importlib.util
spec = importlib.util.spec_from_file_location("run_model_v3_1", os.path.join(parent_dir, "v3.1", "run_model.py"))
run_model_v3_1 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(run_model_v3_1)
ModelV3_1 = run_model_v3_1.ModelV3_1
EMNIST_CLASSES = run_model_v3_1.EMNIST_CLASSES

def load_quantized_weights(model, hidden_layer_dir):
    try:
        # Load weights (saved as ints)
        c1_q = np.loadtxt(os.path.join(hidden_layer_dir, "conv1.txt")).reshape(4, 1, 3, 3)
        c2_q = np.loadtxt(os.path.join(hidden_layer_dir, "conv2.txt")).reshape(4, 1, 3, 3)
        h1_t_q = np.loadtxt(os.path.join(hidden_layer_dir, "H1_trainable.txt")).reshape(36, 576, 12)
        h1_f_q = np.loadtxt(os.path.join(hidden_layer_dir, "H1_fixed.txt")).reshape(4, 26, 12)
        h2_q = np.loadtxt(os.path.join(hidden_layer_dir, "H2.txt")).reshape(480, 128)
        h3_q = np.loadtxt(os.path.join(hidden_layer_dir, "H3.txt")).reshape(128, 47)
        
        # Load scales
        c1_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "conv1_scale.txt")))
        c2_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "conv2_scale.txt")))
        h1_t_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "H1_trainable_scale.txt")))
        h1_f_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "H1_fixed_scale.txt")))
        h2_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "H2_scale.txt")))
        h3_s = float(np.loadtxt(os.path.join(hidden_layer_dir, "H3_scale.txt")))
        
        # Dequantize back to float32
        model.conv1.weight.data = torch.tensor(c1_q * c1_s, dtype=torch.float32)
        model.conv2.weight.data = torch.tensor(c2_q * c2_s, dtype=torch.float32)
        model.H1_trainable.data = torch.tensor(h1_t_q * h1_t_s, dtype=torch.float32)
        model.H1_fixed.data = torch.tensor(h1_f_q * h1_f_s, dtype=torch.float32)
        model.H2.data = torch.tensor(h2_q * h2_s, dtype=torch.float32)
        model.H3.data = torch.tensor(h3_q * h3_s, dtype=torch.float32)
    except OSError as e:
        print(f"Error loading quantized weights: {e}")
        return False
    return True

def run_inference(image_path, model, quiet=False):
    if not os.path.exists(image_path):
        return None

    image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if image is None: return None
    image = cv2.resize(image, (28, 28), interpolation=cv2.INTER_AREA)

    corners = [float(image[0, 0]), float(image[0, -1]), float(image[-1, 0]), float(image[-1, -1])]
    corner_avg = sum(corners) / 4.0
    if corner_avg > 127:
        image = 255 - image

    I = image.astype(np.float32) / 255.0
    
    # NOTE: The distilled student model in distil_q was trained on transposed (rotated/mirrored) data.
    # When feeding a standard real-world upright user-supplied image, we MUST apply transposition 
    # (swapping rows/columns) before inference to align with the model's trained weight orientation.
    I_transposed = np.transpose(I)
    
    model.eval()
    with torch.no_grad():
        x_tensor = torch.tensor(I_transposed).unsqueeze(0).unsqueeze(0)
        out = model(x_tensor)
        predicted_digit = torch.argmax(out).item()

    predicted_chars = EMNIST_CLASSES[predicted_digit] if predicted_digit < len(EMNIST_CLASSES) else str(predicted_digit)
    
    if not quiet:
        print(f"=> Predicted: {predicted_chars} (Class {predicted_digit})")
    return predicted_digit

if __name__ == "__main__":
    hidden_layer_dir = os.path.join(script_dir, "hiddenLayer")
    
    model = ModelV3_1()
    
    if not load_quantized_weights(model, hidden_layer_dir):
        print("Error: Quantized weights not found in distil_q/hiddenLayer/. Run quantize.py first.")
        sys.exit(1)
        
    print("Loading 8-bit quantized weights and scales...")
    total_params = sum(p.numel() for p in model.parameters())
    print(f"Model Size: {total_params:,} parameters (quantized to 8 bits/parameter)")

    if len(sys.argv) > 1:
        t0 = time.perf_counter()
        run_inference(sys.argv[1], model, quiet=False)
        t1 = time.perf_counter()
        print(f"Inference Time: {(t1 - t0) * 1000:.2f} ms")
    else:
        # Check standard EMNIST dataset locations
        npz_path = os.path.join(parent_dir, "mnist", "emnist_28x28.npz")
        if not os.path.exists(npz_path):
            npz_path = os.path.join(parent_dir, "emnist_28x28.npz")
            
        if not os.path.exists(npz_path):
            print(f"Error: {npz_path} not found.")
            sys.exit(1)
            
        print("\nLoading 100 random samples from EMNIST test dataset...")
        data = np.load(npz_path)
        x_test = data['x_test']
        y_test = data['y_test']
        
        num_samples = 100
        indices = np.random.choice(len(x_test), num_samples, replace=False)
        x_sample = x_test[indices].reshape(-1, 28, 28)
        x_sample = np.transpose(x_sample, (0, 2, 1))
        y_sample = y_test[indices]
        
        print(f"Evaluating {num_samples} random samples using 8-bit quantized weights...")
        correct = 0
        t0 = time.perf_counter()
        
        model.eval()
        with torch.no_grad():
            for i in range(num_samples):
                I_batch = torch.tensor(x_sample[i].reshape(1, 1, 28, 28), dtype=torch.float32)
                A = model(I_batch)
                predicted_digit = torch.argmax(A).item()
                true_label = np.argmax(y_sample[i])
                if predicted_digit == true_label:
                    correct += 1
                
        t1 = time.perf_counter()
        accuracy = (correct / num_samples) * 100
        print(f"[EMNIST Test] 8-Bit Quantized Student Accuracy: {correct}/{num_samples} ({accuracy:.2f}%)")
        print(f"Total Inference Time for 100 images: {t1 - t0:.4f}s ({(t1 - t0)*1000/num_samples:.2f} ms/image)")
