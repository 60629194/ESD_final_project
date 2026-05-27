import sys
import os
import cv2
import numpy as np
import time
import torch
import torch.nn as nn
import torch.nn.functional as F

EMNIST_CLASSES = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 
                  'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', 
                  'a', 'b', 'd', 'e', 'f', 'g', 'h', 'n', 'q', 'r', 't']

CHAR_TO_INDEX = {c: i for i, c in enumerate(EMNIST_CLASSES)}
MERGED_LOWERCASE = ['c', 'i', 'j', 'k', 'l', 'm', 'o', 'p', 's', 'u', 'v', 'w', 'x', 'y', 'z']
for c in MERGED_LOWERCASE:
    CHAR_TO_INDEX[c] = CHAR_TO_INDEX[c.upper()]

SLICES_CONFIG = [
    (1, 576, 24),
    (2, 288, 12),
    (4, 144, 6),
    (8, 72, 3),
    (24, 24, 1),
    (48, 12, 1),
    (96, 6, 1),
    (192, 3, 1),
    (576, 1, 1),
]

# Fixed 3x3 kernels for fixed edge-detectors
SOBEL_X = np.array([
    [-1,  0,  1],
    [-2,  0,  2],
    [-1,  0,  1]
], dtype=np.float32)

SOBEL_Y = np.array([
    [-1, -2, -1],
    [ 0,  0,  0],
    [ 1,  2,  1]
], dtype=np.float32)

SOBEL_DIAG1 = np.array([
    [-2, -1,  0],
    [-1,  0,  1],
    [ 0,  1,  2]
], dtype=np.float32)

SOBEL_DIAG2 = np.array([
    [ 0, -1, -2],
    [ 1,  0, -1],
    [ 2,  1,  0]
], dtype=np.float32)

class ModelV3_1(nn.Module):
    def __init__(self):
        super(ModelV3_1, self).__init__()
        import math
        
        # 1. Trainable conv stream
        self.conv1 = nn.Conv2d(1, 4, kernel_size=3, padding=0, stride=1, bias=False)
        self.conv2 = nn.Conv2d(4, 4, kernel_size=3, padding=0, stride=1, groups=4, bias=False)
        
        # 2. Fixed conv stream (registered as buffer)
        sobel_kernels = np.stack([SOBEL_X, SOBEL_Y, SOBEL_DIAG1, SOBEL_DIAG2], axis=0) # shape (4, 3, 3)
        self.register_buffer('fixed_weight', torch.tensor(sobel_kernels, dtype=torch.float32).unsqueeze(1)) # shape (4, 1, 3, 3)
        
        # 3. Hidden layer projections (H1)
        # H1_trainable: Projects 36 channels (4 channels x 9 slices) from size 576 to 12.
        self.H1_trainable = nn.Parameter(torch.randn(36, 576, 12) * math.sqrt(2.0 / 576))
        # H1_fixed: Projects 4 fixed conv channels from size 26x26 using a 26x12 weight matrix per channel to 12.
        self.H1_fixed = nn.Parameter(torch.randn(4, 26, 12) * math.sqrt(2.0 / 26))
        
        # 4. Dense representation projections
        # H2 shape: Projects concatenated (36 + 4) * 12 = 480 features to 128.
        self.H2 = nn.Parameter(torch.randn(480, 128) * math.sqrt(2.0 / 480))
        # H3 shape: Projects 128 features to 47 EMNIST classes.
        self.H3 = nn.Parameter(torch.randn(128, 47) * math.sqrt(2.0 / 128))
        
    def forward(self, x):
        B = x.shape[0]
        
        # Stream 1: Trainable Conv and Slicing
        c1 = self.conv1(x)
        c2 = self.conv2(c1) # shape (B, 4, 24, 24)
        
        sliced_channels = []
        for I, D, J in SLICES_CONFIG:
            c2_sliced = c2.view(B, 4, I, D).transpose(2, 3).reshape(B, 4, 576)
            sliced_channels.append(c2_sliced)
            
        sliced_all = torch.cat(sliced_channels, dim=1) # shape (B, 36, 576)
        o_trainable = torch.einsum('bci,cij->bcj', sliced_all, self.H1_trainable) # shape (B, 36, 12)
        o_trainable = torch.relu(o_trainable)
        
        # Stream 2: Fixed Conv Edge Detection
        fixed_conv_out = F.conv2d(x, self.fixed_weight, padding=0, stride=1) # shape (B, 4, 26, 26)
        o_fixed = torch.einsum('bcxy,cyz->bcz', fixed_conv_out, self.H1_fixed) # shape (B, 4, 12)
        o_fixed = torch.relu(o_fixed)
        
        # Concatenate Outputs
        o_concat = torch.cat([o_trainable, o_fixed], dim=1) # shape (B, 40, 12)
        o_flat = o_concat.reshape(B, 480)
        
        # Classifier
        h2 = torch.matmul(o_flat, self.H2)
        h2 = torch.relu(h2)
        final_out = torch.matmul(h2, self.H3)
        return final_out

def load_weights(model, hidden_layer_dir):
    try:
        model.conv1.weight.data = torch.tensor(np.loadtxt(os.path.join(hidden_layer_dir, "conv1.txt")).reshape(4, 1, 3, 3), dtype=torch.float32)
        model.conv2.weight.data = torch.tensor(np.loadtxt(os.path.join(hidden_layer_dir, "conv2.txt")).reshape(4, 1, 3, 3), dtype=torch.float32)
        model.H1_trainable.data = torch.tensor(np.loadtxt(os.path.join(hidden_layer_dir, "H1_trainable.txt")).reshape(36, 576, 12), dtype=torch.float32)
        model.H1_fixed.data = torch.tensor(np.loadtxt(os.path.join(hidden_layer_dir, "H1_fixed.txt")).reshape(4, 26, 12), dtype=torch.float32)
        model.H2.data = torch.tensor(np.loadtxt(os.path.join(hidden_layer_dir, "H2.txt")).reshape(480, 128), dtype=torch.float32)
        model.H3.data = torch.tensor(np.loadtxt(os.path.join(hidden_layer_dir, "H3.txt")).reshape(128, 47), dtype=torch.float32)
    except OSError:
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
    
    model.eval()
    with torch.no_grad():
        x_tensor = torch.tensor(I).unsqueeze(0).unsqueeze(0) # (1, 1, 28, 28)
        out = model(x_tensor)
        predicted_digit = torch.argmax(out).item()

    predicted_chars = EMNIST_CLASSES[predicted_digit] if predicted_digit < len(EMNIST_CLASSES) else str(predicted_digit)
    
    if not quiet:
        print(f"=> Predicted: {predicted_chars} (Class {predicted_digit})")
    return predicted_digit

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    hidden_layer_dir = os.path.join(script_dir, "hiddenLayer")
    
    model = ModelV3_1()
    
    if not load_weights(model, hidden_layer_dir):
        print("Error: Model weights not found in hiddenLayer/. Run reset_model.py to initialize them.")
        sys.exit(1)
        
    print("Loading weights...")
    total_params = sum(p.numel() for p in model.parameters())
    total_size_kb = total_params * 4 / 1024.0 # float32 is 4 bytes
    print(f"Model Size: {total_params:,} parameters ({total_size_kb:.2f} KB)")

    if len(sys.argv) > 1:
        t0 = time.perf_counter()
        run_inference(sys.argv[1], model, quiet=False)
        t1 = time.perf_counter()
        print(f"Inference Time: {(t1 - t0) * 1000:.2f} ms")
    else:
        npz_path = os.path.join(script_dir, "..", "emnist_28x28.npz")
        if not os.path.exists(npz_path):
            print(f"Error: {npz_path} not found. Please run prepare_emnist.py first.")
            sys.exit(1)
            
        print("\nLoading 100 random samples from EMNIST test dataset...")
        data = np.load(npz_path)
        x_test = data['x_test']
        y_test = data['y_test']
        
        num_samples = 100
        indices = np.random.choice(len(x_test), num_samples, replace=False)
        x_sample = x_test[indices].reshape(-1, 28, 28)
        # Fix the orientation by transposing the H and W axes to upright
        x_sample = np.transpose(x_sample, (0, 2, 1))
        y_sample = y_test[indices]
        
        print(f"Evaluating {num_samples} random samples...")
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
        print(f"[EMNIST Test Dataset] Accuracy: {correct}/{num_samples} ({accuracy:.2f}%)")
        print(f"[EMNIST Test Dataset] Total Time: {t1 - t0:.4f}s ({(t1 - t0)*1000/num_samples:.2f} ms/image)")
