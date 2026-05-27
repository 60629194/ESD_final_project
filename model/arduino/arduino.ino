#include "weights.h"
#include "input.h"

// Slice Configuration structure
struct SliceConfig {
  int I;
  int D;
  int J;
};

// Slices configurations matching run_model.py
const SliceConfig SLICES_CONFIG[9] = {
  {1, 576, 24},
  {2, 288, 12},
  {4, 144, 6},
  {8, 72, 3},
  {24, 24, 1},
  {48, 12, 1},
  {96, 6, 1},
  {192, 3, 1},
  {576, 1, 1}
};

// Fixed 3x3 kernels for fixed edge-detectors
const int8_t SOBEL_X[3][3] = {
  {-1,  0,  1},
  {-2,  0,  2},
  {-1,  0,  1}
};

const int8_t SOBEL_Y[3][3] = {
  {-1, -2, -1},
  { 0,  0,  0},
  { 1,  2,  1}
};

const int8_t SOBEL_DIAG1[3][3] = {
  {-2, -1,  0},
  {-1,  0,  1},
  { 0,  1,  2}
};

const int8_t SOBEL_DIAG2[3][3] = {
  { 0, -1, -2},
  { 1,  0, -1},
  { 2,  1,  0}
};

// Globally allocated buffers to avoid stack overflow (Due SRAM is 96 KB)
static float c1[4][26][26];
static float c2[4][24][24];
static float fixed_conv_out[4][26][26];
static float o_trainable[36][12];
static float o_fixed[4][12];
static float o_flat[480];
static float h2[128];
static float final_out[47];

void run_inference() {
  uint32_t t_start = millis();

  // ==========================================
  // STREAM 1: Trainable Conv Stream
  // ==========================================
  
  // 1. conv1: Standard 2D convolution (input 1x28x28 -> output 4x26x26)
  // Input: input_image[28][28], weights: conv1_weights[4][1][3][3]
  for (int c = 0; c < 4; c++) {
    for (int y = 0; y < 26; y++) {
      for (int x = 0; x < 26; x++) {
        float sum = 0.0f;
        for (int ky = 0; ky < 3; ky++) {
          for (int kx = 0; kx < 3; kx++) {
            sum += input_image[y + ky][x + kx] * conv1_weights[c][0][ky][kx];
          }
        }
        c1[c][y][x] = sum * conv1_scale;
      }
    }
  }

  // 2. conv2: Depthwise Conv2d (groups = 4, output 4x24x24)
  // Input: c1[4][26][26], weights: conv2_weights[4][1][3][3]
  for (int c = 0; c < 4; c++) {
    for (int y = 0; y < 24; y++) {
      for (int x = 0; x < 24; x++) {
        float sum = 0.0f;
        for (int ky = 0; ky < 3; ky++) {
          for (int kx = 0; kx < 3; kx++) {
            sum += c1[c][y + ky][x + kx] * conv2_weights[c][0][ky][kx];
          }
        }
        c2[c][y][x] = sum * conv2_scale;
      }
    }
  }

  // 3. Slicing & Trainable Projection H1_trainable
  // We perform the slicing mapping on the fly to avoid 82.9 KB of sliced_all buffer!
  for (int ch = 0; ch < 36; ch++) {
    int s = ch / 4;
    int c = ch % 4;
    int I_val = SLICES_CONFIG[s].I;
    int D_val = SLICES_CONFIG[s].D;
    for (int z = 0; z < 12; z++) {
      float sum = 0.0f;
      for (int idx = 0; idx < 576; idx++) {
        // Reverse slicing mapping math
        int v = idx / I_val;
        int u = idx % I_val;
        int k = u * D_val + v;
        int y = k / 24;
        int x = k % 24;
        
        sum += c2[c][y][x] * H1_trainable_weights[ch][idx][z];
      }
      // Apply ReLU & scale
      if (sum < 0.0f) sum = 0.0f;
      o_trainable[ch][z] = sum * H1_trainable_scale;
    }
  }

  // ==========================================
  // STREAM 2: Fixed Conv Edge Detection
  // ==========================================
  
  // 1. Sobel convolution over input_image[28][28] -> output fixed_conv_out[4][26][26]
  for (int y = 0; y < 26; y++) {
    for (int x = 0; x < 26; x++) {
      float sum_x = 0.0f;
      float sum_y = 0.0f;
      float sum_d1 = 0.0f;
      float sum_d2 = 0.0f;
      for (int ky = 0; ky < 3; ky++) {
        for (int kx = 0; kx < 3; kx++) {
          float val = input_image[y + ky][x + kx];
          sum_x += val * SOBEL_X[ky][kx];
          sum_y += val * SOBEL_Y[ky][kx];
          sum_d1 += val * SOBEL_DIAG1[ky][kx];
          sum_d2 += val * SOBEL_DIAG2[ky][kx];
        }
      }
      fixed_conv_out[0][y][x] = sum_x;
      fixed_conv_out[1][y][x] = sum_y;
      fixed_conv_out[2][y][x] = sum_d1;
      fixed_conv_out[3][y][x] = sum_d2;
    }
  }

  // 2. o_fixed = einsum('bcxy,cyz->bcz', fixed_conv_out, H1_fixed)
  for (int c = 0; c < 4; c++) {
    float S[26];
    // Sum over row index x of fixed_conv_out[c][x][y]
    for (int y = 0; y < 26; y++) {
      float sum_x = 0.0f;
      for (int x = 0; x < 26; x++) {
        sum_x += fixed_conv_out[c][x][y];
      }
      S[y] = sum_x;
    }
    
    // Matrix multiply with H1_fixed_weights
    for (int z = 0; z < 12; z++) {
      float sum_y = 0.0f;
      for (int y = 0; y < 26; y++) {
        sum_y += S[y] * H1_fixed_weights[c][y][z];
      }
      // Apply ReLU & scale
      if (sum_y < 0.0f) sum_y = 0.0f;
      o_fixed[c][z] = sum_y * H1_fixed_scale;
    }
  }

  // ==========================================
  // CONCATENATE & FLATTEN
  // ==========================================
  
  // Concatenate o_trainable (36x12) and o_fixed (4x12) into o_flat (480)
  for (int ch = 0; ch < 36; ch++) {
    for (int z = 0; z < 12; z++) {
      o_flat[ch * 12 + z] = o_trainable[ch][z];
    }
  }
  for (int ch = 0; ch < 4; ch++) {
    for (int z = 0; z < 12; z++) {
      o_flat[432 + ch * 12 + z] = o_fixed[ch][z];
    }
  }

  // ==========================================
  // CLASSIFIER DENSE LAYERS
  // ==========================================
  
  // 1. Layer H2: o_flat (480) -> h2 (128)
  for (int j = 0; j < 128; j++) {
    float sum = 0.0f;
    for (int i = 0; i < 480; i++) {
      sum += o_flat[i] * H2_weights[i][j];
    }
    if (sum < 0.0f) sum = 0.0f;
    h2[j] = sum * H2_scale;
  }

  // 2. Layer H3: h2 (128) -> final_out (47)
  for (int k = 0; k < 47; k++) {
    float sum = 0.0f;
    for (int j = 0; j < 128; j++) {
      sum += h2[j] * H3_weights[j][k];
    }
    final_out[k] = sum * H3_scale;
  }

  uint32_t t_inference = millis() - t_start;

  // ==========================================
  // ARGMAX PREDICTION
  // ==========================================
  int predicted_digit = 0;
  float max_score = final_out[0];
  for (int k = 1; k < 47; k++) {
    if (final_out[k] > max_score) {
      max_score = final_out[k];
      predicted_digit = k;
    }
  }

  // Print results
  Serial.println("================================================");
  Serial.print("Predicted Class Index: ");
  Serial.println(predicted_digit);
  Serial.print("Predicted Character:   ");
  Serial.println(EMNIST_CLASSES[predicted_digit]);
  Serial.print("Confidence Score:      ");
  Serial.println(max_score, 4);
  Serial.print("Inference Time:        ");
  Serial.print(t_inference);
  Serial.println(" ms");
  Serial.println("================================================");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  delay(1000);
  
  Serial.println("EMNIST ByMerge Model Initialized on Arduino Due.");
  Serial.println("Performing inference on input.h image...");
  
  run_inference();
}

void loop() {
  // Nothing to do in the loop
  delay(1000);
}
