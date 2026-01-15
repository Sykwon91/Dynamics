#include "transformer.h"


int main() {
    // Example: n tokens, d_model embedding dimension
    int n = 6;
    int d_model = 32;
    int n_heads = 4;
    int d_ff = 64;

    TransformerEncoderLayer layer(d_model, n_heads, d_ff);

    // dummy input X (n x d_model)
    Tensor2D X(n, d_model);
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> ud(-1.0f, 1.0f);
    for (auto& v : X.a) v = ud(rng);

    Tensor2D Y = layer.forward(X);

    std::cout << "Output Y shape: (" << Y.r << ", " << Y.c << ")\n";
    std::cout << "Y[0,0..7] = ";
    for (int j = 0; j < 8; ++j) std::cout << Y(0, j) << " ";
    std::cout << "\n";
    return 0;
}
