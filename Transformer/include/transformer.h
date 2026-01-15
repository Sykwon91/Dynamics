#ifndef TRANSFORMER_H
#define TRANSFORMER_H
#include <iostream>
#include <vector>
#include <cmath>
#include <random>
#include <cassert>
#include <algorithm>

using std::vector;

static inline float gelu(float x) {
    // GELU approximation (tanh-based)
    const float c = 0.044715f;
    const float sqrt_2_over_pi = 0.7978845608028654f;
    float x3 = x * x * x;
    return 0.5f * x * (1.0f + std::tanh(sqrt_2_over_pi * (x + c * x3)));
}

struct Tensor2D {
    int r, c;           // rows, cols
    vector<float> a;    // row-major
    Tensor2D() : r(0), c(0) {}
    Tensor2D(int r_, int c_, float v=0.0f) : r(r_), c(c_), a(r_*c_, v) {}

    float& operator()(int i, int j) { return a[i*c + j]; }
    float  operator()(int i, int j) const { return a[i*c + j]; }
};

static Tensor2D matmul(const Tensor2D& A, const Tensor2D& B) {
    // A: (m x k), B: (k x n) -> (m x n)
    assert(A.c == B.r);
    Tensor2D C(A.r, B.c, 0.0f);
    for (int i = 0; i < A.r; ++i) {
        for (int k = 0; k < A.c; ++k) {
            float aik = A(i, k);
            for (int j = 0; j < B.c; ++j) {
                C(i, j) += aik * B(k, j);
            }
        }
    }
    return C;
}

static Tensor2D add(const Tensor2D& A, const Tensor2D& B) {
    assert(A.r == B.r && A.c == B.c);
    Tensor2D C(A.r, A.c);
    for (int i = 0; i < (int)A.a.size(); ++i) C.a[i] = A.a[i] + B.a[i];
    return C;
}

static Tensor2D add_bias(const Tensor2D& A, const vector<float>& b) {
    // b: (cols)
    assert((int)b.size() == A.c);
    Tensor2D C = A;
    for (int i = 0; i < A.r; ++i)
        for (int j = 0; j < A.c; ++j)
            C(i, j) += b[j];
    return C;
}

static Tensor2D layernorm(const Tensor2D& X, const vector<float>& gamma, const vector<float>& beta, float eps=1e-5f) {
    // LN over last dimension (cols)
    assert((int)gamma.size() == X.c && (int)beta.size() == X.c);
    Tensor2D Y(X.r, X.c);
    for (int i = 0; i < X.r; ++i) {
        float mean = 0.0f;
        for (int j = 0; j < X.c; ++j) mean += X(i, j);
        mean /= (float)X.c;

        float var = 0.0f;
        for (int j = 0; j < X.c; ++j) {
            float d = X(i, j) - mean;
            var += d * d;
        }
        var /= (float)X.c;

        float inv_std = 1.0f / std::sqrt(var + eps);
        for (int j = 0; j < X.c; ++j) {
            float xn = (X(i, j) - mean) * inv_std;
            Y(i, j) = xn * gamma[j] + beta[j];
        }
    }
    return Y;
}

static void softmax_row_inplace(Tensor2D& M) {
    // softmax each row
    for (int i = 0; i < M.r; ++i) {
        float mx = -1e30f;
        for (int j = 0; j < M.c; ++j) mx = std::max(mx, M(i, j));
        float sum = 0.0f;
        for (int j = 0; j < M.c; ++j) {
            M(i, j) = std::exp(M(i, j) - mx);
            sum += M(i, j);
        }
        float inv = 1.0f / (sum + 1e-12f);
        for (int j = 0; j < M.c; ++j) M(i, j) *= inv;
    }
}

static Tensor2D transpose(const Tensor2D& X) {
    Tensor2D T(X.c, X.r);
    for (int i = 0; i < X.r; ++i)
        for (int j = 0; j < X.c; ++j)
            T(j, i) = X(i, j);
    return T;
}

static Tensor2D apply_gelu(const Tensor2D& X) {
    Tensor2D Y(X.r, X.c);
    for (int i = 0; i < (int)X.a.size(); ++i) Y.a[i] = gelu(X.a[i]);
    return Y;
}

static Tensor2D concat_heads(const vector<Tensor2D>& heads) {
    // each head: (n x d_head), concat -> (n x (h*d_head))
    assert(!heads.empty());
    int n = heads[0].r;
    int d_head = heads[0].c;
    for (auto& H : heads) { assert(H.r == n && H.c == d_head); }

    Tensor2D Y(n, (int)heads.size() * d_head);
    for (int i = 0; i < n; ++i) {
        for (int h = 0; h < (int)heads.size(); ++h) {
            for (int j = 0; j < d_head; ++j) {
                Y(i, h*d_head + j) = heads[h](i, j);
            }
        }
    }
    return Y;
}

static Tensor2D slice_cols(const Tensor2D& X, int c0, int c1) {
    // [c0, c1)
    assert(0 <= c0 && c0 < c1 && c1 <= X.c);
    Tensor2D Y(X.r, c1 - c0);
    for (int i = 0; i < X.r; ++i)
        for (int j = c0; j < c1; ++j)
            Y(i, j - c0) = X(i, j);
    return Y;
}

struct Linear {
    // y = xW + b
    // input: (n x in_dim), W: (in_dim x out_dim), b: (out_dim)
    Tensor2D W;
    vector<float> b;

    Linear() {}
    Linear(int in_dim, int out_dim) : W(in_dim, out_dim), b(out_dim, 0.0f) {}

    Tensor2D forward(const Tensor2D& X) const {
        assert(X.c == W.r);
        Tensor2D Y = matmul(X, W);
        Y = add_bias(Y, b);
        return Y;
    }
};

static void init_normal(Tensor2D& W, float stddev=0.02f, uint32_t seed=123) {
    std::mt19937 rng(seed);
    std::normal_distribution<float> nd(0.0f, stddev);
    for (auto& v : W.a) v = nd(rng);
}

static void init_zero(vector<float>& b) {
    std::fill(b.begin(), b.end(), 0.0f);
}

struct TransformerEncoderLayer {
    int d_model;
    int n_heads;
    int d_head;
    int d_ff;

    // projections
    Linear Wq, Wk, Wv;
    Linear Wo;

    // FFN
    Linear W1;
    Linear W2;

    // LayerNorm params
    vector<float> ln1_g, ln1_b;
    vector<float> ln2_g, ln2_b;

    TransformerEncoderLayer(int d_model_, int n_heads_, int d_ff_)
        : d_model(d_model_), n_heads(n_heads_), d_head(d_model_/n_heads_), d_ff(d_ff_),
          Wq(d_model_, d_model_), Wk(d_model_, d_model_), Wv(d_model_, d_model_),
          Wo(d_model_, d_model_),
          W1(d_model_, d_ff_), W2(d_ff_, d_model_),
          ln1_g(d_model_, 1.0f), ln1_b(d_model_, 0.0f),
          ln2_g(d_model_, 1.0f), ln2_b(d_model_, 0.0f)
    {
        assert(d_model % n_heads == 0);

        // init
        init_normal(Wq.W, 0.02f, 1); init_zero(Wq.b);
        init_normal(Wk.W, 0.02f, 2); init_zero(Wk.b);
        init_normal(Wv.W, 0.02f, 3); init_zero(Wv.b);
        init_normal(Wo.W, 0.02f, 4); init_zero(Wo.b);

        init_normal(W1.W, 0.02f, 5); init_zero(W1.b);
        init_normal(W2.W, 0.02f, 6); init_zero(W2.b);
    }

    Tensor2D self_attention(const Tensor2D& X) const {
        // X: (n x d_model)
        Tensor2D Q = Wq.forward(X);  // (n x d_model)
        Tensor2D K = Wk.forward(X);
        Tensor2D V = Wv.forward(X);

        vector<Tensor2D> heads;
        heads.reserve(n_heads);

        const float scale = 1.0f / std::sqrt((float)d_head);

        Tensor2D Kt = transpose(K); // (d_model x n)

        for (int h = 0; h < n_heads; ++h) {
            int c0 = h * d_head;
            int c1 = (h + 1) * d_head;

            Tensor2D Qh = slice_cols(Q, c0, c1);      // (n x d_head)
            Tensor2D Kh = slice_cols(K, c0, c1);      // (n x d_head)
            Tensor2D Vh = slice_cols(V, c0, c1);      // (n x d_head)

            // scores = Qh * Kh^T  => (n x n)
            Tensor2D KhT = transpose(Kh);            // (d_head x n)
            Tensor2D scores = matmul(Qh, KhT);       // (n x n)

            // scale
            for (auto& v : scores.a) v *= scale;

            // softmax row-wise
            softmax_row_inplace(scores);

            // out = scores * Vh => (n x d_head)
            Tensor2D out = matmul(scores, Vh);
            heads.push_back(std::move(out));
        }

        Tensor2D cat = concat_heads(heads);          // (n x d_model)
        Tensor2D attn_out = Wo.forward(cat);         // (n x d_model)
        return attn_out;
    }

    Tensor2D forward(const Tensor2D& X) const {
        // Pre-LN Transformer:
        // Z1 = X + MHA(LN(X))
        // Z2 = Z1 + FFN(LN(Z1))

        Tensor2D X1 = layernorm(X, ln1_g, ln1_b);
        Tensor2D A  = self_attention(X1);
        Tensor2D Z1 = add(X, A);

        Tensor2D Z1n = layernorm(Z1, ln2_g, ln2_b);
        Tensor2D H   = W1.forward(Z1n);
        H = apply_gelu(H);
        Tensor2D F   = W2.forward(H);

        Tensor2D Z2  = add(Z1, F);
        return Z2;
    }
};
#endif