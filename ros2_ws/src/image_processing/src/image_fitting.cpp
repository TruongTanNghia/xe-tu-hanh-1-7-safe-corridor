#include "image_declarations.h"

#include <Eigen/Dense>

namespace
{
double binomial(int n, int k)
{
    if (k < 0 || k > n) return 0.0;
    double result = 1.0;
    for (int i = 1; i <= k; ++i)
        result *= static_cast<double>(n - (k - i)) / i;
    return result;
}

std::vector<double> denormalize_coefficients(
    const Eigen::VectorXd& normalized, double mean, double scale)
{
    const int degree = static_cast<int>(normalized.size()) - 1;
    std::vector<double> raw(degree + 1, 0.0);
    for (int k = 0; k <= degree; ++k) {
        const double factor = normalized(k) / std::pow(scale, k);
        for (int j = 0; j <= k; ++j) {
            raw[j] += factor * binomial(k, j) *
                      std::pow(-mean, k - j);
        }
    }
    return raw;
}
}  // namespace

// ── poly_fit ─────────────────────────────────────────────────────────────────
// Least-squares polynomial fit bậc `degree` qua N điểm (rows[i], centroid_x[i])
// bằng QR trên tọa độ row đã chuẩn hóa để cubic không bị ill-conditioned.
std::vector<double> ImageProcessor::poly_fit(
    const std::vector<int>&    rows,
    const std::vector<double>& centroid_x,
    int degree)
{
    const int n = static_cast<int>(rows.size());
    if (n != static_cast<int>(centroid_x.size()) || n < degree + 1)
        return {};

    double mean = 0.0;
    for (int row : rows) mean += row;
    mean /= n;

    double scale = 0.0;
    for (int row : rows)
        scale = std::max(scale, std::abs(row - mean));
    if (scale < 1e-9) return {};

    Eigen::MatrixXd A(n, degree + 1);
    Eigen::VectorXd values(n);
    for (int i = 0; i < n; ++i) {
        const double z = (rows[i] - mean) / scale;
        double power = 1.0;
        for (int j = 0; j <= degree; ++j) {
            A(i, j) = power;
            power *= z;
        }
        values(i) = centroid_x[i];
    }

    const Eigen::VectorXd normalized =
        A.colPivHouseholderQr().solve(values);
    return denormalize_coefficients(normalized, mean, scale);
}

// ── draw_poly_curve ───────────────────────────────────────────────────────────
// Evaluate đường cong polynomial từng pixel row rồi tô màu cam
void ImageProcessor::draw_poly_curve(
    unsigned char* d,
    int step, int channels, int width,
    const std::vector<double>& coeffs,
    int scan_top, int scan_bot)
{
    const uint8_t R = 255, G = 140, B = 0;  // màu cam

    for (int y = scan_top; y <= scan_bot; y++) {
        // Evaluate x = c0 + c1*y + ... + c_degree*y^degree.
        double x = 0.0, yp = 1.0;
        for (double c : coeffs) { x += c * yp; yp *= y; }

        int xi = static_cast<int>(std::round(x));
        if (xi < 1 || xi >= width - 1) continue;

        // Vẽ chấm rộng 3px ngang để dễ nhìn
        for (int dx = -1; dx <= 1; dx++) {
            int px = y * step + (xi + dx) * channels;
            d[px] = R; d[px + 1] = G; d[px + 2] = B;
        }
    }
}

// ── publish_fitting_image ─────────────────────────────────────────────────────
// Vẽ lên ảnh sạch: 5 centroid (trắng) + đường cong poly fit (cam) + điểm vanishing (đỏ)
void ImageProcessor::publish_fitting_image(
    const sensor_msgs::msg::Image::SharedPtr src,
    const std::vector<int>&    scan_rows,
    const std::vector<double>& centroid_x,
    const std::vector<bool>& valid_bands,
    const std::vector<double>& poly_coeffs,
    int scan_top, int scan_bot)
{
    auto out = clone_image(src);

    const int width    = static_cast<int>(src->width);
    const int step     = static_cast<int>(src->step);
    const int channels = step / width;
    const int height   = static_cast<int>(src->height);
    unsigned char* d   = out->data.data();

    // 1. Vẽ đường cong polynomial (cam)
    draw_poly_curve(d, step, channels, width, poly_coeffs, scan_top, scan_bot);

    // 2. Vẽ 5 centroid điểm dữ liệu (trắng 5x5)
    for (int i = 0; i < N; i++) {
        if (!valid_bands[i]) continue;
        int ry = scan_rows[i];
        int rx = static_cast<int>(std::round(centroid_x[i]));
        for (int dr = -2; dr <= 2; dr++)
        for (int dc = -2; dc <= 2; dc++) {
            int r = ry + dr, x = rx + dc;
            if (r < 0 || r >= height || x < 0 || x >= width) continue;
            int px = r * step + x * channels;
            d[px] = 0; d[px + 1] = 0; d[px + 2] = 0;
        }
    }

    // 3. Điểm vanishing = giao đường cong với scan_top (đỏ 7x7)
    {
        double x = 0.0, yp = 1.0;
        for (double c : poly_coeffs) { x += c * yp; yp *= scan_top; }
        int vx = static_cast<int>(std::round(x));
        int vy = scan_top;
        for (int dr = -3; dr <= 3; dr++)
        for (int dc = -3; dc <= 3; dc++) {
            int r = vy + dr, xi = vx + dc;
            if (r < 0 || r >= height || xi < 0 || xi >= width) continue;
            int px = r * step + xi * channels;
            d[px] = 0; d[px + 1] = 0; d[px + 2] = 0;  // đỏ
        }
    }

    pub_fitting_image_->publish(*out);
}
