#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

double ocv_from_soc(double soc) {
    if (soc < 0.0) soc = 0.0;
    if (soc > 1.0) soc = 1.0;

    // 一个简单可用的 OCV-SOC 近似函数
    // 后面你可以换成更真实的拟合曲线
    return 3.0 + 1.2 * soc - 0.1 * soc * soc;
}

int main() {
    // ===== 电池参数 =====
    const double Q_ah = 2.3;              // 电池容量 Ah
    const double Q_coulomb = Q_ah * 3600.0;
    const double R0 = 0.015;              // 欧姆内阻
    const double R1 = 0.01;               // RC支路电阻
    const double C1 = 2400.0;             // RC支路电容
    const double dt = 1.0;                // 时间步长 1 s
    const int total_steps = 600;          // 600秒

    // ===== 初值 =====
    double soc = 1.0;                     // 初始SOC 100%
    double v_rc = 0.0;                    // RC支路初始电压

    // ===== 输出文件 =====
    std::ofstream file("data/ecm_data_600s.csv");
    if (!file.is_open()) {
        std::cerr << "Failed to open output file: data/ecm_data_600s.csv" << std::endl;
        return 1;
    }

    file << "time,current,voltage,soc\n";
    file << std::fixed << std::setprecision(6);

    // RC 离散参数
    const double alpha = std::exp(-dt / (R1 * C1));

    for (int k = 0; k <= total_steps; ++k) {
        double time = k * dt;

        // ===== 设定一个简单电流工况 =====
        // 你后面可以替换成自己的工况
        double current = 0.0;
        if (time < 100) {
            current = 1.0;
        } else if (time < 200) {
            current = 2.0;
        } else if (time < 300) {
            current = 0.5;
        } else if (time < 400) {
            current = 1.5;
        } else if (time < 500) {
            current = 0.8;
        } else {
            current = 1.2;
        }

        // ===== 计算输出电压 =====
        double ocv = ocv_from_soc(soc);
        double voltage = ocv - current * R0 - v_rc;

        // 写入当前时刻数据
        file << time << "," << current << "," << voltage << "," << soc << "\n";

        // ===== 更新状态到下一时刻 =====
        double soc_next = soc - (current * dt / Q_coulomb);
        double v_rc_next = alpha * v_rc + R1 * (1.0 - alpha) * current;

        // 限制SOC范围
        if (soc_next < 0.0) soc_next = 0.0;
        if (soc_next > 1.0) soc_next = 1.0;

        soc = soc_next;
        v_rc = v_rc_next;
    }

    file.close();
    std::cout << "CSV generated: data/ecm_data_600s.csv" << std::endl;
    return 0;
}
