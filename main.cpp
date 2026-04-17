#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <filesystem>

using namespace std;
using namespace gtsam;
namespace fs = std::filesystem;

int main(int argc, char** argv) {
    if (argc < 2) {
        cout << "❗用法: ./soc_fgo <输入CSV路径>" << endl;
        return 1;
    }

    string csv_path = argv[1];
    ifstream file(csv_path);
    if (!file.is_open()) {
        cerr << "❌ 无法打开文件: " << csv_path << endl;
        return 1;
    }

    vector<double> time, voltage, current, soc_true, soc_kf;

    string line;
    getline(file, line); // 跳过表头
    while (getline(file, line)) {
        stringstream ss(line);
        string cell;
        vector<string> tokens;
        while (getline(ss, cell, ',')) tokens.push_back(cell);
        if (tokens.size() < 5) continue;

        time.push_back(stod(tokens[0]));
        voltage.push_back(stod(tokens[1]));
        current.push_back(stod(tokens[2]));
        soc_true.push_back(stod(tokens[3]));
        soc_kf.push_back(stod(tokens[4]));
    }
    file.close();

    size_t N = time.size();
    cout << "✅ 已读取数据点数量: " << N << endl;

    NonlinearFactorGraph graph;
    Values initial;

    auto priorNoise = noiseModel::Isotropic::Sigma(1, 0.01);
    auto smoothNoise = noiseModel::Isotropic::Sigma(1, 0.05);

    Symbol x0('x', 0);
    graph.add(PriorFactor<double>(x0, soc_kf[0], priorNoise));
    initial.insert(x0, soc_kf[0]);

    for (size_t i = 1; i < N; ++i) {
        Symbol x_prev('x', i - 1);
        Symbol x_curr('x', i);
        double dt = time[i] - time[i - 1];
        double delta_soc = -current[i - 1] * dt / 3600.0 / 27.0;
        double predicted = soc_kf[i - 1] + delta_soc;
        graph.add(PriorFactor<double>(x_curr, predicted, smoothNoise));
        initial.insert(x_curr, soc_kf[i]);
    }

    LevenbergMarquardtOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();

    string filename = fs::path(csv_path).stem().string();  // 提取文件名（无扩展名）
    string outname = "estimated_SOC_fgo_" + filename + ".csv";

    ofstream out(outname);
    out << "time,SOC_fgo,SOC_kf,SOC_true\n";
    for (size_t i = 0; i < N; ++i) {
        Symbol xi('x', i);
        out << time[i] << "," << result.at<double>(xi) << "," << soc_kf[i] << "," << soc_true[i] << "\n";
    }
    out.close();

    cout << "✅ 结果已保存至: " << outname << endl;
    return 0;
}

