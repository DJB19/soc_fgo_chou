#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>

#include <iostream>

using namespace std;
using namespace gtsam;

int main() {
    NonlinearFactorGraph graph;

    // 创建变量 symbol：x0, x1
    Symbol x0('x', 0);
    Symbol x1('x', 1);

    // 添加先验因子：x0 ≈ 0
    auto priorNoise = noiseModel::Diagonal::Sigmas(Vector1(0.1));
    graph.add(PriorFactor<double>(x0, 0.0, priorNoise));

    // 添加 x0 与 x1 的相对因子：x1 = x0 + 1
    auto betweenNoise = noiseModel::Diagonal::Sigmas(Vector1(0.1));
    graph.add(BetweenFactor<double>(x0, x1, 1.0, betweenNoise));

    // 初始估计
    Values initial;
    initial.insert(x0, 0.5);  // 错一点没关系
    initial.insert(x1, 1.8);  // 错一点没关系

    // 优化
    LevenbergMarquardtOptimizer optimizer(graph, initial);
    Values result = optimizer.optimize();

    // 输出优化结果
    cout << "优化后结果：" << endl;
    result.print("结果");

    return 0;
}
