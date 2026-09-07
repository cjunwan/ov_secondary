#include <gtest/gtest.h>

#include "pose_graph.h"

namespace {

Eigen::Matrix3d yawRotation(double yaw_degrees)
{
    const double yaw = yaw_degrees * M_PI / 180.0;
    Eigen::Matrix3d rotation;
    rotation << std::cos(yaw), -std::sin(yaw), 0.0,
                std::sin(yaw),  std::cos(yaw), 0.0,
                0.0,            0.0,           1.0;
    return rotation;
}

TEST(RecoveryFactors, TimeIndexedRangesRecoverYawAndTranslation)
{
    constexpr double true_yaw = 32.0;
    const Eigen::Vector3d true_translation(2.0, -1.2, 0.7);
    const Eigen::Matrix3d true_rotation = yawRotation(true_yaw);
    const std::vector<Eigen::Vector3d> neighbors = {
        {5.0, 1.0, 1.4}, {-2.0, 4.0, -0.8}, {0.5, -3.5, 2.2}};

    double yaw[1] = {18.0};
    double translation[3] = {1.5, -0.8, 0.4};
    ceres::Problem problem;
    problem.AddParameterBlock(yaw, 1, AngleLocalParameterization::Create());
    problem.AddParameterBlock(translation, 3);
    for (int index = 0; index < 12; ++index)
    {
        const Eigen::Vector3d local_position(0.35 * index,
                                             std::sin(0.45 * index),
                                             0.25 * std::cos(0.31 * index));
        const Eigen::Vector3d global_position = true_rotation * local_position + true_translation;
        for (const auto &neighbor : neighbors)
        {
            const double distance = (global_position - neighbor).norm();
            problem.AddResidualBlock(
                DistanceFactor::Create(distance, neighbor, local_position, 0.05),
                nullptr, yaw, translation);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ASSERT_TRUE(summary.IsSolutionUsable()) << summary.FullReport();
    EXPECT_NEAR(true_yaw, yaw[0], 1e-5);
    EXPECT_NEAR(true_translation.x(), translation[0], 1e-5);
    EXPECT_NEAR(true_translation.y(), translation[1], 1e-5);
    EXPECT_NEAR(true_translation.z(), translation[2], 1e-5);
}

TEST(RecoveryFactors, StoredToLiveVisualTransformIsInvertedCorrectly)
{
    const Eigen::Matrix3d world_R_stored = yawRotation(20.0);
    const Eigen::Vector3d world_P_stored(2.0, 3.0, 0.4);
    const Eigen::Matrix3d world_R_live = yawRotation(-15.0);
    const Eigen::Vector3d world_P_live(-1.0, 4.0, 1.2);

    const Eigen::Matrix3d live_R_stored = world_R_live.transpose() * world_R_stored;
    const Eigen::Vector3d live_P_stored = world_R_live.transpose() *
                                           (world_P_stored - world_P_live);
    const Eigen::Matrix3d recovered_rotation = world_R_stored * live_R_stored.transpose();
    const Eigen::Vector3d recovered_position = world_P_stored -
                                                recovered_rotation * live_P_stored;

    EXPECT_LT((recovered_rotation - world_R_live).norm(), 1e-12);
    EXPECT_LT((recovered_position - world_P_live).norm(), 1e-12);
}

TEST(RecoveryFactors, PlanarRangeSolveKeepsVerticalContinuity)
{
    constexpr double true_yaw = -27.0;
    const Eigen::Vector3d true_translation(1.3, 2.1, 0.35);
    const Eigen::Matrix3d true_rotation = yawRotation(true_yaw);
    const std::vector<Eigen::Vector3d> neighbors = {
        {-3.0, 0.0, 0.35}, {4.0, -2.0, 0.35}, {2.0, 6.0, 0.35}};

    double yaw[1] = {-10.0};
    double translation[3] = {0.8, 1.6, true_translation.z()};
    ceres::Problem problem;
    problem.AddParameterBlock(yaw, 1, AngleLocalParameterization::Create());
    problem.AddParameterBlock(translation, 3);
    problem.SetParameterization(
        translation, new ceres::SubsetParameterization(3, std::vector<int>{2}));
    for (int index = 0; index < 12; ++index)
    {
        const Eigen::Vector3d local_position(0.25 * index,
                                             0.6 * std::sin(0.4 * index), 0.0);
        const Eigen::Vector3d global_position =
            true_rotation * local_position + true_translation;
        for (const auto &neighbor : neighbors)
        {
            problem.AddResidualBlock(
                DistanceFactor::Create((global_position - neighbor).norm(),
                                       neighbor, local_position, 0.05),
                nullptr, yaw, translation);
        }
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    ASSERT_TRUE(summary.IsSolutionUsable()) << summary.FullReport();
    EXPECT_NEAR(true_yaw, yaw[0], 1e-5);
    EXPECT_NEAR(true_translation.x(), translation[0], 1e-5);
    EXPECT_NEAR(true_translation.y(), translation[1], 1e-5);
    EXPECT_DOUBLE_EQ(true_translation.z(), translation[2]);
}

}  // namespace

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
