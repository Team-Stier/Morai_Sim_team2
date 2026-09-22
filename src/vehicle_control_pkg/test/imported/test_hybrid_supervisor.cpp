#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "hybrid_path_tracking/hybrid_supervisor.hpp"

namespace hybrid_path_tracking {
namespace {

constexpr double kMaxSteeringRad =
    40.0 * 3.14159265358979323846 / 180.0;

ControllerCandidate ppCandidate(double curvature) {
  return {true, 0.1, 0.2, 0.1, curvature, false, "pure_pursuit"};
}

ControllerCandidate ppCandidateWith(double steer, double cross_track,
                                    double curvature) {
  return {true, steer, cross_track, 0.1, curvature, false, "pure_pursuit"};
}

ControllerCandidate stanleyCandidate() {
  return {true, 0.12, 0.2, 0.1, 0.08, false, "stanley"};
}

ControllerCandidate stanleyCandidateWith(double steer, double cross_track,
                                         double curvature) {
  return {true, steer, cross_track, 0.1, curvature, false, "stanley"};
}

ControllerCandidate invalidCandidate() {
  return {false, 0.0, 0.0, 0.0, 0.0, false, "invalid"};
}

SupervisorConfig defaultSupervisorConfig() { return SupervisorConfig{}; }
SupervisorContext globalContext(double now_sec = 10.0) {
  return {PathSource::GLOBAL, now_sec, true};
}
SupervisorContext visionContext(double now_sec = 10.0) {
  return {PathSource::VISION, now_sec, true};
}
SupervisorContext avoidanceContext(double now_sec = 10.0) {
  return {PathSource::AVOIDANCE, now_sec, true};
}

void enterStanley(HybridSupervisor* supervisor, double now_sec = 10.0) {
  for (int i = 0; i < 3; ++i) {
    supervisor->select(ppCandidate(0.12), stanleyCandidate(),
                       globalContext(now_sec));
  }
  ASSERT_EQ(supervisor->mode(), HybridMode::STANLEY_GLOBAL);
}

TEST(HybridSupervisor, InitialModeIsStop) {
  HybridSupervisor supervisor(defaultSupervisorConfig());

  EXPECT_EQ(supervisor.mode(), HybridMode::STOP);
}

TEST(HybridSupervisor, OmittedPathValidityFailsClosed) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.1);

  const double steer = supervisor.select(
      ppCandidate(0.12), stanleyCandidate(), {PathSource::GLOBAL, 10.0});

  EXPECT_EQ(supervisor.mode(), HybridMode::STOP);
  EXPECT_NEAR(steer, 0.07, 1e-12);
}

TEST(HybridSupervisor, ExplicitValidPathEnablesCandidateSelection) {
  HybridSupervisor supervisor(defaultSupervisorConfig());

  supervisor.select(ppCandidate(0.02), stanleyCandidate(),
                    {PathSource::GLOBAL, 10.0, true});

  EXPECT_EQ(supervisor.mode(), HybridMode::PP_GLOBAL);
}

TEST(HybridSupervisor, HighCurvatureEntersStanleyAfterConfirmation) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  for (int i = 0; i < 2; ++i) {
    supervisor.select(ppCandidate(0.12), stanleyCandidate(), globalContext());
    EXPECT_EQ(supervisor.mode(), HybridMode::PP_GLOBAL);
  }

  supervisor.select(ppCandidate(0.12), stanleyCandidate(), globalContext());
  EXPECT_EQ(supervisor.mode(), HybridMode::STANLEY_GLOBAL);
}

TEST(HybridSupervisor, StanleyEntryCounterResetsWhenTriggerClears) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.select(ppCandidate(0.12), stanleyCandidate(), globalContext());
  supervisor.select(ppCandidate(0.12), stanleyCandidate(), globalContext());
  supervisor.select(ppCandidate(0.05), stanleyCandidate(), globalContext());
  supervisor.select(ppCandidate(0.12), stanleyCandidate(), globalContext());
  supervisor.select(ppCandidate(0.12), stanleyCandidate(), globalContext());

  EXPECT_EQ(supervisor.mode(), HybridMode::PP_GLOBAL);
}

TEST(HybridSupervisor, CurvatureOrCrossTrackCanTriggerStanleyEntry) {
  HybridSupervisor curvature_supervisor(defaultSupervisorConfig());
  HybridSupervisor cross_track_supervisor(defaultSupervisorConfig());

  for (int i = 0; i < 3; ++i) {
    curvature_supervisor.select(
        ppCandidateWith(0.1, 0.2, 0.060001), stanleyCandidate(),
        globalContext());
    cross_track_supervisor.select(
        ppCandidateWith(0.1, -0.800001, 0.02), stanleyCandidate(),
        globalContext());
  }

  EXPECT_EQ(curvature_supervisor.mode(), HybridMode::STANLEY_GLOBAL);
  EXPECT_EQ(cross_track_supervisor.mode(), HybridMode::STANLEY_GLOBAL);
}

TEST(HybridSupervisor, EntryThresholdEqualityDoesNotEnterStanley) {
  HybridSupervisor curvature_supervisor(defaultSupervisorConfig());
  HybridSupervisor cross_track_supervisor(defaultSupervisorConfig());

  for (int i = 0; i < 4; ++i) {
    curvature_supervisor.select(
        ppCandidateWith(0.1, 0.2, 0.06), stanleyCandidate(), globalContext());
    cross_track_supervisor.select(
        ppCandidateWith(0.1, -0.8, 0.02), stanleyCandidate(), globalContext());
  }

  EXPECT_EQ(curvature_supervisor.mode(), HybridMode::PP_GLOBAL);
  EXPECT_EQ(cross_track_supervisor.mode(), HybridMode::PP_GLOBAL);
}

TEST(HybridSupervisor, HysteresisBandKeepsStanleySelected) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  enterStanley(&supervisor);

  for (int i = 0; i < 4; ++i) {
    supervisor.select(ppCandidateWith(0.1, 0.5, 0.05),
                      stanleyCandidateWith(0.12, 0.5, 0.05),
                      globalContext(11.0 + 0.1 * i));
  }

  EXPECT_EQ(supervisor.mode(), HybridMode::STANLEY_GLOBAL);
}

TEST(HybridSupervisor, StanleyExitRequiresConfirmationAndDwell) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  enterStanley(&supervisor, 10.0);
  const auto low_pp = ppCandidateWith(0.1, 0.2, 0.03);
  const auto low_stanley = stanleyCandidateWith(0.12, 0.2, 0.03);

  supervisor.select(low_pp, low_stanley, globalContext(10.2));
  supervisor.select(low_pp, low_stanley, globalContext(10.5));
  supervisor.select(low_pp, low_stanley, globalContext(10.9));
  EXPECT_EQ(supervisor.mode(), HybridMode::STANLEY_GLOBAL);

  supervisor.select(low_pp, low_stanley, globalContext(11.0));
  EXPECT_EQ(supervisor.mode(), HybridMode::PP_GLOBAL);
}

TEST(HybridSupervisor, StanleyExitNeedsBothMetricsBelowExitThresholds) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  enterStanley(&supervisor, 10.0);

  for (int i = 0; i < 3; ++i) {
    supervisor.select(ppCandidateWith(0.1, 0.2, 0.03),
                      stanleyCandidateWith(0.12, 0.5, 0.03),
                      globalContext(11.0 + 0.1 * i));
  }
  EXPECT_EQ(supervisor.mode(), HybridMode::STANLEY_GLOBAL);

  for (int i = 0; i < 3; ++i) {
    supervisor.select(ppCandidateWith(0.1, 0.2, 0.03),
                      stanleyCandidateWith(0.12, 0.399999, 0.039999),
                      globalContext(11.3 + 0.1 * i));
  }
  EXPECT_EQ(supervisor.mode(), HybridMode::PP_GLOBAL);
}

TEST(HybridSupervisor, ExitThresholdEqualityDoesNotExitStanley) {
  HybridSupervisor curvature_supervisor(defaultSupervisorConfig());
  HybridSupervisor cross_track_supervisor(defaultSupervisorConfig());
  enterStanley(&curvature_supervisor, 10.0);
  enterStanley(&cross_track_supervisor, 10.0);

  for (int i = 0; i < 4; ++i) {
    curvature_supervisor.select(
        ppCandidateWith(0.1, 0.2, 0.03),
        stanleyCandidateWith(0.12, 0.2, 0.04),
        globalContext(11.0 + 0.1 * i));
    cross_track_supervisor.select(
        ppCandidateWith(0.1, 0.2, 0.03),
        stanleyCandidateWith(0.12, 0.4, 0.03),
        globalContext(11.0 + 0.1 * i));
  }

  EXPECT_EQ(curvature_supervisor.mode(), HybridMode::STANLEY_GLOBAL);
  EXPECT_EQ(cross_track_supervisor.mode(), HybridMode::STANLEY_GLOBAL);
}

TEST(HybridSupervisor, StanleyExitCounterResetsWhenConditionBreaks) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  enterStanley(&supervisor, 10.0);
  const auto low_pp = ppCandidateWith(0.1, 0.2, 0.03);
  const auto low_stanley = stanleyCandidateWith(0.12, 0.2, 0.03);

  supervisor.select(low_pp, low_stanley, globalContext(11.0));
  supervisor.select(low_pp, low_stanley, globalContext(11.1));
  supervisor.select(ppCandidateWith(0.1, 0.5, 0.05),
                    stanleyCandidateWith(0.12, 0.5, 0.05),
                    globalContext(11.2));
  supervisor.select(low_pp, low_stanley, globalContext(11.3));
  supervisor.select(low_pp, low_stanley, globalContext(11.4));

  EXPECT_EQ(supervisor.mode(), HybridMode::STANLEY_GLOBAL);
}

TEST(HybridSupervisor, VisionAlwaysUsesPurePursuit) {
  HybridSupervisor supervisor(defaultSupervisorConfig());

  supervisor.select(ppCandidate(0.12), stanleyCandidate(), visionContext());
  EXPECT_EQ(supervisor.mode(), HybridMode::PP_VISION);
}

TEST(HybridSupervisor, AvoidanceAlwaysUsesPurePursuit) {
  HybridSupervisor supervisor(defaultSupervisorConfig());

  supervisor.select(ppCandidate(0.12), stanleyCandidate(),
                    avoidanceContext());
  EXPECT_EQ(supervisor.mode(), HybridMode::PP_AVOIDANCE);
}

TEST(HybridSupervisor, IneligibleStanleyNeverLeaksToVisionOrAvoidance) {
  HybridSupervisor vision_supervisor(defaultSupervisorConfig());
  HybridSupervisor avoidance_supervisor(defaultSupervisorConfig());
  vision_supervisor.seedSteering(0.0);
  avoidance_supervisor.seedSteering(0.0);
  const auto stanley = stanleyCandidateWith(0.5, 0.2, 0.08);

  const double vision_steer =
      vision_supervisor.select(invalidCandidate(), stanley, visionContext());
  const double avoidance_steer = avoidance_supervisor.select(
      invalidCandidate(), stanley, avoidanceContext());

  EXPECT_EQ(vision_supervisor.mode(), HybridMode::STOP);
  EXPECT_EQ(avoidance_supervisor.mode(), HybridMode::STOP);
  EXPECT_DOUBLE_EQ(vision_steer, 0.0);
  EXPECT_DOUBLE_EQ(avoidance_steer, 0.0);
}

TEST(HybridSupervisor, InvalidPreferredGlobalControllerUsesValidFallback) {
  HybridSupervisor pp_fallback(defaultSupervisorConfig());
  enterStanley(&pp_fallback);
  pp_fallback.select(ppCandidate(0.12), invalidCandidate(),
                     globalContext(10.1));
  EXPECT_EQ(pp_fallback.mode(), HybridMode::DEGRADED);

  HybridSupervisor stanley_fallback(defaultSupervisorConfig());
  stanley_fallback.select(invalidCandidate(), stanleyCandidate(),
                          globalContext());
  EXPECT_EQ(stanley_fallback.mode(), HybridMode::DEGRADED);
}

TEST(HybridSupervisor, ForcedFallbackDoesNotFlapBackBeforeActualRouteDwell) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  enterStanley(&supervisor, 10.0);

  supervisor.select(ppCandidate(0.12), invalidCandidate(),
                    globalContext(11.0));
  EXPECT_EQ(supervisor.mode(), HybridMode::DEGRADED);

  supervisor.select(ppCandidate(0.12), stanleyCandidate(),
                    globalContext(11.1));
  EXPECT_EQ(supervisor.mode(), HybridMode::DEGRADED);

  supervisor.select(ppCandidate(0.12), stanleyCandidate(),
                    globalContext(12.0));
  EXPECT_EQ(supervisor.mode(), HybridMode::STANLEY_GLOBAL);
}

TEST(HybridSupervisor, InitialForcedFallbackStartsRecoveryDwell) {
  HybridSupervisor supervisor(defaultSupervisorConfig());

  supervisor.select(invalidCandidate(), stanleyCandidate(),
                    globalContext(10.0));
  EXPECT_EQ(supervisor.mode(), HybridMode::DEGRADED);

  supervisor.select(ppCandidate(0.02), stanleyCandidate(),
                    globalContext(10.1));
  EXPECT_EQ(supervisor.mode(), HybridMode::DEGRADED);

  supervisor.select(ppCandidate(0.02), stanleyCandidate(),
                    globalContext(11.0));
  EXPECT_EQ(supervisor.mode(), HybridMode::PP_GLOBAL);
}

TEST(HybridSupervisor, SourceAwayAndBackRequiresActualPpRouteDwell) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  enterStanley(&supervisor, 10.0);

  supervisor.select(ppCandidate(0.12), stanleyCandidate(),
                    visionContext(10.2));
  EXPECT_EQ(supervisor.mode(), HybridMode::PP_VISION);

  supervisor.select(ppCandidate(0.12), stanleyCandidate(),
                    globalContext(10.3));
  supervisor.select(ppCandidate(0.12), stanleyCandidate(),
                    globalContext(10.4));
  supervisor.select(ppCandidate(0.12), stanleyCandidate(),
                    globalContext(10.5));
  EXPECT_EQ(supervisor.mode(), HybridMode::PP_GLOBAL);

  supervisor.select(ppCandidate(0.12), stanleyCandidate(),
                    globalContext(11.2));
  EXPECT_EQ(supervisor.mode(), HybridMode::STANLEY_GLOBAL);
}

TEST(HybridSupervisor, NoValidEligibleCandidateStops) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.1);

  const double steer = supervisor.select(invalidCandidate(),
                                         invalidCandidate(), globalContext());
  EXPECT_EQ(supervisor.mode(), HybridMode::STOP);
  EXPECT_TRUE(std::isfinite(steer));
  EXPECT_NEAR(steer, 0.07, 1e-12);
}

TEST(HybridSupervisor, InvalidSelectedPathStopsBeforeCandidateSelection) {
  PathSourceManager manager(PathSourceConfig{});
  const SelectedPath selected = manager.select({false, false, 10.0});
  ASSERT_FALSE(selected.valid);
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.1);

  const double steer = supervisor.select(
      ppCandidate(0.12), stanleyCandidate(),
      {selected.source, 10.0, selected.valid});

  EXPECT_EQ(supervisor.mode(), HybridMode::STOP);
  EXPECT_TRUE(std::isfinite(steer));
  EXPECT_NEAR(steer, 0.07, 1e-12);
}

TEST(HybridSupervisor, TransitionLimitsFirstCommandStep) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.0);

  const double steer =
      supervisor.select(ppCandidate(0.4), invalidCandidate(), visionContext());
  EXPECT_LE(std::abs(steer), 0.03);
}

TEST(HybridSupervisor, TransferBlendProgressesLinearlyBeforeStepLimit) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.0);
  const auto pp = ppCandidateWith(0.02, 0.2, 0.02);

  const double at_start =
      supervisor.select(pp, invalidCandidate(), visionContext(10.0));
  const double at_half =
      supervisor.select(pp, invalidCandidate(), visionContext(10.25));
  const double at_end =
      supervisor.select(pp, invalidCandidate(), visionContext(10.5));

  EXPECT_NEAR(at_start, 0.0, 1e-12);
  EXPECT_NEAR(at_half, 0.01, 1e-12);
  EXPECT_NEAR(at_end, 0.02, 1e-12);
}

TEST(HybridSupervisor, PpStanleyTransfersBlendFixedEndpointsBothWays) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.0);
  const auto high_pp = ppCandidateWith(0.0, 0.2, 0.12);
  const auto high_stanley = stanleyCandidateWith(0.02, 0.2, 0.08);

  supervisor.select(high_pp, high_stanley, globalContext(10.0));
  supervisor.select(high_pp, high_stanley, globalContext(10.0));
  const double stanley_start =
      supervisor.select(high_pp, high_stanley, globalContext(10.0));
  const double stanley_half =
      supervisor.select(high_pp, high_stanley, globalContext(10.25));
  const double stanley_end =
      supervisor.select(high_pp, high_stanley, globalContext(10.5));

  EXPECT_NEAR(stanley_start, 0.0, 1e-12);
  EXPECT_NEAR(stanley_half, 0.01, 1e-12);
  EXPECT_NEAR(stanley_end, 0.02, 1e-12);

  const auto low_pp = ppCandidateWith(-0.02, 0.2, 0.03);
  const auto low_stanley = stanleyCandidateWith(0.02, 0.2, 0.03);
  supervisor.select(low_pp, low_stanley, globalContext(11.0));
  supervisor.select(low_pp, low_stanley, globalContext(11.0));
  const double pp_start =
      supervisor.select(low_pp, low_stanley, globalContext(11.0));
  const double pp_half =
      supervisor.select(low_pp, low_stanley, globalContext(11.25));
  const double pp_end =
      supervisor.select(low_pp, low_stanley, globalContext(11.5));

  EXPECT_NEAR(pp_start, 0.02, 1e-12);
  EXPECT_NEAR(pp_half, 0.0, 1e-12);
  EXPECT_NEAR(pp_end, -0.02, 1e-12);
}

TEST(HybridSupervisor, BlendEndpointIgnoresLiveCandidateChanges) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.0);

  const double at_start = supervisor.select(
      ppCandidateWith(0.02, 0.2, 0.02), invalidCandidate(),
      visionContext(10.0));
  const double at_half = supervisor.select(
      ppCandidateWith(0.2, 0.2, 0.02), invalidCandidate(),
      visionContext(10.25));
  const double at_end = supervisor.select(
      ppCandidateWith(0.2, 0.2, 0.02), invalidCandidate(),
      visionContext(10.5));
  const double after_blend = supervisor.select(
      ppCandidateWith(0.2, 0.2, 0.02), invalidCandidate(),
      visionContext(10.6));

  EXPECT_NEAR(at_start, 0.0, 1e-12);
  EXPECT_NEAR(at_half, 0.01, 1e-12);
  EXPECT_NEAR(at_end, 0.02, 1e-12);
  EXPECT_NEAR(after_blend, 0.05, 1e-12);
}

TEST(HybridSupervisor, MidBlendSourceChangeRestartsFromCurrentOutput) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.0);

  supervisor.select(ppCandidateWith(0.02, 0.2, 0.02),
                    invalidCandidate(), visionContext(10.0));
  const double before_change = supervisor.select(
      ppCandidateWith(0.02, 0.2, 0.02), invalidCandidate(),
      visionContext(10.25));
  const double at_change = supervisor.select(
      ppCandidateWith(-0.01, 0.2, 0.02), invalidCandidate(),
      avoidanceContext(10.25));
  const double restarted_half = supervisor.select(
      ppCandidateWith(0.2, 0.2, 0.02), invalidCandidate(),
      avoidanceContext(10.5));
  const double restarted_end = supervisor.select(
      ppCandidateWith(0.2, 0.2, 0.02), invalidCandidate(),
      avoidanceContext(10.75));

  EXPECT_NEAR(before_change, 0.01, 1e-12);
  EXPECT_NEAR(at_change, 0.01, 1e-12);
  EXPECT_NEAR(restarted_half, 0.0, 1e-12);
  EXPECT_NEAR(restarted_end, -0.01, 1e-12);
}

TEST(HybridSupervisor, FinalStepLimitAppliesThroughoutTransition) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.0);
  const auto pp = ppCandidateWith(0.6, 0.2, 0.02);

  const double first =
      supervisor.select(pp, invalidCandidate(), visionContext(10.0));
  const double second =
      supervisor.select(pp, invalidCandidate(), visionContext(10.25));
  const double third =
      supervisor.select(pp, invalidCandidate(), visionContext(10.5));

  EXPECT_LE(std::abs(first), 0.03);
  EXPECT_LE(std::abs(second - first), 0.03);
  EXPECT_LE(std::abs(third - second), 0.03);
}

TEST(HybridSupervisor, ConfigurationAboveHardStepLimitFailsSafe) {
  SupervisorConfig config = defaultSupervisorConfig();
  config.max_step_rad = 0.2;
  HybridSupervisor supervisor(config);
  supervisor.seedSteering(0.0);

  const double steer = supervisor.select(
      ppCandidateWith(0.6, 0.2, 0.02), invalidCandidate(),
      visionContext());

  EXPECT_EQ(supervisor.mode(), HybridMode::STOP);
  EXPECT_LE(std::abs(steer), 0.03);
}

TEST(HybridSupervisor, TimestampRollbackAndNonFiniteTimeFailSafe) {
  HybridSupervisor rollback(defaultSupervisorConfig());
  rollback.seedSteering(0.1);
  rollback.select(ppCandidate(0.02), invalidCandidate(), visionContext(10.0));
  const double rollback_steer =
      rollback.select(ppCandidate(0.02), invalidCandidate(),
                      visionContext(9.999));
  EXPECT_EQ(rollback.mode(), HybridMode::STOP);
  EXPECT_TRUE(std::isfinite(rollback_steer));
  EXPECT_LE(std::abs(rollback_steer - 0.1), 0.03 + 1e-12);

  HybridSupervisor nonfinite(defaultSupervisorConfig());
  const double nonfinite_steer = nonfinite.select(
      ppCandidate(0.02), invalidCandidate(),
      visionContext(std::numeric_limits<double>::quiet_NaN()));
  EXPECT_EQ(nonfinite.mode(), HybridMode::STOP);
  EXPECT_TRUE(std::isfinite(nonfinite_steer));
  EXPECT_LE(std::abs(nonfinite_steer), 0.03);
}

TEST(HybridSupervisor, InvalidConfigurationFailsSafe) {
  SupervisorConfig config = defaultSupervisorConfig();
  config.curvature_exit_stanley = 0.07;
  HybridSupervisor supervisor(config);
  supervisor.seedSteering(0.1);

  const double steer =
      supervisor.select(ppCandidate(0.12), stanleyCandidate(), globalContext());
  EXPECT_EQ(supervisor.mode(), HybridMode::STOP);
  EXPECT_TRUE(std::isfinite(steer));
  EXPECT_NEAR(steer, 0.07, 1e-12);
}

TEST(HybridSupervisor, RejectsConfigurationJustOutsideSafetyEnvelope) {
  SupervisorConfig configs[] = {
      defaultSupervisorConfig(), defaultSupervisorConfig(),
      defaultSupervisorConfig(), defaultSupervisorConfig(),
      defaultSupervisorConfig(), defaultSupervisorConfig(),
      defaultSupervisorConfig(), defaultSupervisorConfig()};
  configs[0].curvature_enter_stanley = 0.059999;
  configs[1].curvature_exit_stanley = 0.040001;
  configs[2].cross_track_enter_stanley_m = 0.799999;
  configs[3].cross_track_exit_stanley_m = 0.400001;
  configs[4].confirmation_samples = 2;
  configs[5].minimum_dwell_sec = 0.999999;
  configs[6].blend_duration_sec = 0.499999;
  configs[7].max_step_rad = 0.030001;

  for (int i = 0; i < 8; ++i) {
    SCOPED_TRACE(i);
    HybridSupervisor supervisor(configs[i]);
    supervisor.seedSteering(0.1);
    const double steer = supervisor.select(
        ppCandidate(0.12), stanleyCandidate(), globalContext());
    EXPECT_EQ(supervisor.mode(), HybridMode::STOP);
    EXPECT_TRUE(std::isfinite(steer));
    EXPECT_NEAR(steer, 0.07, 1e-12);
  }
}

TEST(HybridSupervisor, NonFiniteCandidateCannotReachOutput) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.0);
  ControllerCandidate pp = ppCandidate(0.02);
  pp.steering_angle_rad = std::numeric_limits<double>::quiet_NaN();

  const double steer =
      supervisor.select(pp, stanleyCandidate(), visionContext());
  EXPECT_EQ(supervisor.mode(), HybridMode::STOP);
  EXPECT_TRUE(std::isfinite(steer));
  EXPECT_LE(std::abs(steer), 0.03);
}

TEST(HybridSupervisor, OverLimitCandidateUsesEligibleFallbackAndStaysLimited) {
  HybridSupervisor supervisor(defaultSupervisorConfig());
  supervisor.seedSteering(0.0);
  ControllerCandidate pp = ppCandidate(0.02);
  pp.steering_angle_rad = kMaxSteeringRad + 0.001;

  const double steer =
      supervisor.select(pp, stanleyCandidate(), globalContext());
  EXPECT_EQ(supervisor.mode(), HybridMode::DEGRADED);
  EXPECT_TRUE(std::isfinite(steer));
  EXPECT_LE(std::abs(steer), 0.03);
  EXPECT_LE(std::abs(steer), kMaxSteeringRad);
}

}  // namespace
}  // namespace hybrid_path_tracking
