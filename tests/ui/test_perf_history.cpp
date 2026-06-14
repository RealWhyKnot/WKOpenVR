#include <gtest/gtest.h>

#include "PerfHistory.h"

#include <algorithm>
#include <string>

namespace {

namespace modules = openvr_pair::common::modules;
namespace moduleperf = openvr_pair::common::moduleperf;
namespace overlay = openvr_pair::overlay;

uint32_t Slot(modules::ModuleId id)
{
	return moduleperf::SlotIndex(id);
}

} // namespace

TEST(PerfHistory, FloorsUnattributedCpuAtZero)
{
	moduleperf::PerfSampleResult sample{};
	sample.process.cpuValid = true;
	sample.process.cpuPctOneCore = 5.0;
	sample.modules[Slot(modules::ModuleId::Calibration)].active = true;
	sample.modules[Slot(modules::ModuleId::Calibration)].sectionCpuPctOneCore = 4.0;
	sample.modules[Slot(modules::ModuleId::Smoothing)].active = true;
	sample.modules[Slot(modules::ModuleId::Smoothing)].threadCpuPctOneCore = 4.0;

	EXPECT_DOUBLE_EQ(8.0, moduleperf::AttributedProcessCpuPercentOneCore(sample));
	EXPECT_DOUBLE_EQ(0.0, overlay::UnattributedPct(sample));
}

TEST(PerfHistory, ReportsUnattributedCpuWhenProcessExceedsModuleAttribution)
{
	moduleperf::PerfSampleResult sample{};
	sample.process.cpuValid = true;
	sample.process.cpuPctOneCore = 25.0;
	sample.modules[Slot(modules::ModuleId::Calibration)].active = true;
	sample.modules[Slot(modules::ModuleId::Calibration)].sectionCpuPctOneCore = 4.5;
	sample.modules[Slot(modules::ModuleId::Smoothing)].active = true;
	sample.modules[Slot(modules::ModuleId::Smoothing)].threadCpuPctOneCore = 1.5;

	EXPECT_DOUBLE_EQ(6.0, moduleperf::AttributedProcessCpuPercentOneCore(sample));
	EXPECT_DOUBLE_EQ(19.0, moduleperf::UnattributedProcessCpuPercentOneCore(sample));
	EXPECT_DOUBLE_EQ(19.0, overlay::UnattributedPct(sample));

	const std::string line = moduleperf::FormatPerfProcessLine("driver-host", sample);
	EXPECT_NE(std::string::npos, line.find("attributed_pct_one_core=6.00"));
	EXPECT_NE(std::string::npos, line.find("unattributed_pct_one_core=19.00"));
}

TEST(PerfHistory, PushesOverlayAndDriverSeries)
{
	overlay::PerfHistoryStore store;

	moduleperf::PerfSampleResult overlaySample{};
	overlaySample.process.cpuValid = true;
	overlaySample.process.cpuPctOneCore = 12.5;
	overlaySample.process.snapshot.memoryValid = true;
	overlaySample.process.snapshot.workingSetBytes = 64ULL * 1024ULL * 1024ULL;
	overlaySample.modules[Slot(modules::ModuleId::Calibration)].active = true;
	overlaySample.modules[Slot(modules::ModuleId::Calibration)].sectionCpuPctOneCore = 2.0;

	moduleperf::PerfSampleResult driverSample{};
	driverSample.process.cpuValid = true;
	driverSample.process.cpuPctOneCore = 7.0;
	driverSample.process.snapshot.memoryValid = true;
	driverSample.process.snapshot.workingSetBytes = 128ULL * 1024ULL * 1024ULL;
	driverSample.modules[Slot(modules::ModuleId::FaceTracking)].active = true;
	driverSample.modules[Slot(modules::ModuleId::FaceTracking)].threadCpuPctOneCore = 1.0;
	driverSample.modules[Slot(modules::ModuleId::FaceTracking)].sidecarValid = true;
	driverSample.modules[Slot(modules::ModuleId::FaceTracking)].sidecarCpuPctOneCore = 3.5;

	overlay::PushPerfHistory(store, 10.0, overlaySample, true, driverSample);

	EXPECT_EQ(1u, store.overlay.totalCpuPctOneCore.Size());
	EXPECT_FLOAT_EQ(12.5f, store.overlay.totalCpuPctOneCore.Latest());
	EXPECT_FLOAT_EQ(64.0f, store.overlay.workingSetMb.Latest());
	EXPECT_FLOAT_EQ(2.0f, store.overlay.moduleCpuPctOneCore[Slot(modules::ModuleId::Calibration)].Latest());
	EXPECT_FLOAT_EQ(7.0f, store.driver.totalCpuPctOneCore.Latest());
	EXPECT_FLOAT_EQ(128.0f, store.driver.workingSetMb.Latest());
	EXPECT_FLOAT_EQ(1.0f, store.driver.moduleCpuPctOneCore[Slot(modules::ModuleId::FaceTracking)].Latest());
	EXPECT_FLOAT_EQ(3.5f, store.sidecarModuleCpuPctOneCore[Slot(modules::ModuleId::FaceTracking)].Latest());
	EXPECT_FLOAT_EQ(3.5f, store.sidecarTotalCpuPctOneCore.Latest());
}

TEST(PerfHistory, EvictsOldPointsByWindow)
{
	overlay::PerfTimeSeries series;
	series.Push(1.0, 1.0f);
	series.Push(overlay::PerfTimeSeries::kWindowSeconds + 1.0, 2.0f);
	series.Push(overlay::PerfTimeSeries::kWindowSeconds + 1.01, 3.0f);

	ASSERT_EQ(2u, series.Size());
	EXPECT_DOUBLE_EQ(overlay::PerfTimeSeries::kWindowSeconds + 1.0, series.At(0).first);
	EXPECT_FLOAT_EQ(3.0f, series.Latest());
}

TEST(PerfHistory, BuildsMergedModuleRows)
{
	moduleperf::PerfSampleResult overlaySample{};
	overlaySample.process.cpuPctOneCore = 5.0;
	overlaySample.process.snapshot.processId = 111;
	overlaySample.process.snapshot.workingSetBytes = 32ULL * 1024ULL * 1024ULL;
	overlaySample.process.snapshot.handleCount = 10;
	overlaySample.processThreadCount = 4;
	overlaySample.modules[Slot(modules::ModuleId::Calibration)].active = true;
	overlaySample.modules[Slot(modules::ModuleId::Calibration)].sectionCpuPctOneCore = 1.25;

	moduleperf::PerfSampleResult driverSample{};
	driverSample.process.cpuPctOneCore = 6.0;
	driverSample.process.snapshot.processId = 222;
	driverSample.process.snapshot.workingSetBytes = 48ULL * 1024ULL * 1024ULL;
	driverSample.process.snapshot.handleCount = 20;
	driverSample.processThreadCount = 8;
	driverSample.modules[Slot(modules::ModuleId::Calibration)].active = true;
	driverSample.modules[Slot(modules::ModuleId::Calibration)].threadCpuPctOneCore = 2.0;
	driverSample.modules[Slot(modules::ModuleId::Calibration)].sidecarValid = true;
	driverSample.modules[Slot(modules::ModuleId::Calibration)].sidecarCpuPctOneCore = 3.0;
	driverSample.modules[Slot(modules::ModuleId::Calibration)].sidecarThreadCount = 5;
	driverSample.modules[Slot(modules::ModuleId::Calibration)].sidecarPid = 333;
	driverSample.modules[Slot(modules::ModuleId::Calibration)].sidecarProcessCount = 1;
	driverSample.modules[Slot(modules::ModuleId::Calibration)].sidecarWorkingSetBytes = 96ULL * 1024ULL * 1024ULL;

	const overlay::PerfViewModel vm = overlay::BuildPerfViewModel(overlaySample, true, driverSample, 0.25);

	ASSERT_EQ(1u, vm.rows.size());
	EXPECT_TRUE(vm.driverConnected);
	EXPECT_EQ(111u, vm.overlayPid);
	EXPECT_EQ(222u, vm.driverPid);
	EXPECT_DOUBLE_EQ(32.0, vm.overlayWorkingSetMb);
	EXPECT_DOUBLE_EQ(48.0, vm.driverWorkingSetMb);
	EXPECT_DOUBLE_EQ(3.0, vm.sidecarTotalPct);

	const overlay::PerfModuleRow& row = vm.rows.front();
	EXPECT_EQ(modules::ModuleId::Calibration, row.id);
	EXPECT_TRUE(row.overlayActive);
	EXPECT_TRUE(row.driverActive);
	EXPECT_TRUE(row.sidecarPresent);
	EXPECT_DOUBLE_EQ(1.25, row.overlayPct);
	EXPECT_DOUBLE_EQ(2.0, row.driverPct);
	EXPECT_DOUBLE_EQ(3.0, row.sidecarPct);
	EXPECT_DOUBLE_EQ(6.25, row.totalPct);
	EXPECT_EQ(333u, row.sidecarPid);
	EXPECT_EQ(5u, row.sidecarThreads);
	EXPECT_EQ(96ULL * 1024ULL * 1024ULL, row.sidecarWorkingSetBytes);
}

TEST(PerfHistory, IgnoresActiveSparePerfSlots)
{
	size_t moduleCount = 0;
	const modules::ModuleInfo* infos = modules::All(&moduleCount);
	ASSERT_GT(moduleCount, 0u);

	uint32_t lastRegisteredSlot = 0;
	for (size_t i = 0; i < moduleCount; ++i) {
		lastRegisteredSlot = std::max(lastRegisteredSlot, Slot(infos[i].id));
	}
	if (lastRegisteredSlot + 1 >= moduleperf::kSlotCount) {
		GTEST_SKIP() << "No spare perf slots remain.";
	}

	const uint32_t spareSlot = lastRegisteredSlot + 1;
	moduleperf::PerfSampleResult overlaySample{};
	overlaySample.modules[spareSlot].active = true;
	overlaySample.modules[spareSlot].sectionCpuPctOneCore = 42.0;

	moduleperf::PerfSampleResult driverSample{};
	driverSample.modules[spareSlot].active = true;
	driverSample.modules[spareSlot].threadCpuPctOneCore = 7.0;
	driverSample.modules[spareSlot].sidecarValid = true;
	driverSample.modules[spareSlot].sidecarCpuPctOneCore = 3.0;

	const overlay::PerfViewModel vm = overlay::BuildPerfViewModel(overlaySample, true, driverSample, 0.0);

	EXPECT_TRUE(vm.rows.empty());
	EXPECT_DOUBLE_EQ(0.0, vm.sidecarTotalPct);
}

TEST(PerfHistory, EmaSeedsThenSmooths)
{
	bool valid = false;
	double value = overlay::EmaUpdate(0.0, 10.0, valid);
	EXPECT_DOUBLE_EQ(10.0, value);
	valid = true;
	value = overlay::EmaUpdate(value, 20.0, valid, 0.25);
	EXPECT_DOUBLE_EQ(12.5, value);
}

TEST(PerfHistory, PercentileIgnoresSingleOutlier)
{
	overlay::PerfTimeSeries series;
	// Twenty calm samples plus one spike. p95 should sit on the calm shelf
	// while the peak captures the spike.
	for (int i = 0; i < 20; ++i) {
		series.Push(static_cast<double>(i), 5.0f);
	}
	series.Push(20.0, 200.0f);

	EXPECT_FLOAT_EQ(200.0f, series.Max());
	EXPECT_FLOAT_EQ(5.0f, series.Percentile(0.95f));
	EXPECT_FLOAT_EQ(200.0f, series.Percentile(1.0f));
}

TEST(PerfHistory, ComputeSpikeStatsReportsCurrentPeakAndP95)
{
	overlay::PerfTimeSeries series;
	for (int i = 0; i < 50; ++i) {
		series.Push(static_cast<double>(i), 8.0f);
	}
	series.Push(50.0, 150.0f); // a spike one sample ago
	series.Push(51.0, 9.0f);   // settled back down

	const overlay::PerfSpikeStats stats = overlay::ComputeSpikeStats(series);
	EXPECT_FLOAT_EQ(9.0f, static_cast<float>(stats.current)); // current is calm again
	EXPECT_FLOAT_EQ(150.0f, static_cast<float>(stats.peak));  // but the spike is remembered
	EXPECT_NEAR(8.0, stats.p95, 1.0);                         // p95 stays on the calm shelf
}

TEST(PerfHistory, SpikeStatsOnEmptySeriesAreZero)
{
	overlay::PerfTimeSeries series;
	const overlay::PerfSpikeStats stats = overlay::ComputeSpikeStats(series);
	EXPECT_DOUBLE_EQ(0.0, stats.current);
	EXPECT_DOUBLE_EQ(0.0, stats.peak);
	EXPECT_DOUBLE_EQ(0.0, stats.p95);
}
