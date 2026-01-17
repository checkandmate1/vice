// server/analytics.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package server

import (
	"encoding/json"
	"os"
	"path/filepath"
	"sync"
	"time"

	"github.com/mmp/vice/log"
)

// ScenarioUsageRecord represents a single session record
type ScenarioUsageRecord struct {
	StartTime time.Time     `json:"start_time"`
	Duration  time.Duration `json:"duration"`
}

// ScenarioAnalytics stores usage data for all scenarios in a facility
type ScenarioAnalytics struct {
	Facility  string                             `json:"facility"`
	Scenarios map[string][]ScenarioUsageRecord `json:"scenarios"` // key: "GroupName/ScenarioName"
}

// ScenarioStats contains pre-computed statistics for a scenario
type ScenarioStats struct {
	Count24h    int           `json:"count_24h"`
	Count7d     int           `json:"count_7d"`
	Count30d    int           `json:"count_30d"`
	Count6m     int           `json:"count_6m"`
	AvgDuration time.Duration `json:"avg_duration"`
}

// FacilityStats aggregates stats across all scenarios in a facility
type FacilityStats struct {
	TotalCount24h    int           `json:"total_count_24h"`
	TotalCount7d     int           `json:"total_count_7d"`
	TotalCount30d    int           `json:"total_count_30d"`
	TotalCount6m     int           `json:"total_count_6m"`
	TotalAvgDuration time.Duration `json:"total_avg_duration"`
}

// AnalyticsManager handles scenario usage analytics storage and retrieval
type AnalyticsManager struct {
	dataByFacility map[string]*ScenarioAnalytics
	dataDir        string
	mu             sync.RWMutex
	lg             *log.Logger
	dirty          bool
}

// NewAnalyticsManager creates a new analytics manager
func NewAnalyticsManager(lg *log.Logger) *AnalyticsManager {
	am := &AnalyticsManager{
		dataByFacility: make(map[string]*ScenarioAnalytics),
		dataDir:        "analytics",
		lg:             lg,
	}
	am.loadAll()
	go am.periodicSave()
	return am
}

// loadAll loads all analytics files from the data directory
func (am *AnalyticsManager) loadAll() {
	if err := os.MkdirAll(am.dataDir, 0755); err != nil {
		am.lg.Errorf("Failed to create analytics directory: %v", err)
		return
	}

	entries, err := os.ReadDir(am.dataDir)
	if err != nil {
		am.lg.Errorf("Failed to read analytics directory: %v", err)
		return
	}

	for _, entry := range entries {
		if entry.IsDir() || filepath.Ext(entry.Name()) != ".json" {
			continue
		}

		path := filepath.Join(am.dataDir, entry.Name())
		data, err := os.ReadFile(path)
		if err != nil {
			am.lg.Errorf("Failed to read analytics file %s: %v", path, err)
			continue
		}

		var analytics ScenarioAnalytics
		if err := json.Unmarshal(data, &analytics); err != nil {
			am.lg.Errorf("Failed to parse analytics file %s: %v", path, err)
			continue
		}

		am.dataByFacility[analytics.Facility] = &analytics
	}

	am.lg.Infof("Loaded analytics for %d facilities", len(am.dataByFacility))
}

// periodicSave saves dirty analytics data every 5 minutes
func (am *AnalyticsManager) periodicSave() {
	ticker := time.NewTicker(5 * time.Minute)
	for range ticker.C {
		am.saveAllIfDirty()
		am.pruneOldRecords()
	}
}

// saveAllIfDirty saves all analytics data if there have been changes
func (am *AnalyticsManager) saveAllIfDirty() {
	am.mu.Lock()
	defer am.mu.Unlock()

	if !am.dirty {
		return
	}

	for facility, analytics := range am.dataByFacility {
		path := filepath.Join(am.dataDir, facility+".json")
		data, err := json.MarshalIndent(analytics, "", "  ")
		if err != nil {
			am.lg.Errorf("Failed to marshal analytics for %s: %v", facility, err)
			continue
		}

		if err := os.WriteFile(path, data, 0644); err != nil {
			am.lg.Errorf("Failed to write analytics file %s: %v", path, err)
			continue
		}
	}

	am.dirty = false
	am.lg.Infof("Saved analytics for %d facilities", len(am.dataByFacility))
}

// pruneOldRecords removes records older than 6 months
func (am *AnalyticsManager) pruneOldRecords() {
	am.mu.Lock()
	defer am.mu.Unlock()

	cutoff := time.Now().Add(-6 * 30 * 24 * time.Hour)

	for _, analytics := range am.dataByFacility {
		for key, records := range analytics.Scenarios {
			var kept []ScenarioUsageRecord
			for _, r := range records {
				if r.StartTime.After(cutoff) {
					kept = append(kept, r)
				}
			}
			if len(kept) != len(records) {
				analytics.Scenarios[key] = kept
				am.dirty = true
			}
		}
	}
}

// RecordUsage records a scenario usage session
func (am *AnalyticsManager) RecordUsage(facility, groupName, scenarioName string, startTime time.Time, duration time.Duration) {
	am.mu.Lock()
	defer am.mu.Unlock()

	analytics, ok := am.dataByFacility[facility]
	if !ok {
		analytics = &ScenarioAnalytics{
			Facility:  facility,
			Scenarios: make(map[string][]ScenarioUsageRecord),
		}
		am.dataByFacility[facility] = analytics
	}

	key := groupName + "/" + scenarioName
	analytics.Scenarios[key] = append(analytics.Scenarios[key], ScenarioUsageRecord{
		StartTime: startTime,
		Duration:  duration,
	})

	am.dirty = true
	am.lg.Infof("Recorded usage: %s/%s duration=%s", facility, key, duration)
}

// ComputeScenarioStats computes statistics for a specific scenario
func (am *AnalyticsManager) ComputeScenarioStats(facility, groupName, scenarioName string) *ScenarioStats {
	am.mu.RLock()
	defer am.mu.RUnlock()

	analytics, ok := am.dataByFacility[facility]
	if !ok {
		return nil
	}

	key := groupName + "/" + scenarioName
	records, ok := analytics.Scenarios[key]
	if !ok || len(records) == 0 {
		return nil
	}

	return computeStatsFromRecords(records)
}

// ComputeFacilityStats computes aggregate statistics for all scenarios in a facility
func (am *AnalyticsManager) ComputeFacilityStats(facility string) *FacilityStats {
	am.mu.RLock()
	defer am.mu.RUnlock()

	analytics, ok := am.dataByFacility[facility]
	if !ok {
		return nil
	}

	// Collect all records from all scenarios
	var allRecords []ScenarioUsageRecord
	for _, records := range analytics.Scenarios {
		allRecords = append(allRecords, records...)
	}

	if len(allRecords) == 0 {
		return nil
	}

	stats := computeStatsFromRecords(allRecords)
	return &FacilityStats{
		TotalCount24h:    stats.Count24h,
		TotalCount7d:     stats.Count7d,
		TotalCount30d:    stats.Count30d,
		TotalCount6m:     stats.Count6m,
		TotalAvgDuration: stats.AvgDuration,
	}
}

// GetAllStats returns all scenario stats for a facility (for client caching)
func (am *AnalyticsManager) GetAllStats(facility string) map[string]*ScenarioStats {
	am.mu.RLock()
	defer am.mu.RUnlock()

	analytics, ok := am.dataByFacility[facility]
	if !ok {
		return nil
	}

	result := make(map[string]*ScenarioStats)
	for key, records := range analytics.Scenarios {
		if len(records) > 0 {
			result[key] = computeStatsFromRecords(records)
		}
	}

	return result
}

// computeStatsFromRecords computes statistics from a slice of records
func computeStatsFromRecords(records []ScenarioUsageRecord) *ScenarioStats {
	now := time.Now()
	stats := &ScenarioStats{}
	var totalDuration time.Duration
	count := 0

	for _, record := range records {
		age := now.Sub(record.StartTime)

		if age <= 24*time.Hour {
			stats.Count24h++
		}
		if age <= 7*24*time.Hour {
			stats.Count7d++
		}
		if age <= 30*24*time.Hour {
			stats.Count30d++
		}
		if age <= 6*30*24*time.Hour {
			stats.Count6m++
			totalDuration += record.Duration
			count++
		}
	}

	if count > 0 {
		stats.AvgDuration = totalDuration / time.Duration(count)
	}

	return stats
}

///////////////////////////////////////////////////////////////////////////
// RPC Handlers

// ReportScenarioUsageRequest is the request for reporting scenario usage
type ReportScenarioUsageRequest struct {
	Facility     string        `json:"facility"`
	GroupName    string        `json:"group_name"`
	ScenarioName string        `json:"scenario_name"`
	StartTime    time.Time     `json:"start_time"`
	Duration     time.Duration `json:"duration"`
}

const ReportScenarioUsageRPC = "SimManager.ReportScenarioUsage"

// ReportScenarioUsage records a scenario usage session from a client
func (sm *SimManager) ReportScenarioUsage(req *ReportScenarioUsageRequest, _ *struct{}) error {
	defer sm.lg.CatchAndReportCrash()

	if sm.analyticsManager == nil {
		return nil // Analytics not enabled
	}

	// Validate minimum duration (3 minutes)
	if req.Duration < 3*time.Minute {
		return nil // Silently ignore short sessions
	}

	sm.analyticsManager.RecordUsage(req.Facility, req.GroupName, req.ScenarioName, req.StartTime, req.Duration)
	return nil
}

// GetScenarioStatsRequest is the request for getting scenario statistics
type GetScenarioStatsRequest struct {
	Facilities []string `json:"facilities"` // List of facilities to get stats for
}

// GetScenarioStatsResult contains statistics for requested facilities
type GetScenarioStatsResult struct {
	// Stats by facility, then by "GroupName/ScenarioName"
	ScenarioStats map[string]map[string]*ScenarioStats `json:"scenario_stats"`
	FacilityStats map[string]*FacilityStats            `json:"facility_stats"`
}

const GetScenarioStatsRPC = "SimManager.GetScenarioStats"

// GetScenarioStats returns scenario usage statistics
func (sm *SimManager) GetScenarioStats(req *GetScenarioStatsRequest, result *GetScenarioStatsResult) error {
	defer sm.lg.CatchAndReportCrash()

	if sm.analyticsManager == nil {
		return nil // Analytics not enabled
	}

	result.ScenarioStats = make(map[string]map[string]*ScenarioStats)
	result.FacilityStats = make(map[string]*FacilityStats)

	for _, facility := range req.Facilities {
		scenarioStats := sm.analyticsManager.GetAllStats(facility)
		if scenarioStats != nil {
			result.ScenarioStats[facility] = scenarioStats
		}

		facilityStats := sm.analyticsManager.ComputeFacilityStats(facility)
		if facilityStats != nil {
			result.FacilityStats[facility] = facilityStats
		}
	}

	return nil
}

// GetAllScenarioStats returns all scenario statistics (for initial load)
type GetAllScenarioStatsResult struct {
	// Stats by facility, then by "GroupName/ScenarioName"
	ScenarioStats map[string]map[string]*ScenarioStats `json:"scenario_stats"`
	FacilityStats map[string]*FacilityStats            `json:"facility_stats"`
	// Stats aggregated by ARTCC
	ARTCCStats map[string]*FacilityStats `json:"artcc_stats"`
}

const GetAllScenarioStatsRPC = "SimManager.GetAllScenarioStats"

// GetAllScenarioStats returns all scenario usage statistics
func (sm *SimManager) GetAllScenarioStats(_ struct{}, result *GetAllScenarioStatsResult) error {
	defer sm.lg.CatchAndReportCrash()

	if sm.analyticsManager == nil {
		return nil // Analytics not enabled
	}

	result.ScenarioStats = make(map[string]map[string]*ScenarioStats)
	result.FacilityStats = make(map[string]*FacilityStats)
	result.ARTCCStats = make(map[string]*FacilityStats)

	sm.analyticsManager.mu.RLock()
	defer sm.analyticsManager.mu.RUnlock()

	// Collect records by ARTCC for aggregation
	artccRecords := make(map[string][]ScenarioUsageRecord)

	for facility, analytics := range sm.analyticsManager.dataByFacility {
		// Get scenario stats for this facility
		scenarioStats := make(map[string]*ScenarioStats)
		var facilityRecords []ScenarioUsageRecord

		for key, records := range analytics.Scenarios {
			if len(records) > 0 {
				scenarioStats[key] = computeStatsFromRecords(records)
				facilityRecords = append(facilityRecords, records...)
			}
		}

		if len(scenarioStats) > 0 {
			result.ScenarioStats[facility] = scenarioStats
		}

		if len(facilityRecords) > 0 {
			stats := computeStatsFromRecords(facilityRecords)
			result.FacilityStats[facility] = &FacilityStats{
				TotalCount24h:    stats.Count24h,
				TotalCount7d:     stats.Count7d,
				TotalCount30d:    stats.Count30d,
				TotalCount6m:     stats.Count6m,
				TotalAvgDuration: stats.AvgDuration,
			}

			// Determine ARTCC for this facility
			artcc := getARTCCForFacility(facility, sm.scenarioCatalogs)
			if artcc != "" {
				artccRecords[artcc] = append(artccRecords[artcc], facilityRecords...)
			}
		}
	}

	// Compute ARTCC stats
	for artcc, records := range artccRecords {
		if len(records) > 0 {
			stats := computeStatsFromRecords(records)
			result.ARTCCStats[artcc] = &FacilityStats{
				TotalCount24h:    stats.Count24h,
				TotalCount7d:     stats.Count7d,
				TotalCount30d:    stats.Count30d,
				TotalCount6m:     stats.Count6m,
				TotalAvgDuration: stats.AvgDuration,
			}
		}
	}

	return nil
}

// getARTCCForFacility returns the ARTCC code for a facility
func getARTCCForFacility(facility string, catalogs map[string]map[string]*ScenarioCatalog) string {
	for _, groups := range catalogs {
		for _, catalog := range groups {
			if catalog.Facility == facility {
				return catalog.ARTCC
			}
		}
	}
	// If not found in TRACONs, it might be an ARTCC itself
	return facility
}

// Shutdown saves any pending analytics data
func (am *AnalyticsManager) Shutdown() {
	am.saveAllIfDirty()
}
