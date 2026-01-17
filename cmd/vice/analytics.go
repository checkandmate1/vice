// analytics.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package main

import (
	"fmt"
	"sync"
	"time"

	"github.com/mmp/vice/client"
	"github.com/mmp/vice/log"
	"github.com/mmp/vice/server"
)

// ScenarioSession tracks a user's current scenario session
type ScenarioSession struct {
	Facility     string
	GroupName    string
	ScenarioName string
	StartTime    time.Time // Real-world UTC time when scenario started
}

// ScenarioAnalyticsClient manages scenario usage tracking and statistics caching
type ScenarioAnalyticsClient struct {
	currentSession *ScenarioSession
	mu             sync.Mutex
	lg             *log.Logger

	// Cached analytics data
	statsCache      *server.GetAllScenarioStatsResult
	lastStatsFetch  time.Time
	statsFetchMu    sync.RWMutex
	fetchInProgress bool
}

// NewScenarioAnalyticsClient creates a new analytics client
func NewScenarioAnalyticsClient(lg *log.Logger) *ScenarioAnalyticsClient {
	return &ScenarioAnalyticsClient{
		lg: lg,
	}
}

// StartSession begins tracking a new scenario session
func (ac *ScenarioAnalyticsClient) StartSession(facility, groupName, scenarioName string) {
	ac.mu.Lock()
	defer ac.mu.Unlock()

	// If there's an existing session, report it first
	if ac.currentSession != nil {
		ac.reportSessionLocked(nil) // nil client means we can't report, just clear
	}

	ac.currentSession = &ScenarioSession{
		Facility:     facility,
		GroupName:    groupName,
		ScenarioName: scenarioName,
		StartTime:    time.Now().UTC(),
	}

	ac.lg.Infof("Analytics: started session for %s/%s/%s", facility, groupName, scenarioName)
}

// EndSession ends the current session and reports usage if duration >= 3 minutes
func (ac *ScenarioAnalyticsClient) EndSession(cc *client.ControlClient) {
	ac.mu.Lock()
	defer ac.mu.Unlock()

	ac.reportSessionLocked(cc)
}

// reportSessionLocked reports the current session if it meets the minimum duration
// Must be called with ac.mu held
func (ac *ScenarioAnalyticsClient) reportSessionLocked(cc *client.ControlClient) {
	if ac.currentSession == nil {
		return
	}

	duration := time.Since(ac.currentSession.StartTime)

	// Only report if duration >= 3 minutes
	if duration >= 3*time.Minute {
		if cc != nil {
			ac.lg.Infof("Analytics: reporting session %s/%s/%s duration=%s",
				ac.currentSession.Facility,
				ac.currentSession.GroupName,
				ac.currentSession.ScenarioName,
				duration)

			cc.ReportScenarioUsage(
				ac.currentSession.Facility,
				ac.currentSession.GroupName,
				ac.currentSession.ScenarioName,
				ac.currentSession.StartTime,
				duration,
			)
		}
	} else {
		ac.lg.Infof("Analytics: session too short (%s), not reporting", duration)
	}

	ac.currentSession = nil
}

// RefreshStatsIfNeeded fetches new stats from the server if the cache is stale
func (ac *ScenarioAnalyticsClient) RefreshStatsIfNeeded(srv *client.Server) {
	ac.statsFetchMu.Lock()
	if ac.fetchInProgress || time.Since(ac.lastStatsFetch) < 30*time.Second {
		ac.statsFetchMu.Unlock()
		return
	}
	ac.fetchInProgress = true
	ac.statsFetchMu.Unlock()

	go func() {
		defer func() {
			ac.statsFetchMu.Lock()
			ac.fetchInProgress = false
			ac.statsFetchMu.Unlock()
		}()

		if srv == nil {
			return
		}

		result, err := srv.GetAllScenarioStats()
		if err != nil {
			ac.lg.Warnf("Analytics: failed to fetch stats: %v", err)
			return
		}

		ac.statsFetchMu.Lock()
		ac.statsCache = result
		ac.lastStatsFetch = time.Now()
		ac.statsFetchMu.Unlock()

		ac.lg.Debugf("Analytics: refreshed stats cache")
	}()
}

// GetScenarioStats returns cached stats for a specific scenario
func (ac *ScenarioAnalyticsClient) GetScenarioStats(facility, groupName, scenarioName string) *server.ScenarioStats {
	ac.statsFetchMu.RLock()
	defer ac.statsFetchMu.RUnlock()

	if ac.statsCache == nil || ac.statsCache.ScenarioStats == nil {
		return nil
	}

	facilityStats, ok := ac.statsCache.ScenarioStats[facility]
	if !ok {
		return nil
	}

	key := groupName + "/" + scenarioName
	return facilityStats[key]
}

// GetFacilityStats returns cached stats for a facility (TRACON)
func (ac *ScenarioAnalyticsClient) GetFacilityStats(facility string) *server.FacilityStats {
	ac.statsFetchMu.RLock()
	defer ac.statsFetchMu.RUnlock()

	if ac.statsCache == nil || ac.statsCache.FacilityStats == nil {
		return nil
	}

	return ac.statsCache.FacilityStats[facility]
}

// GetARTCCStats returns cached stats for an ARTCC
func (ac *ScenarioAnalyticsClient) GetARTCCStats(artcc string) *server.FacilityStats {
	ac.statsFetchMu.RLock()
	defer ac.statsFetchMu.RUnlock()

	if ac.statsCache == nil || ac.statsCache.ARTCCStats == nil {
		return nil
	}

	return ac.statsCache.ARTCCStats[artcc]
}

// HasStats returns true if we have cached statistics
func (ac *ScenarioAnalyticsClient) HasStats() bool {
	ac.statsFetchMu.RLock()
	defer ac.statsFetchMu.RUnlock()
	return ac.statsCache != nil
}

// FormatScenarioTooltip formats a tooltip string for a scenario
func FormatScenarioTooltip(stats *server.ScenarioStats) string {
	if stats == nil {
		return ""
	}
	return fmt.Sprintf("7d: %d sessions | 30d: %d sessions\nAvg: %s",
		stats.Count7d, stats.Count30d, formatDurationMinutes(stats.AvgDuration))
}

// FormatFacilityTooltip formats a tooltip string for a facility/ARTCC
func FormatFacilityTooltip(stats *server.FacilityStats) string {
	if stats == nil {
		return ""
	}
	return fmt.Sprintf("7d: %d sessions | 30d: %d sessions\nAvg: %s",
		stats.TotalCount7d, stats.TotalCount30d, formatDurationMinutes(stats.TotalAvgDuration))
}

// formatDurationMinutes formats a duration as minutes
func formatDurationMinutes(d time.Duration) string {
	mins := int(d.Minutes())
	if mins < 1 {
		return "<1 min"
	}
	if mins == 1 {
		return "1 min"
	}
	return fmt.Sprintf("%d min", mins)
}
