// nav/procedures_test.go
// Copyright(c) 2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"testing"

	"github.com/mmp/vice/math"
	"github.com/mmp/vice/wx"
)

func TestManeuverCompleteUntilAltitude(t *testing.T) {
	nav := &Nav{FlightState: FlightState{Altitude: 499}}
	mc := ManeuverComplete{Type: UntilAltitude, Altitude: 500}

	if mc.Done(nav, Time{}, wx.Sample{}, 0) {
		t.Fatal("expected altitude completion to wait below target altitude")
	}

	nav.FlightState.Altitude = 500
	if !mc.Done(nav, Time{}, wx.Sample{}, 0) {
		t.Fatal("expected altitude completion at target altitude")
	}
}

func TestManeuverCompleteUntilDME(t *testing.T) {
	const nmPerLongitude = 45
	dmeFix := math.Point2LL{-74, 40}
	start := math.Offset2LL(dmeFix, 90, 3, nmPerLongitude)
	crossed := math.Offset2LL(dmeFix, 90, 4.1, nmPerLongitude)

	nav := &Nav{FlightState: FlightState{
		Position: start,
		Altitude: 3000,
	}}
	mc := ManeuverComplete{
		Type:            UntilDME,
		DMEDistance:     4,
		DMEFix:          dmeFix,
		DMEFixElevation: 33,
	}

	if mc.Done(nav, Time{}, wx.Sample{}, 0) {
		t.Fatal("expected DME completion to wait below target distance")
	}

	nav.FlightState.Position = crossed
	if !mc.Done(nav, Time{}, wx.Sample{}, 0) {
		t.Fatal("expected DME completion at target slant distance")
	}
}
