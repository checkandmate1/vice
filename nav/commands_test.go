// nav/commands_test.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"fmt"
	"testing"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
)

// TestCommandValidation verifies that invalid commands produce UnableIntents.
func TestCommandValidation(t *testing.T) {
	makeNav := func(t *testing.T) *FlightTest {
		t.Helper()
		return NewArrivalFlight(t, ArrivalConfig{
			Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
			DepartureAirport: "KMCO",
			ArrivalAirport:   "KJFK",
			AircraftType:     "A320",
			InitialAltitude:  11000,
			InitialSpeed:     250,
		})
	}

	t.Run("AltitudeAboveCeiling", func(t *testing.T) {
		f := makeNav(t)
		intent := f.nav.AssignAltitude(99000, false, f.simTime)
		AssertUnable(t, intent)
	})

	t.Run("SpeedBelowLanding", func(t *testing.T) {
		f := makeNav(t)
		sr := av.MakeAtSpeedRestriction(80)
		intent := f.nav.AssignSpeed(&sr, false)
		AssertUnable(t, intent)
	})

	t.Run("DirectFixInvalid", func(t *testing.T) {
		f := makeNav(t)
		intent := f.nav.DirectFix("ZZZZZZ", av.TurnClosest, f.simTime)
		AssertUnable(t, intent)
	})

	t.Run("CrossFixNotInRoute", func(t *testing.T) {
		f := makeNav(t)
		ar := av.MakeAtAltitudeRestriction(5000)
		intent := f.nav.CrossFixAt("NOTINROUTE", &ar, nil)
		AssertUnable(t, intent)
	})

	t.Run("ClearedApproachWithoutExpect", func(t *testing.T) {
		f := makeNav(t)
		intent, _ := f.nav.ClearedApproach(f.fp.ArrivalAirport, "I22L", false, f.simTime)
		AssertUnable(t, intent)
	})

	t.Run("DescendViaSTARNotOnSTAR", func(t *testing.T) {
		// Create a nav without OnSTAR waypoints
		f := NewArrivalFlight(t, ArrivalConfig{
			Waypoints:        "SAJUL DETGY HAUPT",
			DepartureAirport: "KMCO",
			ArrivalAirport:   "KJFK",
			AircraftType:     "A320",
			InitialAltitude:  11000,
			InitialSpeed:     250,
		})
		intent := f.nav.DescendViaSTAR(f.simTime)
		AssertUnable(t, intent)
	})

	t.Run("ExpediteDescentWhenLevel", func(t *testing.T) {
		// Aircraft at 11000, no assigned descent — level flight
		f := NewArrivalFlight(t, ArrivalConfig{
			Waypoints:        "SAJUL DETGY HAUPT",
			DepartureAirport: "KMCO",
			ArrivalAirport:   "KJFK",
			AircraftType:     "A320",
			InitialAltitude:  11000,
			InitialSpeed:     250,
			AssignedAltitude: 11000,
		})
		intent := f.nav.ExpediteDescent()
		AssertUnable(t, intent)
	})
}

// TestLeftDirectFix verifies that DirectFix with TurnLeft turns the long way.
func TestLeftDirectFix(t *testing.T) {
	// Aircraft heading north (360). Fix is DETGY which is roughly east
	// of SAJUL. By requesting TurnLeft, the aircraft should turn counter-
	// clockwise (heading decreasing from 360 toward 270).
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/star DETGY/star HAUPT/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
		OnSTAR:           true,
	})

	// Assign heading north first, then direct to DETGY with left turn
	f.AssignHeading(360, av.TurnClosest)

	f.AfterTicks(30, func(f *FlightTest) {
		f.DirectFixWithTurn("DETGY", av.TurnLeft)
	})

	// Check after some time that the deferred heading includes a left turn
	f.AfterTicks(60, func(f *FlightTest) {
		if dh := f.nav.DeferredNavHeading; dh != nil && dh.Turn != nil {
			if *dh.Turn != av.TurnLeft {
				t.Errorf("expected TurnLeft in deferred heading, got %v", *dh.Turn)
			}
		}
	})

	f.AtFix("DETGY", func(f *FlightTest) {
		// Just verify the fix was reached
	})

	f.Run()
}

// TestRightDirectFix verifies that DirectFix with TurnRight turns the short way.
func TestRightDirectFix(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/star DETGY/star HAUPT/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  11000,
		InitialSpeed:     250,
		OnSTAR:           true,
	})

	f.AssignHeading(360, av.TurnClosest)

	f.AfterTicks(30, func(f *FlightTest) {
		f.DirectFixWithTurn("DETGY", av.TurnRight)
	})

	f.AfterTicks(60, func(f *FlightTest) {
		if dh := f.nav.DeferredNavHeading; dh != nil && dh.Turn != nil {
			if *dh.Turn != av.TurnRight {
				t.Errorf("expected TurnRight in deferred heading, got %v", *dh.Turn)
			}
		}
	})

	f.AtFix("DETGY", func(f *FlightTest) {
		// Verify fix reached
	})

	f.Run()
}

// TestExpectDirectReducesDelay verifies that calling ExpectDirect before
// DirectFix results in a shorter pilot delay.
func TestExpectDirectReducesDelay(t *testing.T) {
	makeFlight := func(t *testing.T) *FlightTest {
		return NewArrivalFlight(t, ArrivalConfig{
			Waypoints:        "SAJUL/star DETGY/star HAUPT/star",
			DepartureAirport: "KMCO",
			ArrivalAirport:   "KJFK",
			AircraftType:     "A320",
			InitialAltitude:  11000,
			InitialSpeed:     250,
			OnSTAR:           true,
		})
	}

	// Without ExpectDirect
	fNoExpect := makeFlight(t)
	fNoExpect.nav.AssignHeading(math.MagneticHeading(360), av.TurnClosest, fNoExpect.simTime)
	// Wait for heading to take effect
	for i := 0; i < 10; i++ {
		wxs := fNoExpect.weather(fNoExpect.nav.FlightState.Altitude)
		fNoExpect.nav.UpdateWithWeather(fNoExpect.callsign, wxs, &fNoExpect.fp, fNoExpect.simTime, nil)
		fNoExpect.simTime = fNoExpect.simTime.Add(1e9) // 1 second
	}
	fNoExpect.nav.DirectFix("DETGY", av.TurnClosest, fNoExpect.simTime)
	noExpectDelay := fNoExpect.nav.DeferredNavHeading.Time

	// With ExpectDirect
	fExpect := makeFlight(t)
	fExpect.nav.AssignHeading(math.MagneticHeading(360), av.TurnClosest, fExpect.simTime)
	for i := 0; i < 10; i++ {
		wxs := fExpect.weather(fExpect.nav.FlightState.Altitude)
		fExpect.nav.UpdateWithWeather(fExpect.callsign, wxs, &fExpect.fp, fExpect.simTime, nil)
		fExpect.simTime = fExpect.simTime.Add(1e9)
	}
	fExpect.nav.ExpectDirect("DETGY")
	fExpect.nav.DirectFix("DETGY", av.TurnClosest, fExpect.simTime)
	expectDelay := fExpect.nav.DeferredNavHeading.Time

	// With expect, the delay should be shorter (earlier time)
	if !expectDelay.Before(noExpectDelay) {
		t.Errorf("ExpectDirect did not reduce delay: expect=%v noExpect=%v", expectDelay, noExpectDelay)
	}
}

// TestCrossDistanceFromFixAtDirectionUnable verifies that
// CrossDistanceFromFixAt returns unable when the direction is wrong.
func TestCrossDistanceFromFixAtDirectionUnable(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	// Determine the actual approach direction (SAJUL→DETGY), then pick the opposite.
	fixLoc := f.nav.Waypoints[1].Location   // DETGY
	priorLoc := f.nav.Waypoints[0].Location // SAJUL
	approachHeading := math.TrueToMagnetic(
		math.Heading2LL(fixLoc, priorLoc, f.nav.FlightState.NmPerLongitude),
		f.nav.FlightState.MagneticVariation,
	)
	oppositeDir := math.ShortCompass(math.OppositeHeading(approachHeading))
	dir, err := math.ParseCardinalOrdinalDirection(oppositeDir)
	if err != nil {
		t.Fatalf("failed to parse direction: %v", err)
	}

	ar := av.MakeAtAltitudeRestriction(5000)
	intent := f.nav.CrossDistanceFromFixAt("DETGY", 5, dir, &ar, nil)
	AssertUnable(t, intent)
}

// TestCrossDistanceFromFixAtDistanceUnable verifies that
// CrossDistanceFromFixAt returns unable when the distance exceeds the segment.
func TestCrossDistanceFromFixAtDistanceUnable(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	// Determine the correct direction.
	fixLoc := f.nav.Waypoints[1].Location
	priorLoc := f.nav.Waypoints[0].Location
	approachHeading := math.TrueToMagnetic(
		math.Heading2LL(fixLoc, priorLoc, f.nav.FlightState.NmPerLongitude),
		f.nav.FlightState.MagneticVariation,
	)
	approachDir := math.ShortCompass(approachHeading)
	dir, err := math.ParseCardinalOrdinalDirection(approachDir)
	if err != nil {
		t.Fatalf("failed to parse direction: %v", err)
	}

	// Distance of 9999 miles should exceed any segment.
	ar := av.MakeAtAltitudeRestriction(5000)
	intent := f.nav.CrossDistanceFromFixAt("DETGY", 9999, dir, &ar, nil)
	AssertUnable(t, intent)
}

// TestCrossDistanceFromFixAtNotInRoute verifies that an unknown fix returns unable.
func TestCrossDistanceFromFixAtNotInRoute(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	ar := av.MakeAtAltitudeRestriction(5000)
	intent := f.nav.CrossDistanceFromFixAt("BOGUS", 5, math.West, &ar, nil)
	AssertUnable(t, intent)
}

// TestCrossDistanceFromFixAtInsertsWaypoint verifies that a synthetic waypoint
// is inserted at the correct position in the route.
func TestCrossDistanceFromFixAtInsertsWaypoint(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	// Find the correct approach direction.
	fixLoc := f.nav.Waypoints[1].Location
	priorLoc := f.nav.Waypoints[0].Location
	approachHeading := math.TrueToMagnetic(
		math.Heading2LL(fixLoc, priorLoc, f.nav.FlightState.NmPerLongitude),
		f.nav.FlightState.MagneticVariation,
	)
	approachDir := math.ShortCompass(approachHeading)
	dir, err := math.ParseCardinalOrdinalDirection(approachDir)
	if err != nil {
		t.Fatalf("failed to parse direction: %v", err)
	}

	wpsBefore := len(f.nav.Waypoints)
	ar := av.MakeAtAltitudeRestriction(5000)
	intent := f.nav.CrossDistanceFromFixAt("DETGY", 5, dir, &ar, nil)
	if _, ok := intent.(av.UnableIntent); ok {
		t.Fatalf("unexpected unable: %v", intent)
	}

	// Waypoints should have one more entry.
	if len(f.nav.Waypoints) != wpsBefore+1 {
		t.Errorf("expected %d waypoints, got %d", wpsBefore+1, len(f.nav.Waypoints))
	}

	// The synthetic waypoint should be before DETGY (at index 1).
	if f.nav.Waypoints[1].Fix[0] != '_' {
		t.Errorf("expected synthetic waypoint (underscore prefix) at index 1, got %q",
			f.nav.Waypoints[1].Fix)
	}
	if !f.nav.Waypoints[1].OnSTAR() {
		t.Errorf("expected synthetic waypoint to preserve STAR membership")
	}
	if f.nav.Waypoints[2].Fix != "DETGY" {
		t.Errorf("expected DETGY at index 2, got %q", f.nav.Waypoints[2].Fix)
	}
}

// TestCrossDistanceFromFixAtUsesDeferredWaypoints verifies that when a
// deferred route exists, the synthetic waypoint is inserted into that route
// rather than the current nav.Waypoints slice.
func TestCrossDistanceFromFixAtUsesDeferredWaypoints(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	origNavLen := len(f.nav.Waypoints)
	deferred := append([]av.Waypoint(nil), f.nav.Waypoints[1:]...)
	f.nav.DeferredNavHeading = &DeferredNavHeading{Waypoints: deferred}

	fixLoc := deferred[0].Location
	priorLoc := f.nav.FlightState.Position
	approachHeading := math.TrueToMagnetic(
		math.Heading2LL(fixLoc, priorLoc, f.nav.FlightState.NmPerLongitude),
		f.nav.FlightState.MagneticVariation,
	)
	dir, err := math.ParseCardinalOrdinalDirection(math.ShortCompass(approachHeading))
	if err != nil {
		t.Fatalf("failed to parse direction: %v", err)
	}

	ar := av.MakeAtAltitudeRestriction(5000)
	intent := f.nav.CrossDistanceFromFixAt("DETGY", 5, dir, &ar, nil)
	if _, ok := intent.(av.UnableIntent); ok {
		t.Fatalf("unexpected unable: %v", intent)
	}

	if len(f.nav.Waypoints) != origNavLen {
		t.Fatalf("expected nav.Waypoints length %d, got %d", origNavLen, len(f.nav.Waypoints))
	}
	if len(f.nav.DeferredNavHeading.Waypoints) != len(deferred)+1 {
		t.Fatalf("expected deferred waypoints length %d, got %d",
			len(deferred)+1, len(f.nav.DeferredNavHeading.Waypoints))
	}
	if f.nav.DeferredNavHeading.Waypoints[0].Fix[0] != '_' {
		t.Fatalf("expected synthetic waypoint inserted at deferred index 0, got %q",
			f.nav.DeferredNavHeading.Waypoints[0].Fix)
	}
	if !f.nav.DeferredNavHeading.Waypoints[0].OnSTAR() {
		t.Fatalf("expected deferred synthetic waypoint to preserve STAR membership")
	}
	if f.nav.DeferredNavHeading.Waypoints[1].Fix != "DETGY" {
		t.Fatalf("expected DETGY after synthetic waypoint, got %q",
			f.nav.DeferredNavHeading.Waypoints[1].Fix)
	}
}

func TestCrossDistanceFromFixAtIgnoresEmptyDeferredRoute(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	origNavLen := len(f.nav.Waypoints)
	f.nav.DeferredNavHeading = &DeferredNavHeading{}

	fixLoc := f.nav.Waypoints[1].Location
	priorLoc := f.nav.Waypoints[0].Location
	approachHeading := math.TrueToMagnetic(
		math.Heading2LL(fixLoc, priorLoc, f.nav.FlightState.NmPerLongitude),
		f.nav.FlightState.MagneticVariation,
	)
	dir, err := math.ParseCardinalOrdinalDirection(math.ShortCompass(approachHeading))
	if err != nil {
		t.Fatalf("failed to parse direction: %v", err)
	}

	ar := av.MakeAtAltitudeRestriction(5000)
	intent := f.nav.CrossDistanceFromFixAt("DETGY", 5, dir, &ar, nil)
	if _, ok := intent.(av.UnableIntent); ok {
		t.Fatalf("unexpected unable: %v", intent)
	}

	if len(f.nav.Waypoints) != origNavLen+1 {
		t.Fatalf("expected nav.Waypoints length %d, got %d", origNavLen+1, len(f.nav.Waypoints))
	}
	if len(f.nav.DeferredNavHeading.Waypoints) != 0 {
		t.Fatalf("expected empty deferred route to remain empty, got %d waypoints", len(f.nav.DeferredNavHeading.Waypoints))
	}
}

// TestCrossDistanceFromFixAtReplacement verifies that altitude and speed/mach
// synthetic waypoints are independent and replace correctly.
func TestCrossDistanceFromFixAtReplacement(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	// Find the correct approach direction.
	fixLoc := f.nav.Waypoints[1].Location
	priorLoc := f.nav.Waypoints[0].Location
	approachHeading := math.TrueToMagnetic(
		math.Heading2LL(fixLoc, priorLoc, f.nav.FlightState.NmPerLongitude),
		f.nav.FlightState.MagneticVariation,
	)
	dir, _ := math.ParseCardinalOrdinalDirection(math.ShortCompass(approachHeading))

	// 1. Initial command: both altitude and speed.
	ar1 := av.MakeAtAltitudeRestriction(8000)
	sr1 := av.MakeAtSpeedRestriction(230)
	f.nav.CrossDistanceFromFixAt("DETGY", 5, dir, &ar1, &sr1)

	// Original waypoints: SAJUL, DETGY, HAUPT, KJFK (4)
	// After adding one combined synthetic waypoint: SAJUL, _DETGY/5W, DETGY, HAUPT, KJFK (5)
	if len(f.nav.Waypoints) != 5 {
		t.Fatalf("expected 5 waypoints, got %d", len(f.nav.Waypoints))
	}
	expected5 := fmt.Sprintf("_DETGY/5%s", dir.ShortString())
	if f.nav.Waypoints[1].Fix != expected5 {
		t.Errorf("expected %s at index 1, got %q", expected5, f.nav.Waypoints[1].Fix)
	}
	if f.nav.Waypoints[1].AltitudeRestriction() == nil || f.nav.Waypoints[1].SpeedRestriction() == nil {
		t.Fatalf("expected combined synthetic waypoint to carry both altitude and speed restrictions")
	}

	// 2. Update only altitude.
	ar2 := av.MakeAtAltitudeRestriction(9000)
	f.nav.CrossDistanceFromFixAt("DETGY", 7, dir, &ar2, nil)

	// The 5-mile waypoint should retain only speed; altitude moves to the 7-mile waypoint.
	if len(f.nav.Waypoints) != 6 {
		t.Fatalf("expected 6 waypoints after altitude update, got %d", len(f.nav.Waypoints))
	}
	expected7 := fmt.Sprintf("_DETGY/7%s", dir.ShortString())
	if f.nav.Waypoints[1].Fix != expected7 {
		t.Errorf("expected %s at index 1, got %q", expected7, f.nav.Waypoints[1].Fix)
	}
	if f.nav.Waypoints[1].AltitudeRestriction() == nil || f.nav.Waypoints[1].SpeedRestriction() != nil {
		t.Fatalf("expected 7-mile waypoint to carry only altitude restriction")
	}
	if f.nav.Waypoints[2].Fix != expected5 {
		t.Errorf("expected %s at index 2, got %q", expected5, f.nav.Waypoints[2].Fix)
	}
	if f.nav.Waypoints[2].AltitudeRestriction() != nil || f.nav.Waypoints[2].SpeedRestriction() == nil {
		t.Fatalf("expected 5-mile waypoint to carry only speed restriction")
	}

	// 3. Update only speed.
	sr2 := av.MakeAtSpeedRestriction(210)
	f.nav.CrossDistanceFromFixAt("DETGY", 6, dir, nil, &sr2)

	// The old 5-mile speed waypoint should be gone, replaced by a 6-mile speed waypoint.
	if len(f.nav.Waypoints) != 6 {
		t.Fatalf("expected 6 waypoints after speed update, got %d", len(f.nav.Waypoints))
	}
	expected6 := fmt.Sprintf("_DETGY/6%s", dir.ShortString())
	if f.nav.Waypoints[1].Fix != expected7 {
		t.Errorf("expected %s at index 1, got %q", expected7, f.nav.Waypoints[1].Fix)
	}
	if f.nav.Waypoints[2].Fix != expected6 {
		t.Errorf("expected %s at index 2, got %q", expected6, f.nav.Waypoints[2].Fix)
	}
	if f.nav.Waypoints[1].AltitudeRestriction() == nil || f.nav.Waypoints[1].SpeedRestriction() != nil {
		t.Fatalf("expected 7-mile waypoint to carry only altitude restriction")
	}
	if f.nav.Waypoints[2].AltitudeRestriction() != nil || f.nav.Waypoints[2].SpeedRestriction() == nil {
		t.Fatalf("expected 6-mile waypoint to carry only speed restriction")
	}
}

// TestCrossDistanceFromFixAtRejectsOrthogonalDirection verifies that
// orthogonal directions are rejected instead of being mapped onto the route leg.
func TestCrossDistanceFromFixAtRejectsOrthogonalDirection(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	fixLoc := f.nav.Waypoints[1].Location
	priorLoc := f.nav.Waypoints[0].Location
	approachHeading := math.TrueToMagnetic(
		math.Heading2LL(fixLoc, priorLoc, f.nav.FlightState.NmPerLongitude),
		f.nav.FlightState.MagneticVariation,
	)
	orthogonalDir := math.ShortCompass(math.OffsetHeading(approachHeading, 90))
	dir, err := math.ParseCardinalOrdinalDirection(orthogonalDir)
	if err != nil {
		t.Fatalf("failed to parse direction: %v", err)
	}

	ar := av.MakeAtAltitudeRestriction(5000)
	intent := f.nav.CrossDistanceFromFixAt("DETGY", 5, dir, &ar, nil)
	AssertUnable(t, intent)
}

// TestCrossDistanceFromFixAtUsesUnderscoreNamedPriorWaypoint verifies that a
// real prior waypoint with an underscore-prefixed name is still used as the leg start.
func TestCrossDistanceFromFixAtUsesUnderscoreNamedPriorWaypoint(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "SAJUL/a10000/star DETGY/a7000/star HAUPT/a6000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  10000,
		InitialSpeed:     250,
	})

	f.nav.Waypoints[0].Fix = "_THRESHOLD_HELPER"

	fixLoc := f.nav.Waypoints[1].Location
	priorLoc := f.nav.Waypoints[0].Location
	approachHeading := math.TrueToMagnetic(
		math.Heading2LL(fixLoc, priorLoc, f.nav.FlightState.NmPerLongitude),
		f.nav.FlightState.MagneticVariation,
	)
	dir, err := math.ParseCardinalOrdinalDirection(math.ShortCompass(approachHeading))
	if err != nil {
		t.Fatalf("failed to parse direction: %v", err)
	}

	ar := av.MakeAtAltitudeRestriction(5000)
	intent := f.nav.CrossDistanceFromFixAt("DETGY", 5, dir, &ar, nil)
	if _, ok := intent.(av.UnableIntent); ok {
		t.Fatalf("unexpected unable: %v", intent)
	}

	if len(f.nav.Waypoints) < 3 {
		t.Fatalf("expected inserted synthetic waypoint")
	}
	if !f.nav.Waypoints[1].SyntheticCrossing() {
		t.Fatalf("expected synthetic waypoint at index 1, got %q", f.nav.Waypoints[1].Fix)
	}
	if got := math.NMDistance2LL(f.nav.Waypoints[1].Location, f.nav.Waypoints[2].Location); got < 4.5 || got > 5.5 {
		t.Fatalf("expected synthetic waypoint about 5nm before DETGY on the actual leg, got %.2f", got)
	}
}

// TestAtFixClearedInvalidFix verifies that AtFixCleared with a fix not on
// the approach returns an UnableIntent.
func TestAtFixClearedInvalidFix(t *testing.T) {
	f := NewArrivalFlight(t, ArrivalConfig{
		Waypoints:        "HAUPT/a6000/star LEFER/a4000/star ROSLY/a3000/star",
		DepartureAirport: "KMCO",
		ArrivalAirport:   "KJFK",
		AircraftType:     "A320",
		InitialAltitude:  7000,
		InitialSpeed:     210,
	})

	f.ExpectApproach("I22L")
	intent := f.nav.AtFixCleared("BOGUS", "I22L", false)
	AssertUnable(t, intent)
}
