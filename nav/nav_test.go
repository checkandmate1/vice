// nav/nav_test.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package nav

import (
	"encoding/json"
	"os"
	"slices"
	"testing"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/rand"
	"github.com/mmp/vice/util"
	"github.com/mmp/vice/wx"
)

func TestMain(m *testing.M) {
	av.InitDB()
	os.Exit(m.Run())
}

// FlightTest orchestrates a simulated flight with events and assertions.
type FlightTest struct {
	t        *testing.T
	nav      *Nav
	fp       av.FlightPlan
	callsign string
	simTime  Time
	maxTicks int
	weather  func(float32) wx.Sample
	events   []flightEvent
	passed   []string // fixes passed so far
	tick     int
}

// flightEvent is a scheduled action or assertion.
type flightEvent struct {
	name    string
	trigger eventTrigger
	action  func(f *FlightTest)
	fired   bool
}

type eventTrigger struct {
	atFix     string // fire when this fix is passed
	beforeFix string // fire every tick; fail if fix passed without it holding
	atTick    int    // fire at this tick number
}

// ArrivalConfig configures a test arrival flight.
type ArrivalConfig struct {
	Waypoints        string  // waypoint string ("PARCH CAMRN/a13000 ...")
	DepartureAirport string  // ICAO code
	ArrivalAirport   string  // ICAO code
	AircraftType     string  // e.g. "A320", "B738"
	InitialAltitude  float32 // starting altitude in feet
	InitialSpeed     float32 // starting IAS in knots
	AssignedAltitude float32 // 0 = none
	ClearedAltitude  float32 // 0 = none
	OnSTAR           bool    // set OnSTAR flag on all waypoints
}

// NewArrivalFlight creates a FlightTest from an ArrivalConfig.
func NewArrivalFlight(t *testing.T, cfg ArrivalConfig) *FlightTest {
	t.Helper()

	wps := parseRoute(t, cfg.Waypoints)
	if cfg.OnSTAR {
		for i := range wps {
			wps[i].SetOnSTAR(true)
		}
	}

	perf, ok := av.DB.AircraftPerformance[cfg.AircraftType]
	if !ok {
		if alias, aok := av.DB.AircraftTypeAliases[cfg.AircraftType]; aok {
			perf, ok = av.DB.AircraftPerformance[alias]
		}
		if !ok {
			t.Fatalf("unknown aircraft type %q", cfg.AircraftType)
		}
	}

	arrAirport, ok := av.DB.Airports[cfg.ArrivalAirport]
	if !ok {
		t.Fatalf("unknown arrival airport %q", cfg.ArrivalAirport)
	}
	depAirport, ok := av.DB.Airports[cfg.DepartureAirport]
	if !ok {
		t.Fatalf("unknown departure airport %q", cfg.DepartureAirport)
	}

	nmPerLongitude := math.NMPerLongitudeAt(arrAirport.Location)
	magneticVariation, err := av.DB.MagneticGrid.Lookup(arrAirport.Location)
	if err != nil {
		t.Fatalf("magnetic grid lookup failed: %v", err)
	}

	// Deterministic randomness
	rng := &rand.Rand{PCG32: rand.NewPCG32()}
	rng.Seed(42)

	fp := av.FlightPlan{
		Rules:            av.FlightRulesIFR,
		AircraftType:     cfg.AircraftType,
		DepartureAirport: cfg.DepartureAirport,
		ArrivalAirport:   cfg.ArrivalAirport,
		Altitude:         int(cfg.InitialAltitude),
	}

	// Build waypoints with arrival airport at the end
	navWps := make([]av.Waypoint, len(wps)+1)
	copy(navWps, wps)
	navWps[len(wps)] = av.Waypoint{
		Fix:      cfg.ArrivalAirport,
		Location: arrAirport.Location,
	}

	// Compute initial heading from first to second waypoint
	var initialHeading math.MagneticHeading
	if len(navWps) > 1 {
		trueHdg := math.Heading2LL(navWps[0].Location, navWps[1].Location, nmPerLongitude)
		initialHeading = math.TrueToMagnetic(trueHdg, magneticVariation)
	}

	n := &Nav{
		Perf:           perf,
		FinalAltitude:  cfg.InitialAltitude,
		FixAssignments: make(map[string]NavFixAssignment),
		Rand:           rng,
		Waypoints:      navWps,
		FlightState: FlightState{
			MagneticVariation:         magneticVariation,
			NmPerLongitude:            nmPerLongitude,
			Position:                  navWps[0].Location,
			Heading:                   initialHeading,
			Altitude:                  cfg.InitialAltitude,
			IAS:                       cfg.InitialSpeed,
			GS:                        cfg.InitialSpeed,
			DepartureAirportLocation:  depAirport.Location,
			DepartureAirportElevation: float32(depAirport.Elevation),
			ArrivalAirportLocation:    arrAirport.Location,
			ArrivalAirportElevation:   float32(arrAirport.Elevation),
			ArrivalAirport: av.Waypoint{
				Fix:      cfg.ArrivalAirport,
				Location: arrAirport.Location,
			},
		},
	}

	if cfg.AssignedAltitude > 0 {
		alt := cfg.AssignedAltitude
		n.Altitude.Assigned = &alt
	}
	if cfg.ClearedAltitude > 0 {
		alt := cfg.ClearedAltitude
		n.Altitude.Cleared = &alt
	}

	return &FlightTest{
		t:        t,
		nav:      n,
		fp:       fp,
		callsign: "TEST001",
		simTime:  NewTime(time.Date(2025, 1, 1, 12, 0, 0, 0, time.UTC)),
		maxTicks: 7200,
		weather:  func(alt float32) wx.Sample { return wx.MakeStandardSampleForAltitude(alt) },
	}
}

// AtFix fires action when the named fix is passed.
func (f *FlightTest) AtFix(fix string, action func(*FlightTest)) *FlightTest {
	f.events = append(f.events, flightEvent{
		name:    "atFix:" + fix,
		trigger: eventTrigger{atFix: fix},
		action:  action,
	})
	return f
}

// BeforeFix fires action every tick until the fix is passed.
// The check must hold continuously; the fix must eventually be passed.
func (f *FlightTest) BeforeFix(fix string, check func(*FlightTest)) *FlightTest {
	f.events = append(f.events, flightEvent{
		name:    "beforeFix:" + fix,
		trigger: eventTrigger{beforeFix: fix},
		action:  check,
	})
	return f
}

// AfterTicks fires action at the given tick count.
func (f *FlightTest) AfterTicks(n int, action func(*FlightTest)) *FlightTest {
	f.events = append(f.events, flightEvent{
		name:    "afterTicks",
		trigger: eventTrigger{atTick: n},
		action:  action,
	})
	return f
}

// Run executes the simulation loop.
func (f *FlightTest) Run() {
	f.t.Helper()

	for f.tick = 0; f.tick < f.maxTicks; f.tick++ {
		wxs := f.weather(f.nav.FlightState.Altitude)
		passedWp := f.nav.UpdateWithWeather(f.callsign, wxs, &f.fp, f.simTime, nil)

		if passedWp != nil {
			f.passed = append(f.passed, passedWp.Fix)
			f.fireAtFixEvents(passedWp.Fix)
			f.resolveBeforeFixEvents(passedWp.Fix)
		}

		f.fireBeforeFixEvents()
		f.fireAtTickEvents()

		// Advance simulation time by 1 second per tick
		f.simTime = f.simTime.Add(time.Second)

		// Stop when all events have fired
		if f.allEventsFired() {
			return
		}

		// Stop if aircraft has descended to near airport elevation
		if f.nav.FlightState.Altitude <= f.nav.FlightState.ArrivalAirportElevation+100 {
			break
		}
	}

	// Verify all fix-triggered events actually fired
	for _, e := range f.events {
		if !e.fired {
			if e.trigger.atFix != "" {
				f.t.Errorf("fix %q was never reached (passed: %v)", e.trigger.atFix, f.passed)
			}
			if e.trigger.beforeFix != "" {
				f.t.Errorf("fix %q was never reached, beforeFix check never ran (passed: %v)", e.trigger.beforeFix, f.passed)
			}
		}
	}
}

func (f *FlightTest) fireAtFixEvents(fix string) {
	for i := range f.events {
		if f.events[i].trigger.atFix == fix && !f.events[i].fired {
			f.events[i].action(f)
			f.events[i].fired = true
		}
	}
}

func (f *FlightTest) resolveBeforeFixEvents(fix string) {
	for i := range f.events {
		if f.events[i].trigger.beforeFix == fix {
			f.events[i].fired = true
		}
	}
}

func (f *FlightTest) fireBeforeFixEvents() {
	for i := range f.events {
		e := &f.events[i]
		if e.trigger.beforeFix != "" && !e.fired {
			e.action(f)
		}
	}
}

func (f *FlightTest) fireAtTickEvents() {
	for i := range f.events {
		e := &f.events[i]
		if e.trigger.atTick > 0 && f.tick == e.trigger.atTick && !e.fired {
			e.action(f)
			e.fired = true
		}
	}
}

func (f *FlightTest) allEventsFired() bool {
	return !slices.ContainsFunc(f.events, func(e flightEvent) bool { return !e.fired })
}

// Assertion helpers

func (f *FlightTest) AssertAltitudeBelow(alt float32) {
	f.t.Helper()
	if f.nav.FlightState.Altitude > alt {
		f.t.Errorf("tick %d: altitude %.0f exceeds %.0f", f.tick, f.nav.FlightState.Altitude, alt)
	}
}

func (f *FlightTest) AssertAltitudeNear(alt, tolerance float32) {
	f.t.Helper()
	if f.nav.FlightState.Altitude < alt-tolerance || f.nav.FlightState.Altitude > alt+tolerance {
		f.t.Errorf("tick %d: altitude %.0f not within %.0f of %.0f",
			f.tick, f.nav.FlightState.Altitude, tolerance, alt)
	}
}

func (f *FlightTest) AssertAltitudeAbove(alt float32) {
	f.t.Helper()
	if f.nav.FlightState.Altitude < alt {
		f.t.Errorf("tick %d: altitude %.0f below %.0f", f.tick, f.nav.FlightState.Altitude, alt)
	}
}

func (f *FlightTest) AssertDescending() {
	f.t.Helper()
	if f.nav.FlightState.AltitudeRate >= 0 {
		f.t.Errorf("tick %d: expected descending but altitude rate is %.0f", f.tick, f.nav.FlightState.AltitudeRate)
	}
}

func (f *FlightTest) AssertSpeedBelow(spd float32) {
	f.t.Helper()
	if f.nav.FlightState.IAS > spd {
		f.t.Errorf("tick %d: IAS %.0f exceeds %.0f", f.tick, f.nav.FlightState.IAS, spd)
	}
}

func (f *FlightTest) AssertSpeedAbove(spd float32) {
	f.t.Helper()
	if f.nav.FlightState.IAS < spd {
		f.t.Errorf("tick %d: IAS %.0f below %.0f", f.tick, f.nav.FlightState.IAS, spd)
	}
}

// Command helpers

func (f *FlightTest) AssignAltitude(alt float32) {
	f.t.Helper()
	f.nav.AssignAltitude(alt, false)
}

func (f *FlightTest) AssignSpeed(spd float32) {
	f.t.Helper()
	sr := av.MakeAtSpeedRestriction(spd)
	f.nav.AssignSpeed(&sr, false)
}

func (f *FlightTest) ExpectApproach(id string) {
	f.t.Helper()
	airport := f.makeAirport()
	f.nav.ExpectApproach(airport, id, nil, "", nil)
}

// makeAirport constructs an *av.Airport from the FAAAirport in av.DB,
// resolving approach waypoint locations and adding runway threshold
// waypoints — mirroring the essential parts of Airport.PostDeserialize.
func (f *FlightTest) makeAirport() *av.Airport {
	icao := f.fp.ArrivalAirport
	faa, ok := av.DB.Airports[icao]
	if !ok {
		f.t.Fatalf("unknown airport %q", icao)
	}

	nmPerLong := f.nav.FlightState.NmPerLongitude
	magVar := f.nav.FlightState.MagneticVariation
	e := &util.ErrorLogger{}

	approaches := make(map[string]*av.Approach)
	for name, appr := range faa.Approaches {
		a := appr
		// Deep-copy waypoint slices so we don't mutate the database.
		a.Waypoints = make([]av.WaypointArray, len(appr.Waypoints))
		for i, route := range appr.Waypoints {
			a.Waypoints[i] = util.DuplicateSlice(route)
		}

		// Resolve fix names to lat/lon coordinates.
		for i := range a.Waypoints {
			a.Waypoints[i] = a.Waypoints[i].InitializeLocations(dbLocator{}, nmPerLong, magVar, true, e)
			for j := range a.Waypoints[i] {
				a.Waypoints[i][j].SetOnApproach(true)
			}
		}

		// Add runway threshold waypoint to each route.
		if rwy, ok := av.LookupRunway(icao, a.Runway); ok {
			a.Threshold = rwy.Threshold
			for i := range a.Waypoints {
				alt := rwy.Elevation + rwy.ThresholdCrossingHeight
				threshold := math.Offset2LL(rwy.Threshold,
					math.MagneticToTrue(rwy.Heading, magVar),
					rwy.DisplacedThresholdDistance, nmPerLong)
				thresholdWP := av.Waypoint{
					Fix:      "_" + a.Runway + "_THRESHOLD",
					Location: threshold,
				}
				thresholdWP.SetLand(true)
				thresholdWP.SetFlyOver(true)
				thresholdWP.SetAltitudeRestriction(av.MakeAtAltitudeRestriction(float32(alt)))
				a.Waypoints[i] = append(a.Waypoints[i], thresholdWP)
			}
		}

		if opp, ok := av.LookupOppositeRunway(icao, a.Runway); ok {
			a.OppositeThreshold = opp.Threshold
		}

		approaches[name] = &a
	}
	return &av.Airport{
		Location:   faa.Location,
		Approaches: approaches,
	}
}

func (f *FlightTest) ClearedApproach(id string) {
	f.t.Helper()
	f.nav.ClearedApproach(f.fp.ArrivalAirport, id, false, f.simTime)
}

func (f *FlightTest) AssignHeading(hdg int, turn av.TurnDirection) {
	f.t.Helper()
	f.nav.AssignHeading(math.MagneticHeading(hdg), turn, f.simTime)
}

func (f *FlightTest) DirectFix(fix string) {
	f.t.Helper()
	f.nav.DirectFix(fix, av.TurnClosest, f.simTime)
}

// dbLocator implements av.Locator using the static aviation database.
type dbLocator struct{}

func (dbLocator) Locate(fix string) (math.Point2LL, bool) {
	return av.DB.LookupWaypoint(fix)
}

func (dbLocator) Similar(fix string) []string { return nil }

// parseRoute parses a waypoint string using the scenario JSON format
// and resolves fix locations from av.DB. Requires av.DB to be initialized.
func parseRoute(t *testing.T, s string) av.WaypointArray {
	t.Helper()
	var wps av.WaypointArray
	// WaypointArray.UnmarshalJSON expects a JSON-encoded string
	jsonStr, err := json.Marshal(s)
	if err != nil {
		t.Fatalf("failed to marshal waypoint string: %v", err)
	}
	if err := json.Unmarshal(jsonStr, &wps); err != nil {
		t.Fatalf("failed to parse waypoints %q: %v", s, err)
	}
	if len(wps) == 0 {
		t.Fatalf("no waypoints parsed from %q", s)
	}
	// Resolve fix names to coordinates
	e := &util.ErrorLogger{}
	wps = wps.InitializeLocations(dbLocator{}, 0, 0, false, e)
	if e.HaveErrors() {
		t.Fatalf("failed to resolve waypoint locations: %s", e.String())
	}
	return wps
}
