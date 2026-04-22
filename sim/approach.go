// sim/approach.go
// Copyright(c) 2022-2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"slices"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/nav"
	"github.com/mmp/vice/util"
)

// AirportInSightInquiry handles the bare "AP" command. The controller asks
// "do you have the field in sight?" without specifying a direction; the
// pilot's response depends on weather, ceiling, and distance to the airport —
// no o'clock/bearing validation is performed.
func (s *Sim) AirportInSightInquiry(tcw TCW, callsign av.ADSBCallsign) (av.CommandIntent, error) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	return s.dispatchControlledAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) av.CommandIntent {
			if ac.FieldInSight || ac.RequestedVisual {
				return av.LookForFieldFound
			}
			return s.handleAirportAdvisory(ac, 0, 0)
		})
}

// AirportAdvisory handles the AP/{oclock}/{miles} command. The controller tells the
// pilot where to look for the airport: "airport, {oclock} o'clock, {miles} miles".
// The pilot responds with "field in sight", "looking", or an IMC indication.
func (s *Sim) AirportAdvisory(tcw TCW, callsign av.ADSBCallsign, oclock, miles int) (av.CommandIntent, error) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	return s.dispatchControlledAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) av.CommandIntent {
			// If the pilot already has the field in sight, just confirm.
			if ac.FieldInSight || ac.RequestedVisual {
				return av.LookForFieldFound
			}

			return s.handleAirportAdvisory(ac, oclock, miles)
		})
}

// handleAirportAdvisory determines the pilot's response to an AP command.
// It reuses checkVisualEligibility for METAR/VMC/ceiling/distance/bearing
// checks, then layers on AP-specific logic (o'clock validation, probability,
// looking delay).
func (s *Sim) handleAirportAdvisory(ac *Aircraft, oclock int, miles int) av.CommandIntent {
	// Use the shared eligibility check for VMC, ceiling, range, and bearing.
	elig := s.checkVisualEligibility(ac)
	if !elig.FieldInSight {
		if elig.Reason == visualEligibilityIMC {
			return av.LookForFieldLookingIMC
		}
		ac.FieldLookingUntil = s.pilotLookDeadline()
		if elig.Reason == visualEligibilityObscured {
			return av.LookForFieldLookingObscured
		}
		return av.LookForFieldLooking
	}

	// Validate the controller's o'clock direction against the actual bearing.
	// oclock == 0 means the controller didn't give a direction (bare "AP"
	// inquiry), so skip this check.
	if oclock > 0 {
		oclockHeading := float32((oclock % 12) * 30)
		reportedBearing := math.MagneticHeading(math.NormalizeHeading(float32(ac.Heading()) + oclockHeading))
		bearingError := math.HeadingDifference(reportedBearing, elig.BearingToAirport)
		if bearingError > 30 {
			ac.FieldLookingUntil = s.pilotLookDeadline()
			return av.LookForFieldLooking
		}
	}

	if s.Rand.Float32() < pilotSeeProb(elig.MaxRange, elig.Distance) {
		ac.FieldInSight = true
		return av.LookForFieldFound
	}

	// "Looking" — schedule possible delayed field-in-sight call.
	ac.FieldLookingUntil = s.pilotLookDeadline()
	return av.LookForFieldLooking
}

// pilotLookDeadline returns a new "pilot is looking" timer deadline.
func (s *Sim) pilotLookDeadline() Time {
	d := pilotLookDurationMin + s.Rand.Intn(pilotLookDurationMax-pilotLookDurationMin)
	return s.State.SimTime.Add(time.Duration(d) * time.Second)
}

func (s *Sim) ExpectApproach(tcw TCW, callsign av.ADSBCallsign, approach, lahsoRunway string) (av.CommandIntent, error) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	var ap *av.Airport
	if ac, ok := s.Aircraft[callsign]; ok {
		ap = s.State.Airports[ac.FlightPlan.ArrivalAirport]
		if ap == nil {
			return nil, av.ErrUnknownAirport
		}
	}

	return s.dispatchControlledAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) av.CommandIntent {
			return ac.ExpectApproach(approach, ap, lahsoRunway, s.lg)
		})
}

func (s *Sim) ExpectVisualApproach(tcw TCW, callsign av.ADSBCallsign, runway string, lahsoRunway string) (av.CommandIntent, error) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	return s.dispatchControlledAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) av.CommandIntent {
			airport := ac.FlightPlan.ArrivalAirport
			if _, ok := av.LookupRunway(airport, runway); !ok {
				return av.MakeUnableIntent("unable, we don't know that runway")
			}
			if !s.activeArrivalRunway(airport, runway) {
				return av.MakeUnableIntent("unable, that runway isn't active")
			}
			if lahsoRunway != "" {
				if _, ok := av.LookupRunway(airport, lahsoRunway); !ok {
					return av.MakeUnableIntent("unable, we don't know that hold-short runway")
				}
			}
			return av.ApproachIntent{
				Type:         av.ApproachExpect,
				ApproachName: "Visual Approach Runway " + runway,
				LAHSORunway:  lahsoRunway,
			}
		})
}

func (s *Sim) activeArrivalRunway(airport string, runway string) bool {
	hasArrivalRunways := false
	runwayBase := av.RunwayID(runway).Base()
	for _, ar := range s.State.ArrivalRunways {
		if ar.Airport != airport {
			continue
		}
		hasArrivalRunways = true
		if ar.Runway.Base() == runwayBase {
			return true
		}
	}
	return !hasArrivalRunways
}

func (s *Sim) ClearedApproach(tcw TCW, callsign av.ADSBCallsign, approach string, straightIn bool) (av.CommandIntent, error) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	return s.dispatchControlledAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) av.CommandIntent {
			if straightIn {
				return ac.ClearedStraightInApproach(approach, s.State.SimTime, s.lg)
			} else {
				return ac.ClearedApproach(approach, s.State.SimTime, s.lg)
			}
		})
}

// ClearedVisualApproach clears the aircraft for a visual approach to the
// specified runway. Command format is "CVA<runway>" (e.g. "CVA13L"). The
// aircraft uses the assigned approach, or the best known approach for the
// runway, as a visual route template when one is available. For charted visual
// approaches (e.g., Belmont Visual), use the C command with the approach ID instead.
func (s *Sim) ClearedVisualApproach(tcw TCW, callsign av.ADSBCallsign, runway string, lahsoRunway string) (av.CommandIntent, error) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	intent, err := s.dispatchAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) error {
			if !s.TCWCanCommandAircraft(tcw, ac) {
				return av.ErrOtherControllerHasTrack
			}
			return nil
		},
		func(tcw TCW, ac *Aircraft) av.CommandIntent {
			if runway == "" {
				return av.MakeUnableIntent("unable, which runway?")
			}
			if _, ok := av.LookupRunway(ac.FlightPlan.ArrivalAirport, runway); !ok {
				return av.MakeUnableIntent("unable, we don't know runway " + runway)
			}

			// Pilot must have the field or approach-cleared preceding
			// traffic in sight before accepting a visual approach clearance.
			traffic := s.recentApproachTrafficInSightForRunway(ac, runway)
			if !ac.FieldInSight && !ac.RequestedVisual && traffic == nil {
				return av.MakeUnableIntent("unable, we don't have the field in sight")
			}

			return s.clearForVisualApproach(ac, runway, lahsoRunway, traffic)
		})

	// Keep parity with dispatchControlledAircraftCommand behavior:
	// any successfully-dispatched command (even an "unable" intent)
	// suppresses stale initial check-ins.
	if err == nil {
		s.cancelPendingInitialContact(callsign)
	}
	return intent, err
}

// clearForVisualApproach dispatches the nav-layer clearance for a visual
// approach. When traffic is non-nil, the nav layer handles tight in-trail
// sequencing along the leader's route and its geometric fallbacks.
func (s *Sim) clearForVisualApproach(ac *Aircraft, runway, lahsoRunway string, traffic *Aircraft) av.CommandIntent {
	var follow *nav.FollowTraffic
	if traffic != nil {
		follow = &nav.FollowTraffic{Position: traffic.Position(), Route: traffic.Nav.Waypoints}
	}
	refs := s.visualReferenceApproaches(ac, runway, traffic)
	return ac.ClearedVisualApproach(runway, follow, refs, lahsoRunway, s.State.SimTime)
}

// visualReferenceApproaches picks one or more approaches whose geometry can serve as the reference
// for a visual to runway. When traffic is non-nil, its assigned approach is preferred so the
// follower shares the leader's geometry.
func (s *Sim) visualReferenceApproaches(ac *Aircraft, runway string, traffic *Aircraft) []*av.Approach {
	runwayBase := av.RunwayID(runway).Base()

	if traffic != nil {
		if ap := traffic.Nav.Approach.Assigned; ap != nil &&
			av.RunwayID(ap.Runway).Base() == runwayBase &&
			len(ap.Waypoints) > 0 {
			return []*av.Approach{ap}
		}
	}

	if ap := ac.Nav.Approach.Assigned; ap != nil &&
		av.RunwayID(ap.Runway).Base() == runwayBase &&
		len(ap.Waypoints) > 0 {
		return []*av.Approach{ap}
	}

	airport := s.State.Airports[ac.FlightPlan.ArrivalAirport]

	prioritizedTypes := []av.ApproachType{av.VisualApproach, av.ILSApproach, av.LocalizerApproach,
		av.VORApproach, av.RNAVApproach, av.ChartedVisualApproach}
	for _, ty := range prioritizedTypes {
		matches := slices.Collect(util.FilterSeq(util.SortedMapValues(airport.Approaches),
			func(ap *av.Approach) bool {
				return ap.Type == ty && av.RunwayID(ap.Runway).Base() == runwayBase
			}))
		if len(matches) > 0 {
			return matches
		}
	}
	// This should not happen...
	return nil
}

func (s *Sim) InterceptApproach(tcw TCW, callsign av.ADSBCallsign) (av.CommandIntent, error) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	return s.dispatchControlledAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) av.CommandIntent {
			return ac.InterceptApproach(s.lg)
		})
}

func (s *Sim) CancelApproachClearance(tcw TCW, callsign av.ADSBCallsign) (av.CommandIntent, error) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	return s.dispatchControlledAircraftCommand(tcw, callsign,
		func(tcw TCW, ac *Aircraft) av.CommandIntent {
			return ac.CancelApproachClearance()
		})
}

func (s *Sim) hasRecentApproachTrafficInSight(ac *Aircraft) bool {
	return s.recentApproachTrafficInSight(ac) != nil
}

func (s *Sim) recentApproachTrafficInSightForRunway(ac *Aircraft, runway string) *Aircraft {
	traffic := s.recentApproachTrafficInSight(ac)
	if traffic == nil || traffic.Nav.Approach.Assigned == nil || traffic.Nav.Approach.Assigned.Runway != runway {
		return nil
	}
	return traffic
}

func (s *Sim) recentApproachTrafficInSight(ac *Aircraft) *Aircraft {
	if !ac.TrafficInSight || s.State.SimTime.Sub(ac.TrafficInSightTime) > 30*time.Second {
		return nil
	}

	traffic, ok := s.Aircraft[ac.TrafficInSightCallsign]
	if ok && traffic.Nav.Approach.Cleared {
		return traffic
	}
	return nil
}

// checkDelayedPilotSighting advances a "pilot is looking" timer. If the timer
// is unset or expired, it clears the timer (via clear) and returns. Otherwise
// it rolls a per-tick chance to fire; on fire it calls validate (nil counts
// as true) and, if valid, invokes fire.
func (s *Sim) checkDelayedPilotSighting(ac *Aircraft, until *Time, clear func(), validate func() bool, fire func()) {
	if until.IsZero() {
		return
	}
	if s.State.SimTime.After(*until) {
		clear()
		return
	}
	if ac.ControllerFrequency == "" {
		return
	}
	if s.Rand.Float32() >= pilotSpontaneousReport {
		return
	}
	if validate != nil && !validate() {
		return
	}
	fire()
}

// checkDelayedFieldInSight checks if an aircraft that said "looking" (in
// response to an AP command) should now report "field in sight".
func (s *Sim) checkDelayedFieldInSight(ac *Aircraft) {
	if ac.FieldInSight {
		return
	}
	s.checkDelayedPilotSighting(ac, &ac.FieldLookingUntil,
		func() { ac.FieldLookingUntil = Time{} },
		func() bool { return s.checkVisualEligibility(ac).FieldInSight },
		func() {
			ac.FieldInSight = true
			ac.FieldLookingUntil = Time{}
			s.enqueuePilotTransmission(ac.ADSBCallsign, TCP(ac.ControllerFrequency), PendingTransmissionFieldInSight)
		})
}

// checkDelayedTrafficInSight checks if an aircraft that said "looking" in
// response to a traffic call should now report traffic in sight.
func (s *Sim) checkDelayedTrafficInSight(ac *Aircraft) {
	clear := func() {
		ac.TrafficLookingCallsign = ""
		ac.TrafficLookingUntil = Time{}
	}
	s.checkDelayedPilotSighting(ac, &ac.TrafficLookingUntil, clear, nil,
		func() {
			ac.TrafficInSight = true
			ac.TrafficInSightCallsign = ac.TrafficLookingCallsign
			ac.TrafficInSightTime = s.State.SimTime
			clear()
			s.enqueuePilotTransmission(ac.ADSBCallsign, TCP(ac.ControllerFrequency), PendingTransmissionTrafficInSight)
		})
}

// canRequestVisualApproach reports whether an aircraft is eligible to
// spontaneously request the visual approach. The aircraft must be an
// arrival on frequency, assigned a non-visual approach that hasn't been
// cleared yet, and must not have already made the request.
func (ac *Aircraft) canRequestVisualApproach() bool {
	if ac.IsDeparture() || ac.FieldInSight || ac.RequestedVisual || ac.ControllerFrequency == "" {
		return false
	}
	if ac.Nav.Approach.Assigned == nil || ac.Nav.Approach.Cleared {
		return false
	}
	appr := ac.Nav.Approach.Assigned
	// Already on a visual — nothing to request.
	return appr.Type != av.ChartedVisualApproach
}

type visualEligibilityReason int

const (
	visualEligibilityOK visualEligibilityReason = iota
	visualEligibilityIMC
	visualEligibilityOutOfRange
	visualEligibilityObscured
	visualEligibilityBadBearing
)

// VisualEligibility describes whether an aircraft can see the field
// and request a visual approach.
type VisualEligibility struct {
	FieldInSight     bool   // true if VMC, within range, and airport visible
	Runway           string // runway for the visual approach (when FieldInSight)
	Reason           visualEligibilityReason
	Distance         float32
	MaxRange         float32
	BearingToAirport math.MagneticHeading
}

// checkVisualEligibility determines whether the aircraft can see the field.
// A visual approach does not require a charted visual procedure; VMC and
// field in sight are sufficient.
// Shared by AirportAdvisory and checkSpontaneousVisualRequest.
func (s *Sim) checkVisualEligibility(ac *Aircraft) VisualEligibility {
	arrivalAirport := ac.FlightPlan.ArrivalAirport
	ap := s.State.Airports[arrivalAirport]

	// Must be VMC at the arrival airport.
	metar, ok := s.State.METAR[arrivalAirport]
	if !ok || !metar.IsVMC() {
		return VisualEligibility{Reason: visualEligibilityIMC}
	}

	// Aircraft above the ceiling is in the clouds → can't see the field.
	if ceiling, err := metar.Ceiling(); err == nil {
		if faa, ok := av.DB.Airports[arrivalAirport]; ok {
			if ac.Altitude() > float32(faa.Elevation+ceiling) {
				return VisualEligibility{Reason: visualEligibilityIMC}
			}
		}
	}

	// Must be within effective visual range (METAR visibility + altitude bonus).
	var altAGL float32
	if faa, ok := av.DB.Airports[arrivalAirport]; ok {
		altAGL = ac.Altitude() - float32(faa.Elevation)
		if altAGL < 0 {
			altAGL = 0
		}
	}
	maxRange := metar.EffectiveVisualRange(altAGL)
	dist := math.NMDistance2LLFast(ac.Position(), ap.Location, ac.NmPerLongitude())
	if dist > maxRange {
		reason := util.Select(metar.HasObscuration(), visualEligibilityObscured, visualEligibilityOutOfRange)
		return VisualEligibility{
			Distance: dist,
			MaxRange: maxRange,
			Reason:   reason,
		}
	}

	// The airport must be within the pilot's forward visibility arc.
	bearingToAirport := math.TrueToMagnetic(math.Heading2LL(ac.Position(), ap.Location, ac.NmPerLongitude()), ac.MagneticVariation())
	if math.HeadingDifference(ac.Heading(), bearingToAirport) > visualMaxBearingOff {
		return VisualEligibility{
			Distance:         dist,
			MaxRange:         maxRange,
			BearingToAirport: bearingToAirport,
			Reason:           visualEligibilityBadBearing,
		}
	}

	var runway string
	if ac.Nav.Approach.Assigned != nil {
		runway = ac.Nav.Approach.Assigned.Runway
	}

	return VisualEligibility{
		FieldInSight:     true,
		Runway:           runway,
		Reason:           visualEligibilityOK,
		Distance:         dist,
		MaxRange:         maxRange,
		BearingToAirport: bearingToAirport,
	}
}

// Tunables for the pilot-vision model.
const (
	visualMaxBearingOff    = float32(120)  // degrees off nose; forward visibility arc
	visualFieldProb        = float32(0.10) // fraction of pilots who spontaneously report field in sight
	visualRequestProb      = float32(0.10) // fraction of field-in-sight pilots who also request the visual
	visualDelayMin         = 2             // seconds; min delay after field in sight
	visualDelayMax         = 8             // seconds; max delay after field in sight
	pilotLookDurationMin   = 10            // seconds; min "looking" window
	pilotLookDurationMax   = 20            // seconds; max "looking" window (exclusive upper bound)
	pilotSpontaneousReport = float32(0.10) // per-tick chance to spontaneously report while "looking"
)

// pilotSeeProb returns a probability (0..1) that a pilot can visually
// identify a target at distNM, given the effective visual range (NM). Zero
// beyond effective range; tapers linearly below it.
func pilotSeeProb(effectiveRangeNM, distNM float32) float32 {
	if effectiveRangeNM <= 0 || distNM > effectiveRangeNM {
		return 0
	}
	return max(0.4, 0.95*(1-0.5*distNM/effectiveRangeNM))
}

// checkSpontaneousVisualRequest checks if an arrival aircraft should
// spontaneously report "field in sight" or request the visual approach.
// WantsVisual/WantsVisualRequest are decided at aircraft creation time.
// A short random delay (2-8s) after the field first comes into sight
// simulates identification and reaction time.
func (s *Sim) checkSpontaneousVisualRequest(ac *Aircraft) {
	if !ac.canRequestVisualApproach() {
		return
	}

	// Don't report before the pilot has checked in.
	if s.hasPendingCheckIn(ac.ADSBCallsign) {
		return
	}

	if !ac.WantsVisual {
		return
	}

	elig := s.checkVisualEligibility(ac)
	if !elig.FieldInSight {
		ac.VisualRequestTime = Time{} // reset if field lost
		return
	}

	// Set a random delay the first time the field comes into sight.
	if ac.VisualRequestTime.IsZero() {
		delay := visualDelayMin + s.Rand.Intn(visualDelayMax-visualDelayMin+1)
		ac.VisualRequestTime = s.State.SimTime.Add(time.Duration(delay) * time.Second)
		return
	}
	if s.State.SimTime.Before(ac.VisualRequestTime) {
		return
	}

	ac.FieldInSight = true

	// Visual request is a superset of field-in-sight; prefer it if both won.
	if ac.WantsVisualRequest {
		ac.RequestedVisual = true
		s.enqueuePilotTransmission(ac.ADSBCallsign, TCP(ac.ControllerFrequency), PendingTransmissionRequestVisual)
	} else {
		s.enqueuePilotTransmission(ac.ADSBCallsign, TCP(ac.ControllerFrequency), PendingTransmissionFieldInSight)
	}
}
