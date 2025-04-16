// pkg/server/dispatcher.go
// Copyright(c) 2022-2024 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package server

import (
	"strconv"
	"strings"

	av "github.com/mmp/vice/pkg/aviation"
	"github.com/mmp/vice/pkg/math"
	"github.com/mmp/vice/pkg/sim"
	"github.com/mmp/vice/pkg/util"
)

type Dispatcher struct {
	sm *SimManager
}

func (sd *Dispatcher) GetWorldUpdate(token string, update *sim.WorldUpdate) error {
	// Most of the methods in this file are called from the RPC dispatcher,
	// which spawns up goroutines as needed to handle requests, so if we
	// want to catch and report panics, all of the methods need to start
	// like this...
	defer sd.sm.lg.CatchAndReportCrash()

	return sd.sm.GetWorldUpdate(token, update)
}

func (sd *Dispatcher) SignOff(token string, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	return sd.sm.SignOff(token)
}

type ChangeControlPositionArgs struct {
	ControllerToken string
	Callsign        string
	KeepTracks      bool
}

func (sd *Dispatcher) ChangeControlPosition(cs *ChangeControlPositionArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(cs.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.ChangeControlPosition(ctrl.tcp, cs.Callsign, cs.KeepTracks)
	}
}

func (sd *Dispatcher) TakeOrReturnLaunchControl(token string, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(token); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.TakeOrReturnLaunchControl(ctrl.tcp)
	}
}

type SetSimRateArgs struct {
	ControllerToken string
	Rate            float32
}

func (sd *Dispatcher) SetSimRate(r *SetSimRateArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(r.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.SetSimRate(ctrl.tcp, r.Rate)
	}
}

type SetLaunchConfigArgs struct {
	ControllerToken string
	Config          sim.LaunchConfig
}

func (sd *Dispatcher) SetLaunchConfig(lc *SetLaunchConfigArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(lc.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.SetLaunchConfig(ctrl.tcp, lc.Config)
	}
}

func (sd *Dispatcher) TogglePause(token string, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(token); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.TogglePause(ctrl.tcp)
	}
}

type SetScratchpadArgs struct {
	ControllerToken string
	Callsign        string
	Scratchpad      string
}

func (sd *Dispatcher) SetScratchpad(a *SetScratchpadArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(a.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.SetScratchpad(ctrl.tcp, a.Callsign, a.Scratchpad)
	}
}

func (sd *Dispatcher) SetSecondaryScratchpad(a *SetScratchpadArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(a.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.SetSecondaryScratchpad(ctrl.tcp, a.Callsign, a.Scratchpad)
	}
}

func (sd *Dispatcher) AutoAssociateFP(it *InitiateTrackArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(it.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.AutoAssociateFP(ctrl.tcp, it.Callsign, it.Plan)
	}
}

type SetGlobalLeaderLineArgs struct {
	ControllerToken string
	Callsign        string
	Direction       *math.CardinalOrdinalDirection
}

func (sd *Dispatcher) SetGlobalLeaderLine(a *SetGlobalLeaderLineArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(a.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.SetGlobalLeaderLine(ctrl.tcp, a.Callsign, a.Direction)
	}
}

type InitiateTrackArgs struct {
	AircraftSpecifier
	Plan *av.STARSFlightPlan
}

func (sd *Dispatcher) InitiateTrack(it *InitiateTrackArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(it.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.InitiateTrack(ctrl.tcp, it.Callsign, it.Plan)
	}
}

type CreateUnsupportedTrackArgs struct {
	ControllerToken  string
	Callsign         string
	UnsupportedTrack *sim.UnsupportedTrack
}

func (sd *Dispatcher) CreateUnsupportedTrack(it *CreateUnsupportedTrackArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(it.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.CreateUnsupportedTrack(ctrl.tcp, it.Callsign, it.UnsupportedTrack)
	}
}

type UploadPlanArgs struct {
	ControllerToken string
	Type            int
	Plan            *av.STARSFlightPlan
}

func (sd *Dispatcher) UploadFlightPlan(it *UploadPlanArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(it.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.UploadFlightPlan(ctrl.tcp, it.Type, it.Plan)
	}
}

type AircraftSpecifier struct {
	ControllerToken string
	Callsign        string
}

type DropTrackArgs AircraftSpecifier

func (sd *Dispatcher) DropTrack(dt *DropTrackArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(dt.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.DropTrack(ctrl.tcp, dt.Callsign)
	}
}

type HandoffArgs struct {
	ControllerToken string
	Callsign        string
	Controller      string
}

func (sd *Dispatcher) HandoffTrack(h *HandoffArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(h.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.HandoffTrack(ctrl.tcp, h.Callsign, h.Controller)
	}
}

func (sd *Dispatcher) RedirectHandoff(h *HandoffArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(h.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.RedirectHandoff(ctrl.tcp, h.Callsign, h.Controller)
	}
}

func (sd *Dispatcher) AcceptRedirectedHandoff(po *AcceptHandoffArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(po.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.AcceptRedirectedHandoff(ctrl.tcp, po.Callsign)
	}
}

type AcceptHandoffArgs AircraftSpecifier

func (sd *Dispatcher) AcceptHandoff(ah *AcceptHandoffArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(ah.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.AcceptHandoff(ctrl.tcp, ah.Callsign)
	}
}

type CancelHandoffArgs AircraftSpecifier

func (sd *Dispatcher) CancelHandoff(ch *CancelHandoffArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(ch.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.CancelHandoff(ctrl.tcp, ch.Callsign)
	}
}

type PointOutArgs struct {
	ControllerToken string
	Callsign        string
	Controller      string
}

type ForceQLArgs struct {
	ControllerToken string
	Callsign        string
	Controller      string
}

func (sd *Dispatcher) ForceQL(ql *ForceQLArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(ql.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.ForceQL(ctrl.tcp, ql.Callsign, ql.Controller)
	}
}

type GlobalMessageArgs struct {
	ControllerToken string
	Message         string
}

func (sd *Dispatcher) GlobalMessage(gm *GlobalMessageArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(gm.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.GlobalMessage(ctrl.tcp, gm.Message)
	}
}

func (sd *Dispatcher) PointOut(po *PointOutArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(po.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.PointOut(ctrl.tcp, po.Callsign, po.Controller)
	}
}

func (sd *Dispatcher) AcknowledgePointOut(po *PointOutArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(po.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.AcknowledgePointOut(ctrl.tcp, po.Callsign)
	}
}

func (sd *Dispatcher) RecallPointOut(po *PointOutArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(po.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.RecallPointOut(ctrl.tcp, po.Callsign)
	}
}

func (sd *Dispatcher) RejectPointOut(po *PointOutArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(po.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.RejectPointOut(ctrl.tcp, po.Callsign)
	}
}

type ToggleSPCArgs struct {
	ControllerToken string
	Callsign        string
	SPC             string
}

func (sd *Dispatcher) ToggleSPCOverride(ts *ToggleSPCArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(ts.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.ToggleSPCOverride(ctrl.tcp, ts.Callsign, ts.SPC)
	}
}

type HeldDepartureArgs AircraftSpecifier

func (sd *Dispatcher) ReleaseDeparture(hd *HeldDepartureArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(hd.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.ReleaseDeparture(ctrl.tcp, hd.Callsign)
	}
}

type AssignAltitudeArgs struct {
	ControllerToken string
	Callsign        string
	Altitude        int
}

func (sd *Dispatcher) SetTemporaryAltitude(alt *AssignAltitudeArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(alt.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.SetTemporaryAltitude(ctrl.tcp, alt.Callsign, alt.Altitude)
	}
}

func (sd *Dispatcher) SetPilotReportedAltitude(alt *AssignAltitudeArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(alt.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.SetPilotReportedAltitude(ctrl.tcp, alt.Callsign, alt.Altitude)
	}
}

func (sd *Dispatcher) ToggleDisplayModeCAltitude(ac *AircraftSpecifier, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(ac.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.ToggleDisplayModeCAltitude(ctrl.tcp, ac.Callsign)
	}
}

type DeleteAircraftArgs AircraftSpecifier

func (sd *Dispatcher) DeleteAllAircraft(da *DeleteAircraftArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if ctrl, s, ok := sd.sm.LookupController(da.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	} else {
		return s.DeleteAllAircraft(ctrl.tcp)
	}
}

type AircraftCommandsArgs struct {
	ControllerToken string
	Callsign        string
	Commands        string
}

// If an RPC call returns an error, then the result argument is not returned(!?).
// So we don't use the error type for syntax errors...
type AircraftCommandsResult struct {
	ErrorMessage   string
	RemainingInput string
}

func (sd *Dispatcher) RunAircraftCommands(cmds *AircraftCommandsArgs, result *AircraftCommandsResult) error {
	defer sd.sm.lg.CatchAndReportCrash()

	ctrl, s, ok := sd.sm.LookupController(cmds.ControllerToken)
	if !ok {
		return ErrNoSimForControllerToken
	}
	callsign := cmds.Callsign

	commands := strings.Fields(cmds.Commands)

	for i, command := range commands {
		rewriteError := func(err error) {
			result.RemainingInput = strings.Join(commands[i:], " ")
			if err != nil {
				result.ErrorMessage = err.Error()
			}
		}

		switch command[0] {
		case 'A', 'C':
			if command == "CAC" {
				// Cancel approach clearance
				if err := s.CancelApproachClearance(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "CVS" {
				if err := s.ClimbViaSID(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if len(command) > 4 && command[:3] == "CSI" && !util.IsAllNumbers(command[3:]) {
				// Cleared straight in approach.
				if err := s.ClearedApproach(ctrl.tcp, callsign, command[3:], true); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command[0] == 'C' && len(command) > 2 && !util.IsAllNumbers(command[1:]) {
				if components := strings.Split(command, "/"); len(components) > 1 {
					// Cross fix [at altitude] [at speed]
					fix := components[0][1:]
					var ar *av.AltitudeRestriction
					speed := 0

					for _, cmd := range components[1:] {
						if len(cmd) == 0 {
							rewriteError(ErrInvalidCommandSyntax)
							return nil
						}

						var err error
						if cmd[0] == 'A' && len(cmd) > 1 {
							if ar, err = av.ParseAltitudeRestriction(cmd[1:]); err != nil {
								rewriteError(err)
								return nil
							}
							// User input here is 100s of feet, while AltitudeRestriction is feet...
							ar.Range[0] *= 100
							ar.Range[1] *= 100
						} else if cmd[0] == 'S' {
							if speed, err = strconv.Atoi(cmd[1:]); err != nil {
								rewriteError(err)
								return nil
							}
						} else {
							rewriteError(ErrInvalidCommandSyntax)
							return nil
						}
					}

					if err := s.CrossFixAt(ctrl.tcp, callsign, fix, ar, speed); err != nil {
						rewriteError(err)
						return nil
					}
				} else if err := s.ClearedApproach(ctrl.tcp, callsign, command[1:], false); err != nil {
					rewriteError(err)
					return nil
				}
			} else {
				if command[0] == 'A' {
					components := strings.Split(command, "/")
					if len(components) != 2 || len(components[1]) == 0 || components[1][0] != 'C' {
						rewriteError(ErrInvalidCommandSyntax)
						return nil
					}

					fix := strings.ToUpper(components[0][1:])
					approach := components[1][1:]
					if err := s.AtFixCleared(ctrl.tcp, callsign, fix, approach); err != nil {
						rewriteError(err)
						return nil
					} else {
						continue
					}
				}

				// Otherwise look for an altitude
				if alt, err := strconv.Atoi(command[1:]); err != nil {
					rewriteError(err)
					return nil
				} else if err := s.AssignAltitude(ctrl.tcp, callsign, 100*alt, false); err != nil {
					rewriteError(err)
					return nil
				}
			}

		case 'D':
			if command == "DVS" {
				if err := s.DescendViaSTAR(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if components := strings.Split(command, "/"); len(components) > 1 && len(components[1]) > 1 {
				fix := components[0][1:]

				switch components[1][0] {
				case 'D':
					// Depart <fix1> direct <fix2>
					if err := s.DepartFixDirect(ctrl.tcp, callsign, fix, components[1][1:]); err != nil {
						rewriteError(err)
						return nil
					}
				case 'H':
					// Depart <fix> at heading <hdg>
					if hdg, err := strconv.Atoi(components[1][1:]); err != nil {
						rewriteError(err)
						return nil
					} else if err := s.DepartFixHeading(ctrl.tcp, callsign, fix, hdg); err != nil {
						rewriteError(err)
						return nil
					}

				default:
					rewriteError(ErrInvalidCommandSyntax)
					return nil
				}
			} else if len(command) > 1 && command[1] >= '0' && command[1] <= '9' {
				// Looks like an altitude.
				if alt, err := strconv.Atoi(command[1:]); err != nil {
					rewriteError(err)
					return nil
				} else if err := s.AssignAltitude(ctrl.tcp, callsign, 100*alt, false); err != nil {
					rewriteError(err)
					return nil
				}
			} else if _, ok := s.State.Locate(string(command[1:])); ok {
				if err := s.DirectFix(ctrl.tcp, callsign, command[1:]); err != nil {
					rewriteError(err)
					return nil
				}
			} else {
				rewriteError(ErrInvalidCommandSyntax)
				return nil
			}

		case 'E':
			if command == "ED" {
				if err := s.ExpediteDescent(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "EC" {
				if err := s.ExpediteClimb(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if len(command) > 1 {
				// Expect approach.
				if err := s.ExpectApproach(ctrl.tcp, callsign, command[1:]); err != nil {
					rewriteError(err)
					return nil
				}
			} else {
				rewriteError(ErrInvalidCommandSyntax)
				return nil
			}

		case 'F':
			if command == "FC" {
				if err := s.HandoffControl(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else {
				rewriteError(ErrInvalidCommandSyntax)
				return nil
			}

		case 'H':
			if len(command) == 1 {
				if err := s.AssignHeading(&sim.HeadingArgs{
					TCP:      ctrl.tcp,
					Callsign: callsign,
					Present:  true,
				}); err != nil {
					rewriteError(err)
					return nil
				}
			} else if hdg, err := strconv.Atoi(command[1:]); err != nil {
				rewriteError(err)
				return nil
			} else if err := s.AssignHeading(&sim.HeadingArgs{
				TCP:      ctrl.tcp,
				Callsign: callsign,
				Heading:  hdg,
				Turn:     av.TurnClosest,
			}); err != nil {
				rewriteError(err)
				return nil
			}

		case 'I':
			if len(command) == 1 {
				if err := s.InterceptLocalizer(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "ID" {
				if err := s.Ident(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else {
				rewriteError(ErrInvalidCommandSyntax)
				return nil
			}

		case 'L':
			if l := len(command); l > 2 && command[l-1] == 'D' {
				// turn left x degrees
				if deg, err := strconv.Atoi(command[1 : l-1]); err != nil {
					rewriteError(err)
					return nil
				} else if err := s.AssignHeading(&sim.HeadingArgs{
					TCP:         ctrl.tcp,
					Callsign:    callsign,
					LeftDegrees: deg,
				}); err != nil {
					rewriteError(err)
					return nil
				}
			} else {
				// turn left heading...
				if hdg, err := strconv.Atoi(command[1:]); err != nil {
					rewriteError(err)
					return nil
				} else if err := s.AssignHeading(&sim.HeadingArgs{
					TCP:      ctrl.tcp,
					Callsign: callsign,
					Heading:  hdg,
					Turn:     av.TurnLeft,
				}); err != nil {
					rewriteError(err)
					return nil
				}
			}

		case 'R':
			if l := len(command); l > 2 && command[l-1] == 'D' {
				// turn right x degrees
				if deg, err := strconv.Atoi(command[1 : l-1]); err != nil {
					rewriteError(err)
					return nil
				} else if err := s.AssignHeading(&sim.HeadingArgs{
					TCP:          ctrl.tcp,
					Callsign:     callsign,
					RightDegrees: deg,
				}); err != nil {
					rewriteError(err)
					return nil
				}
			} else {
				// turn right heading...
				if hdg, err := strconv.Atoi(command[1:]); err != nil {
					rewriteError(err)
					return nil
				} else if err := s.AssignHeading(&sim.HeadingArgs{
					TCP:      ctrl.tcp,
					Callsign: callsign,
					Heading:  hdg,
					Turn:     av.TurnRight,
				}); err != nil {
					rewriteError(err)
					return nil
				}
			}

		case 'S':
			if len(command) == 1 {
				// Cancel speed restrictions
				if err := s.AssignSpeed(ctrl.tcp, callsign, 0, false); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "SMIN" {
				if err := s.MaintainSlowestPractical(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "SMAX" {
				if err := s.MaintainMaximumForward(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "SS" {
				if err := s.SaySpeed(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "SQS" {
				if err := s.ChangeTransponderMode(ctrl.tcp, callsign, av.Standby); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "SQA" {
				if err := s.ChangeTransponderMode(ctrl.tcp, callsign, av.Altitude); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "SQON" {
				if err := s.ChangeTransponderMode(ctrl.tcp, callsign, av.On); err != nil {
					rewriteError(err)
					return nil
				}
			} else if len(command) == 6 && command[:2] == "SQ" {
				if sq, err := av.ParseSquawk(command[2:]); err != nil {
					rewriteError(err)
					return nil
				} else if err := s.ChangeSquawk(ctrl.tcp, callsign, sq); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "SH" {
				if err := s.SayHeading(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if command == "SA" {
				if err := s.SayAltitude(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if kts, err := strconv.Atoi(command[1:]); err != nil {
				rewriteError(err)
				return nil
			} else if err := s.AssignSpeed(ctrl.tcp, callsign, kts, false); err != nil {
				rewriteError(err)
				return nil
			}

		case 'T':
			if command == "TO" {
				if err := s.ContactTower(ctrl.tcp, callsign); err != nil {
					rewriteError(err)
					return nil
				}
			} else if n := len(command); n > 2 {
				if deg, err := strconv.Atoi(command[1 : n-1]); err == nil {
					if command[n-1] == 'L' {
						// turn x degrees left
						if err := s.AssignHeading(&sim.HeadingArgs{
							TCP:         ctrl.tcp,
							Callsign:    callsign,
							LeftDegrees: deg,
						}); err != nil {
							rewriteError(err)
							return nil
						} else {
							continue
						}
					} else if command[n-1] == 'R' {
						// turn x degrees right
						if err := s.AssignHeading(&sim.HeadingArgs{
							TCP:          ctrl.tcp,
							Callsign:     callsign,
							RightDegrees: deg,
						}); err != nil {
							rewriteError(err)
							return nil
						} else {
							continue
						}
					}
				}

				switch command[:2] {
				case "TS":
					if kts, err := strconv.Atoi(command[2:]); err != nil {
						rewriteError(err)
						return nil
					} else if err := s.AssignSpeed(ctrl.tcp, callsign, kts, true); err != nil {
						rewriteError(err)
						return nil
					}

				case "TA", "TC", "TD":
					if alt, err := strconv.Atoi(command[2:]); err != nil {
						rewriteError(err)
						return nil
					} else if err := s.AssignAltitude(ctrl.tcp, callsign, 100*alt, true); err != nil {
						rewriteError(err)
						return nil
					}

				default:
					rewriteError(ErrInvalidCommandSyntax)
					return nil
				}
			} else {
				rewriteError(ErrInvalidCommandSyntax)
				return nil
			}

		case 'X':
			s.DeleteAircraft(ctrl.tcp, callsign)

		default:
			rewriteError(ErrInvalidCommandSyntax)
			return nil
		}
	}

	return nil
}

type LaunchAircraftArgs struct {
	ControllerToken string
	Aircraft        av.Aircraft
	DepartureRunway string
}

func (sd *Dispatcher) LaunchAircraft(ls *LaunchAircraftArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	_, s, ok := sd.sm.LookupController(ls.ControllerToken)
	if !ok {
		return ErrNoSimForControllerToken
	}
	s.LaunchAircraft(ls.Aircraft, ls.DepartureRunway)
	return nil
}

type CreateDepartureArgs struct {
	ControllerToken string
	Airport         string
	Runway          string
	Category        string
	Rules           av.FlightRules
}

func (sd *Dispatcher) CreateDeparture(da *CreateDepartureArgs, depAc *av.Aircraft) error {
	defer sd.sm.lg.CatchAndReportCrash()

	_, s, ok := sd.sm.LookupController(da.ControllerToken)
	if !ok {
		return ErrNoSimForControllerToken
	}
	var ac *av.Aircraft
	var err error
	if da.Rules == av.IFR {
		ac, err = s.CreateIFRDeparture(da.Airport, da.Runway, da.Category)
	} else {
		ac, err = s.CreateVFRDeparture(da.Airport)
	}

	if ac != nil && err == nil {
		*depAc = *ac
	}
	return err
}

type CreateArrivalArgs struct {
	ControllerToken string
	Group           string
	Airport         string
}

func (sd *Dispatcher) CreateArrival(aa *CreateArrivalArgs, arrAc *av.Aircraft) error {
	defer sd.sm.lg.CatchAndReportCrash()

	_, s, ok := sd.sm.LookupController(aa.ControllerToken)
	if !ok {
		return ErrNoSimForControllerToken
	}
	ac, err := s.CreateArrival(aa.Group, aa.Airport)
	if err == nil {
		*arrAc = *ac
	}
	return err
}

type CreateOverflightArgs struct {
	ControllerToken string
	Group           string
}

func (sd *Dispatcher) CreateOverflight(oa *CreateOverflightArgs, ofAc *av.Aircraft) error {
	defer sd.sm.lg.CatchAndReportCrash()

	_, s, ok := sd.sm.LookupController(oa.ControllerToken)
	if !ok {
		return ErrNoSimForControllerToken
	}
	ac, err := s.CreateOverflight(oa.Group)
	if err == nil {
		*ofAc = *ac
	}
	return err
}

type RestrictionAreaArgs struct {
	ControllerToken string
	Index           int
	RestrictionArea av.RestrictionArea
}

func (sd *Dispatcher) CreateRestrictionArea(ra *RestrictionAreaArgs, idx *int) error {
	defer sd.sm.lg.CatchAndReportCrash()

	_, s, ok := sd.sm.LookupController(ra.ControllerToken)
	if !ok {
		return ErrNoSimForControllerToken
	}
	i, err := s.CreateRestrictionArea(ra.RestrictionArea)
	if err == nil {
		*idx = i
	}
	return err
}

func (sd *Dispatcher) UpdateRestrictionArea(ra *RestrictionAreaArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	_, s, ok := sd.sm.LookupController(ra.ControllerToken)
	if !ok {
		return ErrNoSimForControllerToken
	}
	return s.UpdateRestrictionArea(ra.Index, ra.RestrictionArea)
}

func (sd *Dispatcher) DeleteRestrictionArea(ra *RestrictionAreaArgs, _ *struct{}) error {
	defer sd.sm.lg.CatchAndReportCrash()

	_, s, ok := sd.sm.LookupController(ra.ControllerToken)
	if !ok {
		return ErrNoSimForControllerToken
	}
	return s.DeleteRestrictionArea(ra.Index)
}

type VideoMapsArgs struct {
	ControllerToken string
	Filename        string
}

func (sd *Dispatcher) GetVideoMapLibrary(vm *VideoMapsArgs, vmf *av.VideoMapLibrary) error {
	defer sd.sm.lg.CatchAndReportCrash()

	if _, _, ok := sd.sm.LookupController(vm.ControllerToken); !ok {
		return ErrNoSimForControllerToken
	}
	if v, err := av.LoadVideoMapLibrary(vm.Filename); err == nil {
		*vmf = *v
		return nil
	} else {
		return err
	}
}
