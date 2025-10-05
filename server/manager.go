// server/manager.go
// Copyright(c) 2022-2025 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package server

import (
	crand "crypto/rand"
	"encoding/base64"
	"fmt"
	"log/slog"
	gomath "math"
	"net"
	"net/http"
	"net/http/pprof"
	"os"
	"runtime"
	"strconv"
	"strings"
	"sync/atomic"
	"text/template"
	"time"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/log"
	"github.com/mmp/vice/rand"
	"github.com/mmp/vice/sim"
	"github.com/mmp/vice/util"
	"github.com/mmp/vice/wx"

	"github.com/brunoga/deep"
	"github.com/gorilla/websocket"
	"github.com/shirou/gopsutil/cpu"
	"github.com/vmihailenco/msgpack/v5"
)

///////////////////////////////////////////////////////////////////////////
// SimManager

type ttsUsageStats struct {
	Calls    int
	Words    int
	LastUsed time.Time
}

type SimManager struct {
	scenarioGroups   map[string]map[string]*scenarioGroup
	configs          map[string]map[string]*Configuration
	simSessions      map[string]*simSession
	sessionsByToken  map[string]*controllerSession
	mu               util.LoggingMutex
	mapManifests     map[string]*sim.VideoMapManifest
	startTime        time.Time
	httpPort         int
	websocketTXBytes atomic.Int64
	tts              sim.TTSProvider
	ttsUsageByIP     map[string]*ttsUsageStats
	local            bool
	wxProvider       wx.Provider
	lg               *log.Logger
}

type controllerSession struct {
	session *simSession
	tcp     string
}

type Configuration struct {
	ScenarioConfigs  map[string]*SimScenarioConfiguration
	ControlPositions map[string]*av.Controller
	DefaultScenario  string
	Facility         string
	ARTCC            string
	Area             string
}

type connectionState struct {
	token               string
	lastUpdateCall      time.Time
	warnedNoUpdateCalls bool
	speechWs            *websocket.Conn
	disableTextToSpeech bool
}

type SimScenarioConfiguration struct {
	SelectedController  string
	SelectedSplit       string
	SplitConfigurations av.SplitConfigurationSet
	PrimaryAirport      string
	MagneticVariation   float32

	LaunchConfig sim.LaunchConfig

	DepartureRunways []sim.DepartureRunway
	ArrivalRunways   []sim.ArrivalRunway
}

func (s *SimScenarioConfiguration) AllAirports() []string {
	allAirports := make(map[string]bool)
	for _, runway := range s.DepartureRunways {
		allAirports[runway.Airport] = true
	}
	for _, runway := range s.ArrivalRunways {
		allAirports[runway.Airport] = true
	}
	return util.SortedMapKeys(allAirports)
}

type simSession struct {
	name          string
	scenarioGroup string
	scenario      string
	sim           *sim.Sim
	password      string
	mu            util.LoggingMutex

	connectionsByTCP map[string]*connectionState
}

type NewSimConfiguration struct {
	NewSimName   string
	GroupName    string
	ScenarioName string

	ScenarioConfig *SimScenarioConfiguration

	TFRs []av.TFR

	TRACONName      string
	RequirePassword bool
	Password        string

	EnforceUniqueCallsignSuffix bool

	AllowInstructorRPO  bool
	Instructor          bool
	DisableTextToSpeech bool

	StartTime time.Time
}

type SimConnectionConfiguration struct {
	RemoteSim           string
	Position            string
	Password            string
	Instructor          bool
	DisableTextToSpeech bool
}

func MakeNewSimConfiguration() NewSimConfiguration {
	return NewSimConfiguration{NewSimName: rand.Make().AdjectiveNoun()}
}

type RemoteSim struct {
	GroupName          string
	ScenarioName       string
	PrimaryController  string
	RequirePassword    bool
	AvailablePositions map[string]av.Controller
	CoveredPositions   map[string]av.Controller
}

func (ss *simSession) AddHumanController(tcp, token string, disableTextToSpeech bool, lg *log.Logger) {
	ss.mu.Lock(lg)
	defer ss.mu.Unlock(lg)

	ss.connectionsByTCP[tcp] = &connectionState{
		token:               token,
		lastUpdateCall:      time.Now(),
		disableTextToSpeech: disableTextToSpeech,
	}
}

func (ss *simSession) CullIdleControllers(sm *SimManager) {
	ss.mu.Lock(sm.lg)

	// Sign off controllers we haven't heard from in 15 seconds so that
	// someone else can take their place. We only make this check for
	// multi-controller sims; we don't want to do this for local sims so
	// that we don't kick people off e.g. when their computer sleeps.
	var tokensToSignOff []string
	for tcp, ctrl := range ss.connectionsByTCP {
		if time.Since(ctrl.lastUpdateCall) > 5*time.Second {
			if !ctrl.warnedNoUpdateCalls {
				ctrl.warnedNoUpdateCalls = true
				sm.lg.Warnf("%s: no messages for 5 seconds", tcp)
				ss.sim.PostEvent(sim.Event{
					Type:        sim.StatusMessageEvent,
					WrittenText: tcp + " has not been heard from for 5 seconds. Connection lost?",
				})
			}

			if time.Since(ctrl.lastUpdateCall) > 15*time.Second {
				sm.lg.Warnf("%s: signing off idle controller", tcp)
				// Collect tokens to sign off after releasing the lock
				tokensToSignOff = append(tokensToSignOff, ctrl.token)
			}
		}
	}
	ss.mu.Unlock(sm.lg)

	// Sign off controllers without holding as.mu to avoid deadlock
	for _, token := range tokensToSignOff {
		if err := sm.SignOff(token); err != nil {
			sm.lg.Errorf("error signing off idle controller: %v", err)
		}
		// Note: SignOff handles deletion from connectionsByTCP
	}
}

func (ss *simSession) GotUpdateCallForTCP(tcp string, lg *log.Logger) {
	ss.mu.Lock(lg)
	defer ss.mu.Unlock(lg)

	if ctrl, ok := ss.connectionsByTCP[tcp]; !ok {
		lg.Errorf("%s: unknown TCP for sim", tcp)
	} else {
		ctrl.lastUpdateCall = time.Now()
		if ctrl.warnedNoUpdateCalls {
			ctrl.warnedNoUpdateCalls = false
			lg.Warnf("%s: connection re-established", tcp)
			ss.sim.PostEvent(sim.Event{
				Type:        sim.StatusMessageEvent,
				WrittenText: tcp + " is back online.",
			})
		}
	}
}

func (ss *simSession) HandleSpeechWSConnection(tcp string, w http.ResponseWriter, r *http.Request, lg *log.Logger) {
	ss.mu.Lock(lg)
	defer ss.mu.Unlock(lg)

	if ctrl, ok := ss.connectionsByTCP[tcp]; !ok {
		lg.Errorf("%s: unknown TCP", tcp)
	} else {
		if ctrl.speechWs != nil {
			ctrl.speechWs.Close()
		}

		var err error
		upgrader := websocket.Upgrader{EnableCompression: false}
		ctrl.speechWs, err = upgrader.Upgrade(w, r, nil)
		if err != nil {
			lg.Errorf("Unable to upgrade speech websocket: %v", err)
		}
	}
}

func (ss *simSession) SendSpeechMP3s(lg *log.Logger) int64 {
	ss.mu.Lock(lg)
	defer ss.mu.Unlock(lg)

	var nb int
	for tcp, ctrl := range ss.connectionsByTCP {
		if ctrl.speechWs == nil {
			continue
		}

		for _, ps := range ss.sim.GetControllerSpeech(tcp) {
			nb += len(ps.MP3)

			w, err := ctrl.speechWs.NextWriter(websocket.BinaryMessage)
			if err != nil {
				lg.Errorf("speechWs: %v", err)
				continue
			}

			if err := msgpack.NewEncoder(w).Encode(ps); err != nil {
				lg.Errorf("speechWs encode: %v", err)
				continue
			}

			if err := w.Close(); err != nil {
				lg.Errorf("speechWs close: %v", err)
				continue
			}
		}
	}

	return int64(nb)
}

func (ss *simSession) SignOff(tcp string, lg *log.Logger) {
	ss.mu.Lock(lg)
	defer ss.mu.Unlock(lg)

	delete(ss.connectionsByTCP, tcp)
}

///////////////////////////////////////////////////////////////////////////

func NewSimManager(scenarioGroups map[string]map[string]*scenarioGroup,
	simConfigurations map[string]map[string]*Configuration, manifests map[string]*sim.VideoMapManifest,
	serverAddress string, isLocal bool, lg *log.Logger) *SimManager {
	sm := &SimManager{
		scenarioGroups:  scenarioGroups,
		configs:         simConfigurations,
		simSessions:     make(map[string]*simSession),
		sessionsByToken: make(map[string]*controllerSession),
		mapManifests:    manifests,
		startTime:       time.Now(),
		tts:             makeTTSProvider(serverAddress, lg),
		ttsUsageByIP:    make(map[string]*ttsUsageStats),
		local:           isLocal,
		lg:              lg,
	}

	var err error
	sm.wxProvider, err = MakeWXProvider(serverAddress, lg)
	if err != nil {
		lg.Errorf("%v", err)
	}

	sm.launchHTTPServer()

	return sm
}

func makeTTSProvider(serverAddress string, lg *log.Logger) sim.TTSProvider {
	// Try to create a Google TTS provider first
	p, err := NewGoogleTTSProvider(lg)
	if err == nil {
		lg.Info("Using Google TTS provider")
		return p
	}

	// If Google TTS is not available (no credentials), try to connect to the remote server
	lg.Infof("Google TTS unavailable: %v, attempting to use remote TTS provider at %s", err, serverAddress)
	rp, err := NewRemoteTTSProvider(serverAddress, lg)
	if err != nil {
		lg.Errorf("Failed to connect to remote TTS provider: %v", err)
		return nil
	}

	lg.Info("Successfully connected to remote TTS provider")
	return rp
}

type NewSimResult struct {
	SimState        *sim.State
	ControllerToken string
	SpeechWSPort    int
}

const NewSimRPC = "SimManager.NewSim"

func (sm *SimManager) NewSim(config *NewSimConfiguration, result *NewSimResult) error {
	c2 := config
	c2.TFRs = nil
	fmt.Printf("NewSim %#v\n", c2)

	lg := sm.lg.With(slog.String("sim_name", config.NewSimName))
	if nsc := sm.makeSimConfiguration(config, lg); nsc != nil {
		manifest := sm.mapManifests[nsc.FacilityAdaptation.VideoMapFile]
		sim := sim.NewSim(*nsc, manifest, lg)
		session := &simSession{
			name:             config.NewSimName,
			scenarioGroup:    config.GroupName,
			scenario:         config.ScenarioName,
			sim:              sim,
			password:         config.Password,
			connectionsByTCP: make(map[string]*connectionState),
		}
		pos := sim.State.PrimaryController
		return sm.Add(session, result, pos, config.Instructor, true, config.DisableTextToSpeech)
	} else {
		return ErrInvalidSSimConfiguration
	}
}

const ConnectToSimRPC = "SimManager.ConnectToSim"

func (sm *SimManager) ConnectToSim(config *SimConnectionConfiguration, result *NewSimResult) error {
	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	session, ok := sm.simSessions[config.RemoteSim]
	if !ok {
		return ErrNoNamedSim
	}

	if session.password != "" && config.Password != session.password {
		return ErrInvalidPassword
	}

	// If signing on as dedicated instructor/RPO, ignore the additional instructor flag
	signOnConfig := *config
	signOnConfig.Instructor = config.Instructor && config.Position == ""
	token, err := sm.signOn(session, &signOnConfig)
	if err != nil {
		return err
	}

	session.AddHumanController(config.Position, token, config.DisableTextToSpeech, sm.lg)
	sm.sessionsByToken[token] = &controllerSession{
		tcp:     config.Position,
		session: session,
	}

	// Get the state for the controller
	state := session.sim.State.GetStateForController(config.Position)

	*result = NewSimResult{
		SimState:        state,
		ControllerToken: token,
		SpeechWSPort:    util.Select(sm.tts != nil, sm.httpPort, 0),
	}
	result.PruneForClient()

	return nil
}

func (sm *SimManager) makeSimConfiguration(config *NewSimConfiguration, lg *log.Logger) *sim.NewSimConfiguration {
	tracon, ok := sm.scenarioGroups[config.TRACONName]
	if !ok {
		lg.Errorf("%s: unknown TRACON", config.TRACONName)
		return nil
	}
	sg, ok := tracon[config.GroupName]
	if !ok {
		lg.Errorf("%s: unknown scenario group", config.GroupName)
		return nil
	}
	sc, ok := sg.Scenarios[config.ScenarioName]
	if !ok {
		lg.Errorf("%s: unknown scenario", config.ScenarioName)
		return nil
	}

	description := util.Select(sm.local, " "+config.ScenarioName,
		"@"+config.NewSimName+": "+config.ScenarioName)

	nsc := sim.NewSimConfiguration{
		TFRs:                        config.TFRs,
		TRACON:                      config.TRACONName,
		LaunchConfig:                config.ScenarioConfig.LaunchConfig,
		FacilityAdaptation:          deep.MustCopy(sg.FacilityAdaptation),
		IsLocal:                     sm.local,
		EnforceUniqueCallsignSuffix: config.EnforceUniqueCallsignSuffix,
		DepartureRunways:            sc.DepartureRunways,
		ArrivalRunways:              sc.ArrivalRunways,
		VFRReportingPoints:          sg.VFRReportingPoints,
		ReportingPoints:             sg.ReportingPoints,
		Description:                 description,
		MagneticVariation:           sg.MagneticVariation,
		NmPerLongitude:              sg.NmPerLongitude,
		Airports:                    sg.Airports,
		Fixes:                       sg.Fixes,
		PrimaryAirport:              sg.PrimaryAirport,
		Center:                      util.Select(sc.Center.IsZero(), sg.FacilityAdaptation.Center, sc.Center),
		Range:                       util.Select(sc.Range == 0, sg.FacilityAdaptation.Range, sc.Range),
		DefaultMaps:                 sc.DefaultMaps,
		DefaultMapGroup:             sc.DefaultMapGroup,
		InboundFlows:                sg.InboundFlows,
		Airspace:                    sg.Airspace,
		ControllerAirspace:          sc.Airspace,
		ControlPositions:            sg.ControlPositions,
		VirtualControllers:          sc.VirtualControllers,
		SignOnPositions:             make(map[string]*av.Controller),
		TTSProvider:                 sm.tts,
		WXProvider:                  sm.wxProvider,
		StartTime:                   config.StartTime,
	}

	if !sm.local {
		selectedSplit := config.ScenarioConfig.SelectedSplit
		var err error
		nsc.PrimaryController, err = sc.SplitConfigurations.GetPrimaryController(selectedSplit)
		if err != nil {
			lg.Errorf("Unable to get primary controller: %v", err)
		}
		nsc.MultiControllers, err = sc.SplitConfigurations.GetConfiguration(selectedSplit)
		if err != nil {
			lg.Errorf("Unable to get multi controllers: %v", err)
		}
	} else {
		nsc.PrimaryController = sc.SoloController
	}

	add := func(callsign string) {
		if ctrl, ok := sg.ControlPositions[callsign]; !ok {
			lg.Errorf("%s: control position unknown??!", callsign)
		} else {
			nsc.SignOnPositions[callsign] = ctrl
		}
	}
	if !sm.local {
		configs, err := sc.SplitConfigurations.GetConfiguration(config.ScenarioConfig.SelectedSplit)
		if err != nil {
			lg.Errorf("unable to get configurations for split: %v", err)
		}
		for callsign := range configs {
			add(callsign)
		}
	} else {
		add(sc.SoloController)
	}
	if config.AllowInstructorRPO {
		nsc.SignOnPositions["INS"] = &av.Controller{
			Position:   "Instructor",
			Instructor: true,
		}
		nsc.SignOnPositions["RPO"] = &av.Controller{
			Position: "Remote Pilot Operator",
			RPO:      true,
		}
	}

	return &nsc
}

const AddLocalRPC = "SimManager.AddLocal"

func (sm *SimManager) AddLocal(sim *sim.Sim, result *NewSimResult) error {
	session := &simSession{ // no password, etc.
		sim:              sim,
		connectionsByTCP: make(map[string]*connectionState),
	}
	if !sm.local {
		sm.lg.Errorf("Called AddLocal with sm.local == false")
	}
	return sm.Add(session, result, sim.State.PrimaryController, false, false, false)
}

func (sm *SimManager) Add(session *simSession, result *NewSimResult, initialTCP string, instructor bool, prespawn bool, disableTextToSpeech bool) error {
	lg := sm.lg
	if session.name != "" {
		lg = lg.With(slog.String("sim_name", session.name))
	}
	session.sim.Activate(lg, sm.tts, sm.wxProvider)

	sm.mu.Lock(sm.lg)

	// Empty sim name is just a local sim, so no problem with replacing it...
	if _, ok := sm.simSessions[session.name]; ok && session.name != "" {
		sm.mu.Unlock(sm.lg)
		return ErrDuplicateSimName
	}

	sm.lg.Infof("%s: adding sim", session.name)
	sm.simSessions[session.name] = session

	signOnConfig := &SimConnectionConfiguration{
		Position:            initialTCP,
		Instructor:          instructor,
		DisableTextToSpeech: disableTextToSpeech,
	}
	token, err := sm.signOn(session, signOnConfig)
	if err != nil {
		sm.mu.Unlock(sm.lg)
		return err
	}

	session.AddHumanController(initialTCP, token, disableTextToSpeech, sm.lg)
	sm.sessionsByToken[token] = &controllerSession{
		tcp:     initialTCP,
		session: session,
	}

	sm.mu.Unlock(sm.lg)

	// Run prespawn after the primary controller is signed in.
	if prespawn {
		session.sim.Prespawn()
	}

	// Get the state after prespawn (if any) has completed
	state := session.sim.State.GetStateForController(initialTCP)

	go func() {
		defer sm.lg.CatchAndReportCrash()

		for !sm.SimShouldExit(session.sim) {
			// Terminate idle Sims after 4 hours, but not local Sims.
			if !sm.local {
				session.CullIdleControllers(sm)
			}

			session.sim.Update()

			sm.websocketTXBytes.Add(session.SendSpeechMP3s(sm.lg))

			time.Sleep(100 * time.Millisecond)
		}

		sm.lg.Infof("%s: terminating sim after %s idle", session.name, session.sim.IdleTime())

		session.sim.Destroy()

		sm.mu.Lock(sm.lg)
		// Clean up all controllers for this sim
		for token, ctrl := range sm.sessionsByToken {
			if ctrl.session == session {
				delete(sm.sessionsByToken, token)
			}
		}
		delete(sm.simSessions, session.name)
		sm.mu.Unlock(sm.lg)
	}()

	*result = NewSimResult{
		SimState:        state,
		ControllerToken: token,
		SpeechWSPort:    util.Select(sm.tts != nil, sm.httpPort, 0),
	}
	result.PruneForClient()

	return nil
}

// PruneForClient tidies the NewSimResult, removing fields that are not used by client code
// in order to reduce the amount of bandwidth used to send the NewSimResult to the client.
func (r *NewSimResult) PruneForClient() {
	r.SimState = deep.MustCopy(r.SimState)

	for _, ap := range r.SimState.Airports {
		ap.Departures = nil
	}
}

type ConnectResult struct {
	Configurations map[string]map[string]*Configuration
	RunningSims    map[string]*RemoteSim
	HaveTTS        bool
}

const ConnectRPC = "SimManager.Connect"

func (sm *SimManager) Connect(version int, result *ConnectResult) error {
	if version != ViceRPCVersion {
		return ErrRPCVersionMismatch
	}

	// Before we acquire the lock...
	if err := sm.GetRunningSims(0, &result.RunningSims); err != nil {
		return err
	}

	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	result.Configurations = sm.configs
	result.HaveTTS = sm.tts != nil

	return nil
}

const GetAvailableWXRPC = "SimManager.GetAvailableWX"

func (sm *SimManager) GetAvailableWX(unused int, result *[]util.TimeInterval) error {
	if sm.wxProvider == nil {
		return ErrWeatherUnavailable
	}
	*result = sm.wxProvider.GetAvailableTimeIntervals()
	return nil
}

// assume SimManager lock is held
func (sm *SimManager) signOn(ss *simSession, config *SimConnectionConfiguration) (string, error) {
	_, err := ss.sim.SignOn(config.Position, config.Instructor, config.DisableTextToSpeech)
	if err != nil {
		return "", err
	}

	var buf [16]byte
	if _, err := crand.Read(buf[:]); err != nil {
		return "", err
	}
	token := base64.StdEncoding.EncodeToString(buf[:])

	return token, nil
}

func (sm *SimManager) SignOff(token string) error {
	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	return sm.signOff(token)
}

func (sm *SimManager) signOff(token string) error {
	ctrl, ok := sm.sessionsByToken[token]
	if !ok {
		return ErrNoSimForControllerToken
	}

	delete(sm.sessionsByToken, token)

	// Hold sm.mu while acquiring session.mu to maintain consistent lock ordering
	// This ensures we always acquire sm.mu before session.mu
	ctrl.session.SignOff(ctrl.tcp, sm.lg)

	// Call SignOff after releasing asim.mu but still holding sm.mu
	return ctrl.session.sim.SignOff(ctrl.tcp)
}

func (sm *SimManager) HandleSpeechWSConnection(w http.ResponseWriter, r *http.Request) {
	authHeader := r.Header.Get("Authorization")
	if !strings.HasPrefix(authHeader, "Bearer ") {
		http.Error(w, "Missing or invalid Authorization header", http.StatusUnauthorized)
		return
	}
	token := strings.TrimPrefix(authHeader, "Bearer ")

	sm.mu.Lock(sm.lg)

	ctrl, ok := sm.sessionsByToken[token]
	if !ok {
		sm.mu.Unlock(sm.lg)
		http.Error(w, "Unauthorized", http.StatusUnauthorized)
		sm.lg.Errorf("Invalid token for speech websocket: %s", token)
		return
	}
	tcp, session := ctrl.tcp, ctrl.session
	sm.mu.Unlock(sm.lg)

	session.HandleSpeechWSConnection(tcp, w, r, sm.lg)
}

const GetRunningSimsRPC = "SimManager.GetRunningSims"

func (sm *SimManager) GetRunningSims(_ int, result *map[string]*RemoteSim) error {
	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	running := make(map[string]*RemoteSim)
	for name, ss := range sm.simSessions {
		rs := &RemoteSim{
			GroupName:         ss.scenarioGroup,
			ScenarioName:      ss.scenario,
			PrimaryController: ss.sim.State.PrimaryController,
			RequirePassword:   ss.password != "",
		}

		rs.AvailablePositions, rs.CoveredPositions = ss.sim.GetAvailableCoveredPositions()

		running[name] = rs
	}

	*result = running
	return nil
}

func (sm *SimManager) LookupController(token string) (string, *sim.Sim, bool) {
	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	return sm.lookupController(token)
}

func (sm *SimManager) lookupController(token string) (string, *sim.Sim, bool) {
	if ctrl, ok := sm.sessionsByToken[token]; ok {
		return ctrl.tcp, ctrl.session.sim, true
	}
	return "", nil, false
}

const simIdleLimit = 4 * time.Hour

func (sm *SimManager) SimShouldExit(sim *sim.Sim) bool {
	if sim.IdleTime() < simIdleLimit {
		return false
	}

	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	nIdle := 0
	for _, ss := range sm.simSessions {
		if ss.sim.IdleTime() >= simIdleLimit {
			nIdle++
		}
	}
	return nIdle > 10
}

const GetSerializeSimRPC = "SimManager.GetSerializeSim"

func (sm *SimManager) GetSerializeSim(token string, s *sim.Sim) error {
	if _, sim, ok := sm.LookupController(token); !ok {
		return ErrNoSimForControllerToken
	} else {
		sm.mu.Lock(sm.lg)
		defer sm.mu.Unlock(sm.lg)

		*s = sim.GetSerializeSim()
	}
	return nil
}

func (sm *SimManager) GetStateUpdate(token string, update *sim.StateUpdate) error {
	sm.mu.Lock(sm.lg)

	if ctrl, ok := sm.sessionsByToken[token]; !ok {
		sm.mu.Unlock(sm.lg)
		return ErrNoSimForControllerToken
	} else {
		session := ctrl.session
		sim := session.sim
		tcp := ctrl.tcp

		sm.mu.Unlock(sm.lg)

		session.GotUpdateCallForTCP(tcp, sm.lg)
		sim.GetStateUpdate(tcp, update)

		return nil
	}
}

type SimBroadcastMessage struct {
	Password string
	Message  string
}

const BroadcastRPC = "SimManager.Broadcast"

func (sm *SimManager) Broadcast(m *SimBroadcastMessage, _ *struct{}) error {
	pw, err := os.ReadFile("password")
	if err != nil {
		return err
	}

	password := strings.TrimRight(string(pw), "\n\r")
	if password != m.Password {
		return ErrInvalidPassword
	}

	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	sm.lg.Infof("Broadcasting message: %s", m.Message)

	for _, ss := range sm.simSessions {
		ss.sim.PostEvent(sim.Event{
			Type:        sim.ServerBroadcastMessageEvent,
			WrittenText: m.Message,
		})
	}
	return nil
}

const GetAllVoicesRPC = "SimManager.GetAllVoices"

// GetAllVoices returns all available voices for TTS
func (sm *SimManager) GetAllVoices(_ struct{}, voices *[]sim.Voice) error {
	if sm.tts == nil {
		return fmt.Errorf("TTS not available")
	}

	fut := sm.tts.GetAllVoices()
	for {
		select {
		case v, ok := <-fut.VoicesCh:
			if ok {
				*voices = v
				return nil
			}
			fut.VoicesCh = nil // stop checking
		case err, ok := <-fut.ErrCh:
			if ok {
				return err
			}
			fut.ErrCh = nil
		}
	}
}

const TextToSpeechRPC = "SimManager.TextToSpeech"

// TextToSpeech converts text to speech and returns the audio data
func (sm *SimManager) TextToSpeech(req *TTSRequest, speechMp3 *[]byte) error {
	if sm.tts == nil {
		return fmt.Errorf("TTS not available")
	}

	if len(strings.Fields(req.Text)) > 50 {
		return fmt.Errorf("TTS capacity exceeded")
	}

	// Use ClientIP from the request (populated by LoggingServerCodec)
	clientIP := req.ClientIP
	if clientIP == "" {
		clientIP = "unknown"
	}

	if err := sm.UpdateTTSUsage(clientIP, req.Text); err != nil {
		return err
	}

	fut := sm.tts.TextToSpeech(req.Voice, req.Text)

	for {
		select {
		case mp3, ok := <-fut.Mp3Ch:
			if ok {
				*speechMp3 = mp3
				return nil
			}
			fut.Mp3Ch = nil // stop checking
		case err, ok := <-fut.ErrCh:
			if ok {
				return err
			}
			fut.ErrCh = nil // stop checking
		}
	}
}

const GetMETARRPC = "SimManager.GetMETAR"

func (sm *SimManager) GetMETAR(airports []string, result *map[string]wx.METARSOA) error {
	defer sm.lg.CatchAndReportCrash()

	if sm.wxProvider == nil {
		return ErrWeatherUnavailable
	}

	var err error
	*result, err = sm.wxProvider.GetMETAR(airports)
	return err
}

const GetTimeIntervalsRPC = "SimManager.GetTimeIntervals"

func (sm *SimManager) GetTimeIntervals(_ struct{}, result *[]util.TimeInterval) error {
	defer sm.lg.CatchAndReportCrash()

	if sm.wxProvider == nil {
		return ErrWeatherUnavailable
	}

	*result = sm.wxProvider.GetAvailableTimeIntervals()
	return nil
}

type PrecipURLArgs struct {
	TRACON string
	Time   time.Time
}

type PrecipURL struct {
	URL      string
	NextTime time.Time
}

const GetPrecipURLRPC = "SimManager.GetPrecipURL"

func (sm *SimManager) GetPrecipURL(args PrecipURLArgs, result *PrecipURL) error {
	defer sm.lg.CatchAndReportCrash()

	if sm.wxProvider == nil {
		return ErrWeatherUnavailable
	}

	var err error
	result.URL, result.NextTime, err = sm.wxProvider.GetPrecipURL(args.TRACON, args.Time)
	return err
}

type GetAtmosArgs struct {
	TRACON string
	Time   time.Time
}

type GetAtmosResult struct {
	AtmosByPointSOA *wx.AtmosByPointSOA
	Time            time.Time
	NextTime        time.Time
}

const GetAtmosGridRPC = "SimManager.GetAtmosGrid"

func (sm *SimManager) GetAtmosGrid(args GetAtmosArgs, result *GetAtmosResult) error {
	defer sm.lg.CatchAndReportCrash()

	if sm.wxProvider == nil {
		return ErrWeatherUnavailable
	}

	// Only load for TRACON scenarios
	if _, ok := av.DB.TRACONs[args.TRACON]; !ok {
		return nil
	}

	var err error
	result.AtmosByPointSOA, result.Time, result.NextTime, err = sm.wxProvider.GetAtmosGrid(args.TRACON, args.Time)
	return err
}

///////////////////////////////////////////////////////////////////////////
// TTS usage tracking

func (sm *SimManager) UpdateTTSUsage(ip, text string) error {
	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	if _, ok := sm.ttsUsageByIP[ip]; !ok {
		sm.ttsUsageByIP[ip] = &ttsUsageStats{}
	}

	stats := sm.ttsUsageByIP[ip]
	stats.Calls++
	stats.Words += len(strings.Fields(text))
	stats.LastUsed = time.Now()

	if stats.Words > 30000 {
		return fmt.Errorf("TTS capacity exceeded")
	}

	return nil
}

func (sm *SimManager) GetTTSStats() []ttsClientStats {
	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	var stats []ttsClientStats
	for ip, usage := range sm.ttsUsageByIP {
		stats = append(stats, ttsClientStats{
			IP:       ip,
			Calls:    usage.Calls,
			Words:    usage.Words,
			LastUsed: usage.LastUsed,
		})
	}
	return stats
}

///////////////////////////////////////////////////////////////////////////
// Status / statistics via HTTP...

func (sm *SimManager) launchHTTPServer() int {
	mux := http.NewServeMux()

	mux.HandleFunc("/sup", func(w http.ResponseWriter, r *http.Request) {
		sm.statsHandler(w, r)
		sm.lg.Infof("%s: served stats request", r.URL.String())
	})

	mux.HandleFunc("/speech", sm.HandleSpeechWSConnection)

	mux.HandleFunc("/debug/pprof/", pprof.Index)
	mux.HandleFunc("/debug/pprof/cmdline", pprof.Cmdline)
	mux.HandleFunc("/debug/pprof/profile", pprof.Profile)
	mux.HandleFunc("/debug/pprof/symbol", pprof.Symbol)
	mux.HandleFunc("/debug/pprof/trace", pprof.Trace)

	var listener net.Listener
	var err error
	var port int
	for i := range 10 {
		port = ViceHTTPServerPort + i
		if listener, err = net.Listen("tcp", ":"+strconv.Itoa(port)); err == nil {
			sm.httpPort = port
			fmt.Printf("Launching HTTP server on port %d\n", port)
			break
		}
	}

	if err != nil {
		sm.lg.Warnf("Unable to start HTTP server")
		return 0
	} else {
		go http.Serve(listener, mux)

		sm.httpPort = port
		return port
	}
}

type ttsClientStats struct {
	IP       string
	Calls    int
	Words    int
	LastUsed time.Time
}

type serverStats struct {
	Uptime           time.Duration
	AllocMemory      uint64
	TotalAllocMemory uint64
	SysMemory        uint64
	RX, TX           int64
	TXWebsocket      int64
	NumGC            uint32
	NumGoRoutines    int
	CPUUsage         int

	SimStatus []simStatus
	TTSStats  []ttsClientStats
}

func formatBytes(v int64) string {
	if v < 1024 {
		return fmt.Sprintf("%d B", v)
	} else if v < 1024*1024 {
		return fmt.Sprintf("%d KiB", v/1024)
	} else if v < 1024*1024*1024 {
		return fmt.Sprintf("%d MiB", v/1024/1024)
	} else {
		return fmt.Sprintf("%d GiB", v/1024/1024/1024)
	}
}

type simStatus struct {
	Name               string
	Config             string
	IdleTime           time.Duration
	Controllers        string
	TotalIFR, TotalVFR int
}

func (ss simStatus) LogValue() slog.Value {
	return slog.GroupValue(
		slog.String("name", ss.Name),
		slog.String("config", ss.Config),
		slog.Duration("idle", ss.IdleTime),
		slog.String("controllers", ss.Controllers),
		slog.Int("total_ifr", ss.TotalIFR),
		slog.Int("total_vfr", ss.TotalVFR))
}

func (sm *SimManager) GetSimStatus() []simStatus {
	sm.mu.Lock(sm.lg)
	defer sm.mu.Unlock(sm.lg)

	var status []simStatus
	for _, name := range util.SortedMapKeys(sm.simSessions) {
		ss := sm.simSessions[name]
		status = append(status, simStatus{
			Name:        name,
			Config:      ss.scenario,
			IdleTime:    ss.sim.IdleTime().Round(time.Second),
			TotalIFR:    ss.sim.State.TotalIFR,
			TotalVFR:    ss.sim.State.TotalVFR,
			Controllers: strings.Join(ss.sim.ActiveControllers(), ", "),
		})
	}

	return status
}

var templateFuncs = template.FuncMap{"bytes": formatBytes}

var statsTemplate = template.Must(template.New("").Funcs(templateFuncs).Parse(`
<!DOCTYPE html>
<html>
<head>
<title>vice vice baby</title>
</head>
<style>
table {
  border-collapse: collapse;
  width: 100%;
}

th, td {
  border: 1px solid #dddddd;
  padding: 8px;
  text-align: left;
}

tr:nth-child(even) {
  background-color: #f2f2f2;
}

#log {
    font-family: "Courier New", monospace;  /* use a monospace font */
    width: 100%;
    height: 500px;
    font-size: 12px;
    overflow: auto;  /* add scrollbars as necessary */
    white-space: pre-wrap;  /* wrap text */
    border: 1px solid #ccc;
    padding: 10px;
}
</style>
<body>
<h1>Server Status</h1>
<ul>
  <li>Uptime: {{.Uptime}}</li>
  <li>CPU usage: {{.CPUUsage}}%</li>
  <li>Bandwidth: {{bytes .RX}} RX, {{bytes .TX}} TX, {{bytes .TXWebsocket}} TX Websocket</li>
  <li>Allocated memory: {{.AllocMemory}} MB</li>
  <li>Total allocated memory: {{.TotalAllocMemory}} MB</li>
  <li>System memory: {{.SysMemory}} MB</li>
  <li>Garbage collection passes: {{.NumGC}}</li>
  <li>Running goroutines: {{.NumGoRoutines}}</li>
</ul>

<h1>Sim Status</h1>
<table>
  <tr>
  <th>Name</th>
  <th>Scenario</th>
  <th>IFR</th>
  <th>VFR</th>
  <th>Idle Time</th>
  <th>Active Controllers</th>

{{range .SimStatus}}
  </tr>
  <td>{{.Name}}</td>
  <td>{{.Config}}</td>
  <td>{{.TotalIFR}}</td>
  <td>{{.TotalVFR}}</td>
  <td>{{.IdleTime}}</td>
  <td><tt>{{.Controllers}}</tt></td>
</tr>
{{end}}
</table>

<h1>Text-to-Speech Usage</h1>
{{if .TTSStats}}
<table>
  <tr>
  <th>Client IP</th>
  <th>Call Count</th>
  <th>Word Count</th>
  <th>Last Used</th>
  </tr>
{{range .TTSStats}}
  <tr>
  <td>{{.IP}}</td>
  <td>{{.Calls}}</td>
  <td>{{.Words}}</td>
  <td>{{.LastUsed.Format "2006-01-02 15:04:05"}}</td>
  </tr>
{{end}}
</table>
{{else}}
<p>No TTS usage recorded.</p>
{{end}}

</body>
</html>
`))

func (sm *SimManager) statsHandler(w http.ResponseWriter, r *http.Request) {
	var m runtime.MemStats
	runtime.ReadMemStats(&m)

	usage, _ := cpu.Percent(time.Second, false)
	stats := serverStats{
		Uptime:           time.Since(sm.startTime).Round(time.Second),
		AllocMemory:      m.Alloc / (1024 * 1024),
		TotalAllocMemory: m.TotalAlloc / (1024 * 1024),
		SysMemory:        m.Sys / (1024 * 1024),
		NumGC:            m.NumGC,
		NumGoRoutines:    runtime.NumGoroutine(),
		CPUUsage:         int(gomath.Round(usage[0])),
		TXWebsocket:      sm.websocketTXBytes.Load(),

		SimStatus: sm.GetSimStatus(),
		TTSStats:  sm.GetTTSStats(),
	}

	stats.RX, stats.TX = util.GetLoggedRPCBandwidth()

	statsTemplate.Execute(w, stats)
}
