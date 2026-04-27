// sim/videomaps.go
// Copyright(c) 2022-2026 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package sim

import (
	"bytes"
	"encoding/gob"
	"errors"
	"fmt"
	"io"
	"io/fs"
	"path/filepath"
	"slices"
	"sort"
	"strings"

	av "github.com/mmp/vice/aviation"
	"github.com/mmp/vice/math"
	"github.com/mmp/vice/util"

	"github.com/klauspost/compress/zstd"
)

// Note: this should match ViceMapSpec/VideoMap in crc2vice/dat2vice. (crc2vice
// doesn't support all of these, though.)
type VideoMap struct {
	Label       string // for DCB
	Group       int    // 0 -> A, 1 -> B
	Name        string // For maps system list
	Id          int
	Category    int
	Restriction struct {
		Id        int
		Text      [2]string
		TextBlink bool
		HideText  bool
	}
	Color int
	Lines [][]math.Point2LL
}

// This should match VideoMapLibrary in dat2vice
type VideoMapLibrary struct {
	Maps          []VideoMap
	ERAMMapGroups ERAMMapGroups
}

type ERAMMap struct {
	BcgName    string
	LabelLine1 string
	LabelLine2 string
	Name       string
	Lines      [][]math.Point2LL
}

type ERAMMapGroup struct {
	Maps       []ERAMMap
	LabelLine1 string
	LabelLine2 string
}

type ERAMMapGroups map[string]ERAMMapGroup

// VideoMapManifest stores which maps are available in a video map file and
// is also able to provide the video map file's hash.
type VideoMapManifest struct {
	names      map[string]any
	filesystem fs.FS
	filename   string
}

func CheckVideoMapManifest(filename string, e *util.ErrorLogger) {
	defer e.CheckDepth(e.CurrentDepth())

	if strings.Contains(filename, "eram") {
		return // ERAM manifest not here
	}

	manifest, err := LoadVideoMapManifest(filename)
	if err != nil {
		e.Error(err)
		return
	}

	vms, err := LoadVideoMapLibrary(filename)
	if err != nil {
		e.Error(err)
		return
	}

	for n := range manifest.names {
		if !slices.ContainsFunc(vms.Maps, func(v VideoMap) bool { return v.Name == n }) {
			e.ErrorString("%s: map is in manifest file but not video map file", n)
		}
	}
	for _, m := range vms.Maps {
		if _, ok := manifest.names[m.Name]; !ok {
			e.ErrorString("%s: map is in video map file but not manifest", m.Name)
		}
	}
}

func LoadVideoMapManifest(filename string) (*VideoMapManifest, error) {
	filesystem := videoMapFS(filename)

	// Load the manifest and do initial error checking
	mf, _ := strings.CutSuffix(filename, ".zst")
	mf, _ = strings.CutSuffix(mf, "-videomaps.gob")
	mf += "-manifest.gob"

	fm, err := filesystem.Open(mf)
	if err != nil {
		return nil, err
	}
	defer fm.Close()

	var names map[string]any
	dec := gob.NewDecoder(fm)
	if err := dec.Decode(&names); err != nil {
		return nil, err
	}

	// Make sure the file exists but don't load it until it's needed.
	f, err := filesystem.Open(filename)
	if err != nil {
		return nil, err
	} else {
		f.Close()
	}

	return &VideoMapManifest{
		names:      names,
		filesystem: filesystem,
		filename:   filename,
	}, nil
}

func (v VideoMapManifest) HasMap(s string) bool {
	for i, m := range v.names {
		if i == s {
			return true
		}
		if names, ok := m.([]string); ok {
			if slices.Contains(names, s) {
				return true
			}
		}
	}
	return false
}

func (v VideoMapManifest) HasMapGroup(s string) bool {
	for i := range v.names {
		if i == s {
			return true
		}
	}
	return false
}

// Hash returns a hash of the underlying video map file (i.e., not the manifest!)
func (v VideoMapManifest) Hash() ([]byte, error) {
	if f, err := v.filesystem.Open(v.filename); err == nil {
		defer f.Close()
		return util.Hash(f)
	} else {
		return nil, err
	}
}

func LoadVideoMapLibrary(path string) (*VideoMapLibrary, error) {
	filesystem := videoMapFS(path)
	f, err := filesystem.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	contents, err := io.ReadAll(f)
	if err != nil {
		return nil, err
	}

	var r io.Reader
	br := bytes.NewReader(contents)
	var zr *zstd.Decoder
	if len(contents) > 4 && contents[0] == 0x28 && contents[1] == 0xb5 && contents[2] == 0x2f && contents[3] == 0xfd {
		// zstd compressed
		zr, _ = zstd.NewReader(br, zstd.WithDecoderConcurrency(0))
		defer zr.Close()
		r = zr
	} else {
		r = br
	}

	// Decode the gobfile.
	var vmf VideoMapLibrary
	if err := gob.NewDecoder(r).Decode(&vmf); err != nil {
		// Try the old format, just an array of maps
		// Create a new reader to avoid racing with the zstd decoder goroutine
		br = bytes.NewReader(contents)
		if zr != nil {
			_ = zr.Reset(br)
		} else {
			r = br
		}
		if strings.Contains(path, "eram") {
			if vmf.ERAMMapGroups == nil {
				vmf.ERAMMapGroups = make(ERAMMapGroups)
			}
			if err := gob.NewDecoder(r).Decode(&vmf.ERAMMapGroups); err != nil {
				return nil, err
			}
		} else {
			if err := gob.NewDecoder(r).Decode(&vmf.Maps); err != nil {
				return nil, err
			}
		}
	}

	return &vmf, nil
}

// Loads the specified video map file, though only if its hash matches the
// provided hash. Returns an error otherwise.
func HashCheckLoadVideoMap(path string, wantHash []byte) (*VideoMapLibrary, error) {
	filesystem := videoMapFS(path)
	f, err := filesystem.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()

	if gotHash, err := util.Hash(f); err != nil {
		return nil, err
	} else if !slices.Equal(gotHash, wantHash) {
		return nil, errors.New("hash mismatch")
	}

	return LoadVideoMapLibrary(path)
}

// Returns an fs.FS that allows us to load the video map with the given path.
func videoMapFS(path string) fs.FS {
	if filepath.IsAbs(path) {
		return util.RootFS{}
	} else {
		return util.GetResourcesFS()
	}
}

func PrintVideoMaps(path string, e *util.ErrorLogger) {
	if vmf, err := LoadVideoMapLibrary(path); err != nil {
		e.Error(err)
		return
	} else {
		sort.Slice(
			vmf.Maps, func(i, j int) bool {
				vi, vj := vmf.Maps[i], vmf.Maps[j]
				if vi.Id != vj.Id {
					return vi.Id < vj.Id
				}
				return vi.Name < vj.Name
			},
		)

		fmt.Printf("%5s\t%20s\t%s\n", "Id", "Label", "Name")
		for _, m := range vmf.Maps {
			fmt.Printf("%5d\t%20s\t%s\n", m.Id, m.Label, m.Name)
		}
	}
}

// GetControllerVideoMaps returns the video map configuration for the given TCW.
// Priority: controller-specific config > area-level config > facility-level.
func (s *Sim) GetControllerVideoMaps(tcw TCW) (videoMaps, defaultMaps []string, beaconCodes []av.Squawk) {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	fa := &s.State.FacilityAdaptation

	tcp := s.State.PrimaryPositionForTCW(tcw)

	// First check controller-specific config.
	if config, ok := fa.Controllers[tcp]; ok && len(config.VideoMapNames) > 0 {
		return config.VideoMapNames, config.DefaultMaps, config.MonitoredBeaconCodeBlocks
	}

	// Fall back to area-level config.
	if ctrl, ok := s.ControlPositions[tcp]; ok && ctrl.Area != "" {
		if ac, ok := fa.Areas[ctrl.Area]; ok && len(ac.VideoMapNames) > 0 {
			dm := s.State.ScenarioDefaultVideoMaps
			if len(dm) == 0 {
				dm = ac.DefaultMaps
			}
			return ac.VideoMapNames, dm, ac.MonitoredBeaconCodeBlocks
		}
	}

	return nil, s.State.ScenarioDefaultVideoMaps, fa.MonitoredBeaconCodeBlocks
}

// GetControllerVideoMapFile returns the effective video map file for
// the given TCW by resolving controller > area > facility priority.
func (s *Sim) GetControllerVideoMapFile(tcw TCW) string {
	s.mu.Lock(s.lg)
	defer s.mu.Unlock(s.lg)

	fa := &s.State.FacilityAdaptation
	tcp := s.State.PrimaryPositionForTCW(tcw)

	// Check controller-specific video_map_file first.
	if config, ok := fa.Controllers[tcp]; ok && config.VideoMapFile != "" {
		return config.VideoMapFile
	}

	if ctrl, ok := s.ControlPositions[tcp]; ok {
		return fa.VideoMapFileForArea(ctrl.Area)
	}
	return fa.VideoMapFile
}
