// config_test.go
// Copyright(c) 2022-2024 vice contributors, licensed under the GNU Public License, Version 3.
// SPDX: GPL-3.0-only

package main

import (
	"testing"

	"github.com/mmp/vice/panes"
	"github.com/mmp/vice/panes/eram"
	"github.com/mmp/vice/panes/stars"
)

func TestBuildDisplayRoot(t *testing.T) {
	config := &ConfigNoSim{
		STARSPane:        stars.NewSTARSPane(),
		ERAMPane:         eram.NewERAMPane(),
		MessagesPane:     panes.NewMessagesPane(),
		FlightStripPane:  panes.NewFlightStripPane(),
		SplitLinePositions: [2]float32{0.8, 0.075},
	}

	// Test with STARS pane
	starsRoot := config.buildDisplayRoot(config.STARSPane)
	if starsRoot == nil {
		t.Fatal("buildDisplayRoot returned nil for STARS pane")
	}
	if starsRoot.SplitLine.Axis != panes.SplitAxisX {
		t.Errorf("Expected SplitAxisX, got %v", starsRoot.SplitLine.Axis)
	}
	if starsRoot.SplitLine.Pos != 0.8 {
		t.Errorf("Expected X split position 0.8, got %f", starsRoot.SplitLine.Pos)
	}
	if starsRoot.Children[0] == nil {
		t.Fatal("Expected left child to exist")
	}
	if starsRoot.Children[0].SplitLine.Axis != panes.SplitAxisY {
		t.Errorf("Expected SplitAxisY, got %v", starsRoot.Children[0].SplitLine.Axis)
	}
	if starsRoot.Children[0].SplitLine.Pos != 0.075 {
		t.Errorf("Expected Y split position 0.075, got %f", starsRoot.Children[0].SplitLine.Pos)
	}

	// Verify the panes are correctly placed
	if starsRoot.Children[0].Children[0].Pane != config.MessagesPane {
		t.Error("MessagesPane not in expected position")
	}
	if starsRoot.Children[0].Children[1].Pane != config.STARSPane {
		t.Error("STARSPane not in expected position")
	}
	if starsRoot.Children[1].Pane != config.FlightStripPane {
		t.Error("FlightStripPane not in expected position")
	}

	// Test with ERAM pane
	eramRoot := config.buildDisplayRoot(config.ERAMPane)
	if eramRoot == nil {
		t.Fatal("buildDisplayRoot returned nil for ERAM pane")
	}
	if eramRoot.Children[0].Children[1].Pane != config.ERAMPane {
		t.Error("ERAMPane not in expected position")
	}
}

func TestGetDefaultConfig(t *testing.T) {
	config := getDefaultConfig()
	
	if config.STARSPane == nil {
		t.Error("STARSPane not initialized in default config")
	}
	if config.ERAMPane == nil {
		t.Error("ERAMPane not initialized in default config")
	}
	if config.MessagesPane == nil {
		t.Error("MessagesPane not initialized in default config")
	}
	if config.FlightStripPane == nil {
		t.Error("FlightStripPane not initialized in default config")
	}
	if config.SplitLinePositions[0] != 0.8 {
		t.Errorf("Expected default X split position 0.8, got %f", config.SplitLinePositions[0])
	}
	if config.SplitLinePositions[1] != 0.075 {
		t.Errorf("Expected default Y split position 0.075, got %f", config.SplitLinePositions[1])
	}
}